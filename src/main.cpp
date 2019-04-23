#include <EEPROM.h>

/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the LCD.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proximity_sensors;
Zumo32U4LineSensors line_sensors;
Zumo32U4Buzzer buzzer;

L3G gyro;

void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar
  lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars
  lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars
  lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars
  lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars
  lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars
  lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

TurnSensor turn_sensor(&buttonA, &lcd, &gyro);


struct Config {
public:
  // Denotes config version
  // Change every time you change this struct
  const uint32_t magic = 0xa719; 
  float gyro_cal = 0;
  
  void load() {
    Serial.println("Reading config");
    uint32_t stored_magic = 0;
    int address = 0;
    EEPROM.get(address, stored_magic);
    address += sizeof(stored_magic);
    Serial.print("read magic");
    Serial.print(stored_magic);
    Serial.println();
    if(stored_magic == magic) {
      Serial.println("magic matched");
      EEPROM.get(address, gyro_cal);
    }
    Serial.print("gyro_cal: ");
    Serial.print(gyro_cal);
    Serial.println();
  }

  void save() {
    Serial.println("Writing config");
    int address = 0;
    EEPROM.put(address, magic);
    address+= sizeof(magic);
    EEPROM.put(address, gyro_cal);

    Serial.print("gyro_cal: ");
    Serial.print(gyro_cal);
    Serial.println();
  }
};

Config config;

void setup()
{
  lcd.print("setup");
  config.load();
  proximity_sensors.initThreeSensors();
  line_sensors.initThreeSensors();
  encoders.init();
  turn_sensor.init();
  turn_sensor.reset();
  turn_sensor.set_calibration(config.gyro_cal);
  Serial.println(config.gyro_cal);

  lcd.clear();
  lcd.print("set2");
  loadCustomCharacters();

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
  while (true)
  {
    turn_sensor.update();
    lcd.gotoXY(0, 0);
    lcd.print(turn_sensor.get_yaw_radians()*180/M_PI);
    lcd.print("deg  ");
    lcd.gotoXY(0,1);
    lcd.print("go,cal");
    if(buttonB.getSingleDebouncedRelease()) {
        turn_sensor.calibrate();
        config.gyro_cal = turn_sensor.get_calibration();
        config.save();
    }
    if(buttonA.getSingleDebouncedRelease()) {
      break;
    }
  }
  lcd.clear();


  turn_sensor.reset();
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  lcd.print(barChars[height]);
}

bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

class Pose {
public:
  void update() {
    float yaw_radians = turn_sensor.get_yaw_radians();
      const float meters_per_encoder_tick = 280./2112000;
    float ds = (encoders.getCountsAndResetLeft()+encoders.getCountsAndResetRight())*meters_per_encoder_tick/2;
    if(ds!=0) {
      Serial.print("dsmm:");
      Serial.println(ds*1000);
      Serial.print("yaw radians:");
      Serial.println(yaw_radians);
      
      Serial.println(ds*1000);
      this->x += ds * cos(yaw_radians);
      this->y += ds * sin(yaw_radians);
    }
  }

  float get_yaw_radians() {
    return turn_sensor.get_yaw_radians();
  }

  float get_yaw_radians_per_second() {
    return turn_sensor.get_yaw_radians_per_second();
  }


  float x = 0;
  float y = 0;
} pose;


double standardized_radians(double theta) {
  return fmod((theta + 3 * M_PI), 2*M_PI) - M_PI;
}

class RobotController {
private:
  float goal_x = NAN;
  float goal_y = NAN;
  float goal_yaw = NAN;
  float goal_distance = NAN;

  const float goal_tolerance_meters = 0.03;
  const float turn_angle_tolerance = 5 * M_PI / 180;
  const float zero_angle_velocity_tolerance = 20 * M_PI / 180;

  struct R_Theta {
    float r;
    float theta;
  };

  void calculate_goal_yaw_and_distance() {
    float dx = goal_x - pose.x;
    float dy = goal_y - pose.y;
    goal_distance = sqrt(dx*dx+dy*dy);
    goal_yaw =  atan2(dy,dx);
  }

  bool turn_is_complete() {
    return 
        fabs(pose.get_yaw_radians_per_second()) < zero_angle_velocity_tolerance 
        && fabs(pose.get_yaw_radians() - goal_yaw) < turn_angle_tolerance;
  }

  

  void execute_move() {
    calculate_goal_yaw_and_distance();

    if(goal_distance < goal_tolerance_meters) {
      motors.setSpeeds(0,0);
      mode = idle;
      return;
    }
    float max_speed = 400;
    float min_speed = 100;

    double yaw_setpoint = goal_yaw;
    double speed_setpoint = min_speed+goal_distance * max_speed / 0.15; // max speed at 5 cm

    double yaw_error = standardized_radians(yaw_setpoint - pose.get_yaw_radians());

    int left_speed = constrain(speed_setpoint - 1000 * yaw_error,-max_speed,max_speed);
    int right_speed = constrain(speed_setpoint + 1000 * yaw_error,-max_speed,max_speed);
    
    motors.setSpeeds(left_speed, right_speed);
    // buzzer.playNote(57, 500, 10);
  }

  void execute_turn() {
    float p = 500;
    float d = 30;
    float yaw_radians = pose.get_yaw_radians();

    float p_error = goal_yaw - yaw_radians;
    float d_error = -pose.get_yaw_radians_per_second();
    float turnSpeed = p*p_error+d*d_error;
    turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

    motors.setSpeeds(-turnSpeed, turnSpeed);
  }

  void execute_idle() {
    motors.setSpeeds(0, 0);

  }

public:
  enum Mode {
    idle,
    turning_before_move,
    moving,
    turning
  } mode;

  void set_position_goal(float goal_x, float goal_y) {
    this->goal_x = goal_x;
    this->goal_y = goal_y;
    mode = turning_before_move;
  }

  void set_yaw_goal(float yaw_goal) {
    this->goal_yaw = goal_yaw;
    mode = turning;
  }

  void execute() {
    if(mode == moving) {
      execute_move();
    } else if (mode == turning) {
      execute_turn();
    } else if (mode == turning_before_move) {
      calculate_goal_yaw_and_distance();
      if(turn_is_complete()) {
        // buzzer.playNote(57, 500, 10);
        mode = moving;
      } else {
        execute_turn();
      }
    } else {
      execute_idle();
    }
  }

  float yaw_goal;
};

RobotController robot;

float random_float(float min, float max)
{
    float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;  
    return (random*range) + min;
}


void show_sensor_bars() {
    lcd.gotoXY(0, 0);
    printBar(proximity_sensors.countsLeftWithLeftLeds());
    printBar(proximity_sensors.countsLeftWithRightLeds());
    printBar(proximity_sensors.countsFrontWithLeftLeds());
    printBar(proximity_sensors.countsFrontWithRightLeds());
    printBar(proximity_sensors.countsRightWithLeftLeds());
    printBar(proximity_sensors.countsRightWithRightLeds());
    lcd.gotoXY(0, 1);
    for(int led=0;led<5;led++) {
      printBar(proximity_sensors.countsWithLeftLeds(led));
      printBar(proximity_sensors.countsWithLeftLeds(led));
    }
    Serial.print("turn_degrees: ");
    Serial.println(pose.get_yaw_radians()*180/M_PI);
}

void loop()
{
  static enum LoopMode {
    start, moving_to_point, chasing
  } loop_mode = start;

  static unsigned long loop_count = 0;
  static unsigned long last_loop_ms = 0;
  ++loop_count;
  unsigned long loop_ms = millis();

  // sense  
  turn_sensor.update();
  proximity_sensors.read();
  pose.update();

  switch(loop_mode) {
    case start:
      robot.set_position_goal(0.33,.33);
      loop_mode = moving_to_point;
      break;
    case moving_to_point:
      if(robot.mode == RobotController::Mode::idle) {
        robot.set_position_goal(random_float(0,1),random_float(0,1));
        loop_mode = moving_to_point;
      }
  }



  robot.execute();

  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    Serial.print("loop count: ");
    Serial.println(loop_count);
  }

  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    Serial.print("L: ");
    Serial.print(proximity_sensors.readBasicLeft());
    Serial.print(" R: ");
    Serial.print(proximity_sensors.readBasicRight());
  }

  if(every_n_ms(last_loop_ms, loop_ms, 100)) {

    lcd.clear();
    lcd.print(pose.x);
    lcd.gotoXY(0,1);
    lcd.print(pose.y);
  }

  last_loop_ms = loop_ms;
}


