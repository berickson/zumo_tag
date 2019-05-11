#include <EEPROM.h>

#include "geometry.h"
#include <Arduino.h>
#include "kalman.h"
#include "speedometer.h"
#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

#define dads_car

const float feet_to_meters = 0.3048;
const float arena_width = 3 * feet_to_meters;
const float arena_height = 3 * feet_to_meters;
const float robot_diameter = 0.05; // for cushion, easiest to consider robot is circular even though it isn't
const float arena_min_x = -arena_width/2 + robot_diameter;
const float arena_max_x = arena_width/2 - robot_diameter;
const float arena_min_y = -arena_width/2 + robot_diameter;
const float arena_max_y = arena_width/2 - robot_diameter;

#ifdef dads_car    
      const float meters_per_encoder_tick = 280./2112000 * 75./50.;
#else
      const float meters_per_encoder_tick = 280./2112000;
#endif


Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
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
    uint32_t stored_magic = 0;
    int address = 0;
    EEPROM.get(address, stored_magic);
    address += sizeof(stored_magic);
    if(stored_magic == magic) {
      EEPROM.get(address, gyro_cal);
    }
  }

  void save() {
    int address = 0;
    EEPROM.put(address, magic);
    address+= sizeof(magic);
    EEPROM.put(address, gyro_cal);
  }
};

Config config;

class Pose {
public:
  void update(int16_t new_ticks_left, int16_t new_ticks_right) {
    float yaw_radians = turn_sensor.get_yaw_radians();
    float ds = (new_ticks_left + new_ticks_right)*meters_per_encoder_tick/2;
    if(ds!=0) {
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

  void set_start_pose() {
    x = arena_max_x;
    y = arena_min_y;
    turn_sensor.reset();
  }


  float x = 0;
  float y = 0;
};


#define line_sensor_count 3

enum HiderSeeker {hider, seeker} role;


void lcd_print_bar(uint8_t height) {
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  lcd.print(barChars[height]);
}

bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}


// returns theta between -PI and PI
// optimized for small theta
double standardized_radians(double theta) {
  while(theta < - M_PI) {
    theta += 2*M_PI;
  }
  while(theta > M_PI) {
    theta  -= 2 * M_PI;
  }
  return theta;
}

class RobotController {
private:
  unsigned long last_execute_us;
  unsigned long us;
  float goal_x = NAN;
  float goal_y = NAN;
  float start_x = NAN; // where we started for current goal
  float start_y = NAN;
  float goal_yaw = NAN;

  const float max_speed = 400;
  const float min_speed = 100;
  const float goal_tolerance_meters = 0.03;
  const float turn_angle_tolerance = 10 * M_PI / 180;
  const float zero_angle_velocity_tolerance = 40 * M_PI / 180;

  struct R_Theta {
    float r;
    float theta;
  };

  bool turn_is_complete() {
    return 
        fabs(pose.get_yaw_radians_per_second()) < zero_angle_velocity_tolerance 
        && fabs(pose.get_yaw_radians() - goal_yaw) < turn_angle_tolerance;
  }

  void execute_move() {
    //lcd.clear();

    float elapsed = (us - last_execute_us) / 1000000.;

    float dx = goal_x - start_x;
    float dy = goal_y - start_y;
    float drx = pose.x - start_x;;
    float dry = pose.y - start_y;
    float l = sqrtf(dx*dx+dy*dy);

    float progress = (drx * dx + dry * dy)/(dx * dx + dy * dy);
    float cte = (drx * dy - dry * dx ) / l;

    float dgx = goal_x - pose.x;
    float dgy = goal_y - pose.y;
    float goal_distance = (1-progress)*l;// sqrt(dgx*dgx+dgy*dgy);
    //lcd.print(goal_distance);

    if(goal_distance < goal_tolerance_meters || progress >= 1) {
      Serial.println("move reached goal");
      motors.setSpeeds(0,0);
      mode = idle;
      return;
    }


    float k_p = 1000;
    float k_d = 500;
    float k_w =  15; // angular velocity

    float yaw_error = standardized_radians(goal_yaw - pose.get_yaw_radians());
    float w_error = -turn_sensor.get_yaw_radians_per_second();

    float adjust = k_p * cte + k_d * yaw_error + k_w * w_error;

    static double speed_setpoint = 0; // max speed at 5 cm

    ///////////////////
    const float max_accel = 6.0;
    const float stop_buffer = 0.0;

    float velocity = get_velocity();
    float safe_velocity = velocity_at_position(goal_distance-stop_buffer, max_accel, 0);
    float a_setpoint = 0;
    float v_setpoint = 0;
    //Serial.println("b1");
    if(velocity > safe_velocity) {
      //Serial.println("b2");
      //lcd.print("-");
      a_setpoint = -acceleration_for_distance_and_velocities(goal_distance, velocity, 0);
      v_setpoint = velocity_at_position(goal_distance, max_accel, 0);
    } else {
      //lcd.print("+");
      //Serial.println("b3");
      v_setpoint = velocity + max_accel * elapsed;
      a_setpoint = max_accel;
    }

    float v_error = v_setpoint - velocity;
    float a_error = a_setpoint - get_acceleration();
    float k_v = 1000;
    float k_a = 0;//1000;
    speed_setpoint += k_v * v_error + k_a * a_error;
    speed_setpoint = constrain(speed_setpoint, -max_speed, max_speed);

    if(0)
    {
    lcd.clear();
    lcd.print("v:");
    lcd.print(velocity);
    lcd.gotoXY(0,1);
    lcd.print("sp:");
    lcd.print(v_setpoint);
    }

/*
    Serial.print("velocity:");
    Serial.println(velocity);
    Serial.print("speed_setpoint: ");
    Serial.println(speed_setpoint);
    Serial.print("v_error:");
    Serial.println(v_error);
    Serial.print("a_error: ");
    Serial.println(a_error);
*/
    int left_speed = constrain(speed_setpoint - adjust, -max_speed,max_speed);
    int right_speed = constrain(speed_setpoint + adjust, -max_speed,max_speed);
    
    motors.setSpeeds(left_speed, right_speed);
  }

  void execute_turn() {
    float p = 500;
    float d = 30;
    float yaw_radians = pose.get_yaw_radians();

    float p_error = goal_yaw - yaw_radians;
    float d_error = -pose.get_yaw_radians_per_second();
    float turnSpeed = p*p_error+d*d_error;
    turnSpeed = constrain(turnSpeed, -max_speed/2, max_speed/2);
    if(0) {
    lcd.clear();
    lcd.print("t:");
    lcd.print(goal_yaw - pose.get_yaw_radians());
    }


    motors.setSpeeds(-turnSpeed, turnSpeed);
  }

  void execute_idle() {
    Serial.println("idle");
    motors.setSpeeds(0, 0);

  }
  

public:
  Speedometer left_speedometer;
  Speedometer right_speedometer;

  float get_velocity() {
    return (left_speedometer.get_velocity() + right_speedometer.get_velocity())/2.;
  }

  float get_acceleration() {
    return (left_speedometer.get_smooth_acceleration() + right_speedometer.get_smooth_acceleration())/2.;
  }

  Pose pose;

  RobotController() {
    left_speedometer.meters_per_tick = meters_per_encoder_tick;
    right_speedometer.meters_per_tick = meters_per_encoder_tick;
  }

  enum Mode {
    idle = 0,
    turning_before_move = 1,
    moving = 2,
    turning = 3,
    chasing = 4
  } mode;

  bool is_idle() {
    return mode == idle;
  }

  // returns yaw that points to x,y
  float direction_to(float target_x, float target_y) {
    float dx = target_x - pose.x;
    float dy = target_y - pose.y;
    if(dx==0 && dy==0) {
      return pose.get_yaw_radians();
    }
    return standardized_radians(atan2(dy,dx) - pose.get_yaw_radians()) + pose.get_yaw_radians();
  }

  void chase(int8_t direction) {
    mode = chasing;
    if(direction == 0) {
      motors.setSpeeds(max_speed, max_speed);
    } else if(direction == -1) {
      motors.setSpeeds(0, max_speed);
    }
    else if(direction == 1) {
      motors.setSpeeds(max_speed, 0);
    }
  }

  void set_position_goal(float goal_x, float goal_y) {
    Serial.println("position goal set");
    this->goal_x = goal_x;
    this->goal_y = goal_y;
    this->start_x = pose.x;
    this->start_y = pose.y;
    goal_yaw = direction_to(goal_x, goal_y);
    mode = turning_before_move;
  }

  float get_goal_x() {
    return this->goal_x;
  }
  float get_goal_y() {
    return this->goal_y;
  }

  void set_yaw_goal(float goal_yaw) {
    this->goal_yaw = goal_yaw;
    mode = turning;
  }

  void point_to_xy(float x, float y) {
    float dx = x - pose.x;
    float dy = y - pose.y;
    set_yaw_goal( standardized_radians(atan2(dy,dx) - pose.get_yaw_radians()) + pose.get_yaw_radians() );
    mode = turning;
  }

  void execute() {
    last_execute_us = us;
    us = micros();
    int16_t left_delta = encoders.getCountsAndResetLeft();
    int16_t right_delta = encoders.getCountsAndResetRight();
    // Serial.print("x: ");
    // Serial.print(pose.x);
    // Serial.print("y: ");
    // Serial.print(pose.y);
    // Serial.println();


    left_speedometer.add_ticks(us, left_delta);
    right_speedometer.add_ticks(us, right_delta);
    pose.update(left_delta, right_delta);
/*
    Serial.print("left delta:");
    Serial.print(left_delta);
    Serial.print("right delta:");
    Serial.print(right_delta);
    Serial.print("left v:");
    Serial.print(left_speedometer.get_velocity());
    Serial.print("right v:");
    Serial.print(right_speedometer.get_velocity());
    Serial.println();
*/
    if(mode == moving) {
      execute_move();
    } else if (mode == turning) {
      execute_turn();
      if(turn_is_complete()) {
        Serial.println("turn complete, going idle");
        mode = idle;
      }
    } else if (mode == turning_before_move) {
      //calculate_goal_yaw_and_distance();
      if(turn_is_complete()) {
        Serial.println("turn complete, moving");
        mode = moving;
      } else {
        execute_turn();
      }
    } else if (mode == chasing)  {
      // continue chasing
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
    lcd_print_bar(proximity_sensors.countsLeftWithLeftLeds());
    lcd_print_bar(proximity_sensors.countsLeftWithRightLeds());
    lcd_print_bar(proximity_sensors.countsFrontWithLeftLeds());
    lcd_print_bar(proximity_sensors.countsFrontWithRightLeds());
    lcd_print_bar(proximity_sensors.countsRightWithLeftLeds());
    lcd_print_bar(proximity_sensors.countsRightWithRightLeds());
    lcd.gotoXY(0, 1);
    for(int led=0;led<5;led++) {
      lcd_print_bar(proximity_sensors.countsWithLeftLeds(led));
      lcd_print_bar(proximity_sensors.countsWithRightLeds(led));
    }

}

void show_pose() {
    lcd.clear();
    lcd.print(robot.pose.x);
    lcd.gotoXY(0,1);
    lcd.print(robot.pose.y);
}

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

  loadCustomCharacters();

  // Display the angle (in degrees from -180 to 180) until the
  // user presses a button.
  lcd.clear();
  while (true)
  {
    turn_sensor.update();
    lcd.gotoXY(0, 0);
    lcd.print(turn_sensor.get_yaw_radians()*180/M_PI);
    lcd.print("deg  ");
    lcd.gotoXY(0,1);
    lcd.print("sk,hd,cl");
    if(buttonA.getSingleDebouncedRelease()) {
      role = seeker;
      break;
    }
    if(buttonB.getSingleDebouncedRelease()) {
      role = hider;
      break;
    }
    if(buttonC.getSingleDebouncedRelease()) {
        turn_sensor.calibrate();
        config.gyro_cal = turn_sensor.get_calibration();
        config.save();
        turn_sensor.reset();
    }
  }
  lcd.clear();


  robot.pose.set_start_pose();
}


void loop()
{
  static enum LoopMode {
    start = 0, 
    rush_to_center = 1, 
    sweep_scan = 2, 
    moving_to_point = 3, 
    chasing = 4, 
    hide_in_corner = 5, 
    hide_rush_to_corner = 6,
    done = 7
  } loop_mode = start;

  static enum ObstacleStatus {
    no_obstacle, obstacle_ahead, obstacle_right, obstacle_left
  } obstacle_status;

  static uint32_t obstacle_last_seen_ms = 0;

  static unsigned long loop_count = 0;
  static unsigned long last_loop_ms = 0;
  static bool look_for_targets = false;

  ++loop_count;
  unsigned long loop_ms = millis();
  

  // sense  
  turn_sensor.update();

  unsigned int line_readings[line_sensor_count];
  line_sensors.read(line_readings);


  proximity_sensors.read();
  {
    uint8_t left = proximity_sensors.countsFrontWithLeftLeds();
    uint8_t right = proximity_sensors.countsFrontWithRightLeds();
    if(every_n_ms(last_loop_ms, loop_ms, 10000)){
      Serial.print("Proximity ");
      Serial.print("FL: ");
      Serial.print(left);
      Serial.print("FR: ");
      Serial.print(right);
      Serial.println();
    }

    uint8_t sum = left+right;
    int8_t diff = right-left;


    if(sum<1) {
      obstacle_status = no_obstacle;
    } else {
      obstacle_last_seen_ms = loop_ms;
      if (diff < 0) {
        obstacle_status = obstacle_left;
      } else if (diff > 0) {
        obstacle_status = obstacle_right;
      } else {
        obstacle_status = obstacle_ahead;
      }
    }
  }

  // only left and center lines work,  
  bool left_line_sensed = line_readings[0] > 600;
  bool center_line_sensed = line_readings[1] > 600;
  bool right_line_sensed  = line_readings[2] > 600;

  Pose & pose = robot.pose;

  bool line_detected = left_line_sensed || center_line_sensed || right_line_sensed;
  if (line_detected) {
    buzzer.playNote(61,100,15); // root
    if(fabs(pose.x) > fabs(pose.y)) {
      pose.x = pose.x < 0 ? arena_min_x : arena_max_x;
    } else {
      pose.y = pose.y < 0 ? arena_min_y : arena_max_y;
    }
  }


  if(look_for_targets && (obstacle_status != no_obstacle)) {
    loop_mode = chasing;
    buzzer.playNote(68,100,15); // fifth
  }

  // stop everything and rush to center if we see a line, probably chasing a spectator
  if(line_detected && role==seeker &&  loop_mode!= start && loop_mode != rush_to_center) {
    loop_mode = start;
  }

  struct XY {
    XY(float x, float y) : x(x), y(y){}
      float x=NAN; 
      float y=NAN;
    };
  const XY arena_corners[] = {{arena_max_x, arena_min_y}, {arena_max_x, arena_max_y}, {arena_min_x, arena_max_y}, {arena_min_x, arena_min_y}};
  static uint8_t corner_index = 0;


  switch(loop_mode) {
    case start:
      look_for_targets = false;
      if(role==seeker) {
        robot.set_position_goal(0.,0.);
        loop_mode = rush_to_center;
      }
      if(role==hider) {
        robot.point_to_xy(0,0);
        loop_mode = hide_in_corner;
      }
      break;
    
    case hide_in_corner:
      if(robot.is_idle() && obstacle_status != ObstacleStatus::no_obstacle) {
        if(obstacle_status == obstacle_left) {
          corner_index = (corner_index==3) ? 0 : corner_index + 1;
        } else {
          corner_index = (corner_index==0) ? 3 : corner_index - 1;
        }
        loop_mode = hide_rush_to_corner;
        robot.set_position_goal(arena_corners[corner_index].x, arena_corners[corner_index].y);
      }
      break;

    case hide_rush_to_corner:
      if(robot.is_idle()) {
        robot.point_to_xy(0,0);
        loop_mode = hide_in_corner;
      }
      break;

    case chasing:
      if(obstacle_status == obstacle_ahead) {
        robot.chase(0);
      } else if (obstacle_status == obstacle_left) {
        robot.chase(-1);
      } else if (obstacle_status == obstacle_right) {
        robot.chase(1);
      } else {
        // give up after 1.0 seconds
        if(obstacle_last_seen_ms - loop_ms > 1000) {
          robot.set_position_goal(random_float(arena_min_x,arena_max_x),random_float(arena_min_y,arena_max_y));
          loop_mode = moving_to_point;
        }
      }
      break;

    case rush_to_center:
      if( robot.mode == RobotController::Mode::idle ) {
        robot.set_yaw_goal(pose.get_yaw_radians()+M_PI*2);
        loop_mode = sweep_scan;
        look_for_targets = true;
      }
      break;
    
    case sweep_scan:
      if(robot.mode == RobotController::Mode::idle)  {
        robot.set_position_goal(random_float(arena_min_x,arena_max_x),random_float(arena_min_y,arena_max_y));
        loop_mode = moving_to_point;
      }
      break;
    
    case moving_to_point:
      if(robot.mode == RobotController::Mode::idle) {
        robot.set_position_goal(random_float(arena_min_x,arena_max_x),random_float(arena_min_y,arena_max_y));
        loop_mode = moving_to_point;
      }
      break;

    case done:
      break;
  }



  robot.execute();

  if(every_n_ms(last_loop_ms, loop_ms, 10000)) {
    Serial.print("loop count: ");
    Serial.print(loop_count);
    /*
    Serial.print(" down sensors L:");
    Serial.print(line_readings[0]);
    Serial.print(" C:");
    Serial.print(line_readings[1]);
    */
    Serial.print("line readings:");
    for(int i =0;i<line_sensor_count;i++) {
      Serial.print(" [");
      Serial.print(i);
      Serial.print("]:");
      Serial.print(line_readings[i]);
      Serial.print("");
    }

    Serial.println();
  }


  // print status on led
  if(every_n_ms(last_loop_ms, loop_ms, 100)) {
    lcd.clear();
    lcd.print(loop_mode); 
    lcd.print(robot.mode);
    //show_sensor_bars();
  }

  last_loop_ms = loop_ms;
}


