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
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proximity_sensors;
Zumo32U4LineSensors line_sensors;

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

unsigned long start_millis;
void setup()
{
  proximity_sensors.initThreeSensors();
  line_sensors.initThreeSensors();
  encoders.init();
  turn_sensor.init();
  turn_sensor.calibrate();

  delay(500);
  turn_sensor.reset();

  loadCustomCharacters();

  lcd.clear();
  lcd.print(F("Try to"));
  lcd.gotoXY(0, 1);
  lcd.print(F("turn me!"));
  start_millis = millis();
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

void loop()
{
  static float position_x = 0;
  static float position_y = 0;

  const float meters_per_encoder_tick = 280./2112000;
  static unsigned long loop_count = 0;
  static unsigned long last_loop_ms = 0;
  ++loop_count;
  unsigned long loop_ms = millis();


  int clock_seconds= (loop_ms-start_millis)/1000;//%60;
  float clock_radians = -clock_seconds*360/60*M_PI/180;
  
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  turn_sensor.update();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of p and a derivative constant of d.
  float p = 1000;
  float d = 30;
  float turn_radians = turn_sensor.get_yaw_radians();

  float p_error = clock_radians - turn_radians;
  float d_error = -turn_sensor.get_yaw_radians_per_second();
  float turnSpeed = p*p_error+d*d_error;


  // update position
  float ds = (encoders.getCountsAndResetLeft()+encoders.getCountsAndResetRight())*meters_per_encoder_tick/2;
  if(ds!=0) {
    Serial.print("dsmm:");
    Serial.println(ds*1000);
    Serial.print("yaw radians:");
    Serial.println(turn_radians);
    
    Serial.println(ds*1000);
    position_x += ds * cos(turn_radians);
    position_y += ds * sin(turn_radians);
  }

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
    proximity_sensors.read();

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
    Serial.println(turn_radians*180/M_PI);

  }


  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

  

  motors.setSpeeds(-turnSpeed, turnSpeed);
  last_loop_ms = loop_ms;
}


