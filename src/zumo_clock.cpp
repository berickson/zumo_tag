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

L3G gyro;

unsigned long start_millis;
void setup()
{
  encoders.init();
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  lcd.clear();
  lcd.print(F("Try to"));
  lcd.gotoXY(0, 1);
  lcd.print(F("turn me!"));
  start_millis = millis();
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
  int clock_degrees = -clock_seconds*360/60;
  
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of p and a derivative constant of d.
  float p = 30;
  float d = .06;
  const float turn_to_radians = 2. * M_PI / 180. / turnAngle1;
  float turn_radians = turnAngle * turn_to_radians;
  float turn_degrees = float(turnAngle) / float(turnAngle1);
  float p_error = clock_degrees - turn_degrees;
  while(p_error>180) {
    p_error -= 180;
  }
  while(p_error<-180) {
    p_error += 180;
  }
  float d_error = -turnRate;
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

  if(every_n_ms(last_loop_ms, loop_ms, 100)) {
    lcd.clear();
    lcd.print("X ");
    lcd.print(position_x);
    lcd.gotoXY(0, 1);
    lcd.print("Y ");
    lcd.print(position_y);
  }


  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

  

  //motors.setSpeeds(-turnSpeed, turnSpeed);
  last_loop_ms = loop_ms;
}

