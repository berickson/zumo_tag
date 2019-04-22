/* Turnsensor.h and TurnSensor.cpp provide functions for
configuring the L3GD20H gyro, calibrating it, and using it to
measure how much the robot has turned about its Z axis. */

#include <Wire.h>
#include "TurnSensor.h"


/* This should be called in setup() to enable and calibrate the
gyro.  It uses the LCD, yellow LED, and button A.  While the LCD
is displaying "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the L3GD20H gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void TurnSensor::init()
{
  Wire.begin();
  gyro.init();

  // 800 Hz output data rate,
  // low-pass filter cutoff 100 Hz
  gyro.writeReg(L3G::CTRL1, 0b11111111);

  // 2000 dps full scale
  gyro.writeReg(L3G::CTRL4, 0b00100000);

  // High-pass filter disabled
  gyro.writeReg(L3G::CTRL5, 0b00000000);
}

void TurnSensor::calibrate()
{
  lcd.clear();
  lcd.print(F("a: start"));
  while(!button.getSingleDebouncedRelease()) {
  }


  lcd.clear();
  lcd.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  const unsigned long cal_millis = 3000;
  unsigned long cal_start = millis();
  unsigned long n_samples = 0;
  while(millis() - cal_start < cal_millis)
  {
    // Wait for new data to be available, then read it.
    while(! (gyro.readReg(L3G::STATUS_REG) & 0x08) );
    gyro.read();

    // Add the Z axis reading to the total.
    total += gyro.g.z;
    ++n_samples;
  }
  ledYellow(0);
  yaw_zero_offset = (float)total / n_samples;
  Serial.print("n_samples: ");
  Serial.println(n_samples);
  Serial.print("yaw_zero_offset: ");
  Serial.println(yaw_zero_offset);


  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
  this->reset();
  while (!button.getSingleDebouncedRelease())
  {
    this->update();
    lcd.gotoXY(0, 0);
    lcd.print(get_yaw_radians()*180/M_PI);
    lcd.print(F("deg"));
    lcd.gotoXY(0,1);
    lcd.print("a: go");
  }
  lcd.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void TurnSensor::reset()
{
  last_update_us = micros();
  yaw_radians = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void TurnSensor::update()
{
  // Read the measurements from the gyro.
  gyro.read();
  unsigned long us = micros();

  auto reading = gyro.g.z - yaw_zero_offset;
  // Gyro readings are 1 unit = 0.07 deg / second
  yaw_radians_per_second = reading * 0.07 * M_PI / 180.;

  float dt = (us - this->last_update_us)/1E6;
  this->last_update_us = us;
  yaw_radians += yaw_radians_per_second * dt;
}

float TurnSensor::get_yaw_radians() {
  return yaw_radians;
}

float TurnSensor::get_yaw_radians_per_second() {
  return yaw_radians_per_second;
}
