/* Turnsensor.h and TurnSensor.cpp provide functions for
configuring the L3GD20H gyro, calibrating it, and using it to
measure how much the robot has turned about its Z axis. */

#pragma once

#include <Zumo32U4.h>



class TurnSensor {
public: 
    TurnSensor(
        Pushbutton * button,
        Zumo32U4LCD * lcd,
        L3G * gyro) :
        button(*button),
        lcd(*lcd),
        gyro(*gyro) 
        {}

    void init();
    void calibrate();
    void reset();
    void update();


    float get_yaw_radians();
    float get_yaw_radians_per_second();

private:
    float yaw_radians_per_second = 0;
    float yaw_radians = 0;
    float yaw_zero_offset = 0;
    unsigned long last_update_us = 0;

    Pushbutton & button;
    Zumo32U4LCD & lcd;
    L3G & gyro;

};
