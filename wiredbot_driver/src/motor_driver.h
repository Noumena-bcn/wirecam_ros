//
// Created by starsky on 26/03/19.
//

#ifndef WIREDBOT_DRIVER_MOTOR_DRIVER_H
#define WIREDBOT_DRIVER_MOTOR_DRIVER_H

#define I2C_BUS 1
#define I2C_ADDRESS 0x40
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600
#define SERVO_PULSE_RANGE 4096

#define HS422_SERVO_MIN_PUL     150
#define HS422_SERVO_MAX_PUL     600
#define MOTOR_SERVO_MIN_PUL     200
#define MOTOR_SERVO_MAX_PUL     700

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
//#include <wiredbot_driver/PWMPCA9685.h>

class MotorDriver {
public:
    MotorDriver();
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
private:

//    PCA9685 *pca9685;

    void motor_driver_test();
    int map( int x, int in_min, int in_max, int out_min, int out_max);
    int get_key();

};

#endif //WIREDBOT_DRIVER_MOTOR_DRIVER_H
