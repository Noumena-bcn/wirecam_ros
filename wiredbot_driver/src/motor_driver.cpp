//
// Created by starsky on 26/03/19.
//

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <wiredbot_driver/PWMPCA9685.h>

int MIN = 240;
int MAX = 463;
int _pwm_pulse = MIN;

double power = 0.00;
int pwm;
int duty_cycle = 4095;


// 8666 16000

int _max = 1950;
int servoMin = 120;
int servoMax = 720;


int map(int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
//    ROS_INFO("Velocity-> x: [%f], y: [%f], z: [%f]", msg->linear.x, msg->linear.y, msg->linear.z);
    _pwm_pulse = msg->linear.x;
}

double pulseUS(double pulse, int hz) {
    double pulseLength;

    pulseLength = 1000000;   // 1,000,000 us per second
    pulseLength /= hz;   // 60 Hz
    ROS_INFO("%fus per period", pulseLength);

    pulseLength /= 4096;  // 12 bits of resolution
    ROS_INFO("%fus per bit", pulseLength);
//    pulse *= 1000000;  // convert to us
//    ROS_INFO("%fus ", pulse);
    return pulseLength;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wiredbot_driver_motors");
//    ros::Rate loop_rate(10);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 5, cmd_vel_callback);

    PCA9685 *pca9685 = new PCA9685();

    int err = pca9685->openPCA9685();
    if (err < 0) {
        ROS_ERROR("Error: %d", pca9685->error);
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n : OPEN", pca9685->kI2CAddress);
        pca9685->setAllPWM(0, 0);
        pca9685->reset();
        pca9685->setPWMFrequency(50);

        while (nh.ok()) {
            if (power >= 1.00)
            {
                ROS_INFO("PCA9685 power : %f", power);
                pwm = -int(duty_cycle * power);
                ROS_INFO("PCA9685 pwm : %i", pwm);
                pca9685->setPWM(0, 0, pwm);
                power = power + 0.01;
                sleep(1);
            }

//            for (uint16_t pwm_pulse = MIN; pwm_pulse < MAX; pwm_pulse++) {
//                ROS_INFO("PCA9685 pwm : %i", pwm_pulse);
//                pca9685->setPWM(0, 0, pwm_pulse);
//                sleep(1);
//            }
//            sleep(1);
//
//            for (uint16_t pwm_pulse = MAX; pwm_pulse > MIN; pwm_pulse--)  {
//                ROS_INFO("PCA9685 pwm : %i", pwm_pulse);
//                pca9685->setPWM(0, 0, pwm_pulse);
//                sleep(1);
//            }
            ros::spinOnce();
        }
        ROS_INFO("PCA9685 Device Address: 0x%02X\n : CLOSE", pca9685->kI2CAddress);
        pca9685->setPWM(0, 0, 0);
        pca9685->closePCA9685();
    }

    return 0;
}