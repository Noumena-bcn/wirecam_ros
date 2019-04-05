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

#define MOTOR_CHANNEL 0
#define PWM_FULL_REVERSE 240 // 1ms/20ms * 4096
#define PWM_NEUTRAL 343      // 1.5ms/20ms * 4096
#define PWM_FULL_FORWARD 445 // 2ms/20ms * 4096

int MIN = 240;
int MAX = 445;
int pwm_pulse;


// 8666 16000

//int _max = 1950;
//int servoMin = 120;
//int servoMax = 720;


int map(int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
//    ROS_INFO("Velocity-> x: [%f], y: [%f], z: [%f]", msg->linear.x, msg->linear.y, msg->linear.z);
    pwm_pulse = msg->linear.x;
}

double servo_pulse(double pulse, int frequencyHz=60) {
    double pulse_length;
    int duty_cycle = 4095 + 1;

    pulse_length = 1000000.0;   // 1,000,000 us per second
    pulse_length /= frequencyHz;   // Hz
    ROS_ERROR("%f us per period", pulse_length);
    pulse_length /= duty_cycle;  // 12 bits of resolution 4.88281, 4.06901
    ROS_ERROR("%f us per bit", pulse_length);
    pulse *= 1000.0;
    pulse /= pulse_length;
    ROS_ERROR("%f pulse", pulse);
    return pulse;
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
        sleep(1);
        pca9685->setPWM(MOTOR_CHANNEL, 0, PWM_NEUTRAL);

        while (nh.ok()) {
            double _i_A = 0;
            while(_i_A <= 1.0){
                ROS_INFO("PWM_FULL_POWER: %f", _i_A);
                pca9685->setPWM(MOTOR_CHANNEL, 0, (int)servo_pulse(_i_A));
                _i_A = _i_A + 0.1;
                sleep(2);
            }
        }
        ROS_INFO("PCA9685 Device Address: 0x%02X\n : CLOSE", pca9685->kI2CAddress);
        pca9685->setPWM(0, 0, 0);
        pca9685->closePCA9685();
    }

    return 0;
}