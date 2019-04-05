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
// 219 - 290 || 306

#define PERIOD_HZ 50
#define MOTOR_A_CHANNEL 0
#define MOTOR_B_CHANNEL 1
#define PWM_FULL_REVERSE_MIN 185 // 1ms/20ms * 4096 | 204 - 241
#define PWM_FULL_REVERSE_MAX 241 // 1ms/20ms * 4096 | 204 - 241

#define PWM_NEUTRAL 307      // 1.5ms/20ms * 4096
#define PWM_FULL_FORWARD 409 // 2ms/20ms * 4096 | 260

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

void servo_pulse(double pulse, int period_hz = 60) {
    double pulse_length;
    int duty_cycle = 4095 + 1;
    pulse += 1;

    pulse_length = 1000000.0;   // 1,000,000 us per second
    pulse_length /= period_hz;   // Hz
    ROS_INFO("%f us per period", pulse_length);
    pulse_length /= duty_cycle;  // 12 bits of resolution 4.88281, 4.06901
    ROS_INFO("%f us per bit", pulse_length);
    pulse *= 1000.0;
    pulse /= pulse_length;
    ROS_INFO("%f pulse", pulse);
//    return pulse;
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
        pca9685->setPWMFrequency(PERIOD_HZ);
        sleep(1);
        pca9685->setPWM(MOTOR_A_CHANNEL, 0, PWM_NEUTRAL);

        while (nh.ok()) {
            ROS_INFO("PWM_FULL_REVERSE_MIN: %i", PWM_FULL_REVERSE_MIN);
            pca9685->setPWM(MOTOR_A_CHANNEL, 0, PWM_FULL_REVERSE_MIN);
            sleep(2);
            servo_pulse(PWM_FULL_REVERSE_MIN, PERIOD_HZ);
//            double FORWARD = PWM_FULL_FORWARD;
//            double REVERSE = PWM_FULL_REVERSE;
//
//            while(REVERSE <= PWM_NEUTRAL){
//                ROS_INFO("PWM_FULL_REVERSE: %f", REVERSE);
//                pca9685->setPWM(MOTOR_A_CHANNEL, 0, (int)servo_pulse(REVERSE, PERIOD_HZ));
//                REVERSE = REVERSE + 1;
//                sleep(2);
//            }
//            while(FORWARD <= PWM_NEUTRAL*2){
//                ROS_INFO("PWM_FULL_FORWARD: %f", FORWARD);
//                pca9685->setPWM(MOTOR_A_CHANNEL, 0, (int)servo_pulse(FORWARD, PERIOD_HZ));
//                FORWARD = FORWARD + 1;
//                sleep(2);
//            }
        }
        ROS_INFO("PCA9685 Device Address: 0x%02X\n : CLOSE", pca9685->kI2CAddress);
        pca9685->setPWM(MOTOR_A_CHANNEL, 0, PWM_NEUTRAL);
        pca9685->closePCA9685();
    }

    return 0;
}