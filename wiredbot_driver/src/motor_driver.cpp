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

double pwm_signal = 0.0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_INFO("Velocity-> x: [%f], y: [%f], z: [%f]", msg->linear.x, msg->linear.y, msg->linear.z);
    pwm_signal = msg->linear.x;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wiredbot_driver_motors");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 5, cmd_vel_callback);

    PCA9685 *pca9685 = new PCA9685();

    int err = pca9685->openPCA9685();
    if (err < 0){
        ROS_ERROR("Error: %d", pca9685->error);
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;

        while (nh.ok()){
            ROS_INFO("pwm_signal: %f", pwm_signal) ;
//            pca9685->setPWM(0,0,servoMin) ;
            ros::spinOnce();
        }
        pca9685->closePCA9685();
    }

    ros::spin();
    return 0;
}