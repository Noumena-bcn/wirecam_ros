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
#include "motor_driver.h"

MotorDriver::MotorDriver() {

    motor_driver_test();
}

void MotorDriver::motor_driver_test() {
    ROS_INFO("test");
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_INFO("Velocity-> x: [%f], y: [%f], z: [%f]", msg->linear.x, msg->linear.y, msg->linear.z);
}

int main(int argc, char **argv) {
//    MotorDriver motor_driver;
    ros::init(argc, argv, "wiredbot_driver_motors");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 5, cmd_vel_callback);

    ros::spin();
    return 0;
}