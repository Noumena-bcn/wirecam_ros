//
// Created by starsky on 26/03/19.
//

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "motor_driver.h"

MotorDriver::MotorDriver() {

    motor_driver_test();
}

void MotorDriver::motor_driver_test(){
    ROS_INFO("test");
}

void MotorDriver::cmd_vel_callback(const std_msgs::Int32::ConstPtr& msg){
    ROS_INFO("%d", msg->data);
}

int main(int argc, char **argv) {
    MotorDriver motor_driver;
    ros::init(argc, argv, "wiredbot_driver_motors");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, &MotorDriver::cmd_vel_callback, &motor_driver);

    ros::spin();
    return 0;
}