//
// Copyright (c) 2019 Noumena - ROMI
// Created by Noumena on 24/03/19.
//
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray pins_msg;
ros::Publisher pins_msg_pub("/ardunio", &pins_msg);

class _ROS_PUB{
  private:
  
  void set_pin_msg_ros()  {
    pins_msg.data[0] = 0.0;
    pins_msg.data[1] = 0.0;
    pins_msg.data[2] = ch_3;
    pins_msg.data[3] = 0.0;
    pins_msg.data[4] = 0.0;
    pins_msg.data[5] = 0.0;
    pins_msg.data[6] = 0.0;
    pins_msg.data[7] = 0.0;
    pins_msg.data[8] = 0.0;
    pins_msg.data[9] = laser_1;
    pins_msg.data[10] = laser_2;

  }
  
  public:
      ros::NodeHandle  nh;

      double ch_3 = 0;
      double laser_1 = 0;
      double laser_2 = 0;
      
    void ros_pub_setup(){
       nh.initNode();
       nh.advertise(pins_msg_pub);
    }  
    
    void ros_pub_loop(){
       nh.initNode();
       set_pin_msg_ros();
       nh.advertise(pins_msg_pub);
       nh.spinOnce();
    }  
};
