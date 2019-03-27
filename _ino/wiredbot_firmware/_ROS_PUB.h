//
// Copyright (c) 2019 Noumena - ROMI
// Created by Noumena on 24/03/19.
//
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>


std_msgs::Float32MultiArray pins_msg;

ros::Publisher pins_msg_pub("/arduino_A", &pins_msg);

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
    pins_msg.data[9] = 0.0;
    pins_msg.data[10] = 0.0;

  }
  
  public:
      ros::NodeHandle  nh;

      float ch_3;
      float laser_1;
      float laser_2;
      
    void ros_pub_setup(){
       Serial.begin(57600);
       pins_msg.data_length = 11;
       nh.initNode();
       nh.advertise(pins_msg_pub);
    }  
    
    void ros_pub_loop(){
       set_pin_msg_ros();
       pins_msg_pub.publish(&pins_msg);
       nh.spinOnce(); 
       delay(100);
    }  
};
