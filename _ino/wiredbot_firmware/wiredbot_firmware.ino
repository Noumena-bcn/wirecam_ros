////
//// Copyright (c) 2019 Noumena - ROMI
//// Created by Noumena on 24/03/19.
////
/*----------INCLUDES------------------*/
#include "RECEIVER_X8R.h"
#include "_ROS_PUB.h"
RECEIVER_X8R receiver;
_ROS_PUB _ros_pub;

/*----------SETUP------------------*/
 void setup(){
  Serial.begin(57600);
  receiver.receiver_setup();
  _ros_pub.ros_pub_setup();
 }

 void loop(){
  receiver.receiver_loop();
  _ros_pub.ch_3 = receiver.PWM_CH3;
  _ros_pub.ros_pub_loop();
 }

///*----------INCLUDES------------------*/
//#include <ros.h>
//#include <std_msgs/Int32.h>
///*----------VAR DEFINITIONS----------*/
//double ch3_pin_value;
//
//int DEBUG = 0;
//int old_value = 0;
//int new_value = 0;
//int pos = 0;
///*----------PIN DEFINITIONS----------*/
//const int ch3_pin = 13;
///*----------ROS----------------------*/
//ros::NodeHandle  nh;
//std_msgs::Int32 ch3_msg;
//ros::Publisher ch3_msg_pub("/teleop_ch3", &ch3_msg);
///*-----------------------------------*/
//
//void setup() {
//  Serial.begin(57600);
//  /*-----INIT PINS------------------*/
//  pinMode(ch3_pin, INPUT);
//  nh.initNode();
//  nh.advertise(ch3_msg_pub);
//  /*-----TEST-----------------------*/
//  /*--------------------------------*/
//  }
//
//void loop() {
//  ch3_pin_value = pulseIn(ch3_pin, HIGH, 25000);
////  ch3_pin_value = 1;
//
//  ch3_msg.data = ch3_pin_value;
//  ch3_msg_pub.publish( &ch3_msg );
//  Serial.println(ch3_msg.data);
//  nh.spinOnce();
//  delay(100);
//  }
