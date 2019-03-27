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
  receiver.receiver_setup();
  _ros_pub.ros_pub_setup();
 }

 void loop(){
  receiver.receiver_loop();
  _ros_pub.ch_3 = receiver.DATA_CH3;
  _ros_pub.ros_pub_loop();
 }
