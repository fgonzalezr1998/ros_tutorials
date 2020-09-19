/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#ifndef REMOTE_CONTROLLER__REMOTE_CONTROLLER_H
#define REMOTE_CONTROLLER__REMOTE_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>

namespace remote_controller
{

typedef struct JoyStickType JoyStickType;
struct JoyStickType
{
  float x_coord;
  float y_coord;
};

typedef struct ControllerSt ControllerSt;
struct ControllerSt
{
  JoyStickType joystick_left;
  JoyStickType joystick_right;
};

class RemoteController
{
public:
  RemoteController();

protected:
  ControllerSt ControllerState();

  ros::Time last_detection_ts_;

private:

  void initParams();
  void joystickCallback(const sensor_msgs::Joy::ConstPtr & msg);

  ControllerSt xboxOneState();

  ros::NodeHandle nh_;

  ros::Subscriber joystick_sub_;
  sensor_msgs::Joy controller_msg_;

  std::string joystick_topic_, controller_type_;
};

};  // namespace remote_controller

#endif  // REMOTE_CONTROLLER__REMOTE_CONTROLLER_H
