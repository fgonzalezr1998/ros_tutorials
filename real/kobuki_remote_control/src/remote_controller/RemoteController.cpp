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

#include "remote_controller/RemoteController.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <vector>

namespace remote_controller
{

RemoteController::RemoteController()
:
nh_("~")
{
  initParams();

  joystick_sub_ = nh_.subscribe(
    joystick_topic_, 1, &RemoteController::joystickCallback, this);
}

/* Protected Methods */

ControllerSt
RemoteController::ControllerState()
{
  if (controller_type_ == "xbox one")
    return xboxOneState();
}

/* Private Methods */

ControllerSt
RemoteController::xboxOneState()
{
  ControllerSt state;

  state.joystick_left.x_coord = controller_msg_.axes[0];
  state.joystick_left.y_coord = controller_msg_.axes[1];

  state.joystick_right.x_coord = controller_msg_.axes[3];
  state.joystick_right.y_coord = controller_msg_.axes[4];

  return state;
}

void
RemoteController::joystickCallback(const sensor_msgs::Joy::ConstPtr & msg)
{
  controller_msg_ = *msg;
  last_detection_ts_ = ros::Time::now();
}

void
RemoteController::initParams()
{
  controller_type_ = "xbox one";
  joystick_topic_ = "/joy_orig";

  nh_.param("controller_type", controller_type_, controller_type_);
  nh_.param("joystick_topic", joystick_topic_, joystick_topic_);
}

};  // namespace remote_controller
