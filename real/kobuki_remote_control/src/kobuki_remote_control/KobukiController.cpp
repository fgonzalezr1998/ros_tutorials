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

#include "kobuki_remote_control/KobukiController.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <vector>

#define MAXLINEARVEL 0.5
#define MAXANGULARVEL 0.9

namespace kobuki_remote_control
{

KobukiController::KobukiController()
:
nh_("~")
{
  initParams();

  kobuki_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(kobuki_vel_topic_, 1);
}

void
KobukiController::step()
{
  if ((ros::Time::now() - last_detection_ts_).toSec() > 2.0)
    return;

  remote_controller::ControllerSt controller_st = ControllerState();

  ROS_INFO("x left: %f, y left: %f\n",
    controller_st.joystick_left.x_coord,
    controller_st.joystick_left.y_coord);
  ROS_INFO("x right: %f, y right: %f\n",
    controller_st.joystick_right.x_coord,
    controller_st.joystick_right.y_coord);
  ROS_INFO("------------");

  // Send propper commands to the kobuki:

  sendCommands(controller_st);
}

/* Private Methods */

void
KobukiController::sendCommands(remote_controller::ControllerSt controller_st)
{
  // Send velocity:

  sendVelocity(controller_st);
}

void
KobukiController::sendVelocity(remote_controller::ControllerSt controller_st)
{
  geometry_msgs::Vector3 linear_vel;
  geometry_msgs::Vector3 angular_vel;
  geometry_msgs::Twist vel;
  float ang;

  // Set linear vel:

  linear_vel.x = controller_st.joystick_right.y_coord * MAXLINEARVEL;

  // Set angular vel:

  ang = atan(
    controller_st.joystick_left.x_coord / controller_st.joystick_left.y_coord);
  ang = ang / (3.1416 / 2.0);

  // Now, ang is between -1.0 and 1.0

  angular_vel.z = ang * MAXANGULARVEL;
  if (std::isnan(angular_vel.z))
    angular_vel.z = 0.0;

  vel.linear = linear_vel;
  vel.angular = angular_vel;

  ROS_INFO("Linear: %f, Angular: %f\n", linear_vel.x, angular_vel.z);

  kobuki_vel_pub_.publish(vel);
}

void
KobukiController::initParams()
{
  kobuki_vel_topic_ = "/mobile_base/commands/velocity";

  nh_.param("kobuki_vel_topic", kobuki_vel_topic_, kobuki_vel_topic_);
}

};  // namespace kobuki_remote_control
