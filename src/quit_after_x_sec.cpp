/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Gregor Seljak
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * @file quit_after_x_sec.cpp
 * @author Gregor Seljak
 * @brief Program exits after specified ammount of time
 */

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quit_after_x_sec");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double duration = 0;
  private_nh.param<double>("duration", duration, 10);

  ROS_INFO("Start");
  ROS_INFO_STREAM("Program will exit after " << duration << " seconds");

  ros::Time t_start = ros::Time::now();
  ros::Time t_current = ros::Time::now();
  double elapsed = (t_start - t_current).toSec();
  ros::Rate rate(10);

  ROS_DEBUG_STREAM("Start time: " << t_start);

  while (ros::ok() && elapsed <= duration)
  {
    t_current = ros::Time::now();
    elapsed = (t_current - t_start).toSec();
    int remaining = static_cast<int>(duration - elapsed);
    ROS_INFO_STREAM_THROTTLE(1, remaining << " seconds remaining ...");

    if (remaining < 5)
    {
      ROS_WARN_ONCE("Almost out of time!");
    }

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("End");

  ros::shutdown();
  return 0;
}