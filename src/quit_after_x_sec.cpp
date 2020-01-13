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
#include <actionlib/server/simple_action_server.h>
#include <ros_ci_example_gh/QuitAction.h>

class Quit
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  actionlib::SimpleActionServer<ros_ci_example_gh::QuitAction> as_;
  std::string action_name_;
  int warn_remaining_;

  ros_ci_example_gh::QuitFeedback feedback_;
  ros_ci_example_gh::QuitResult result_;

public:
  Quit(ros::NodeHandle& nh, const std::string& name)
    : nh_(nh)
    , private_nh_("~")
    , as_(nh_, name, boost::bind(&Quit::executeCallback, this, _1), false)
    , action_name_(name)
  {
    private_nh_.param<int>("warn_remaining", warn_remaining_, 10);
    as_.start();
  }

  ~Quit()
  {
  }

  void executeCallback(const ros_ci_example_gh::QuitGoalConstPtr& goal)
  {
    ROS_INFO_STREAM(action_name_ << " started");

    feedback_.remaining = goal->duration;
    result_.result = -1;

    // start executing the action
    ros::Time t_start = ros::Time::now();
    ros::Time t_current = ros::Time::now();
    double elapsed = (t_start - t_current).toSec();
    ros::Rate rate(10);
    bool success = true;

    ROS_DEBUG_STREAM("Start time: " << t_start);

    while (ros::ok() && elapsed <= goal->duration)
    {
      t_current = ros::Time::now();
      elapsed = (t_current - t_start).toSec();

      int remaining = static_cast<int>(goal->duration - elapsed);
      feedback_.remaining = remaining;
      ROS_INFO_STREAM_THROTTLE(1, remaining << " seconds remaining ...");

      if (remaining < warn_remaining_)
      {
        ROS_WARN_ONCE("Almost out of time!");
      }

      ros::spinOnce();
      rate.sleep();
    }

    if (success)
    {
      result_.result = 0;
      ROS_INFO_STREAM(action_name_ << " succeeded");
    }

    as_.setSucceeded(result_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quit_after_x_sec");
  ros::NodeHandle nh;

  Quit qa(nh, "quit_action");
  ros::spin();

  return 0;
}