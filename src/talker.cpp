/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher message_a_pub = n.advertise<std_msgs::String>("MessageA", 1000);
  ros::Publisher message_b_pub = n.advertise<std_msgs::String>("MessageB", 1000);

  int message_a_count = 0;
  int message_b_count = 0;

  while (0 == message_a_pub.getNumSubscribers() || 0 == message_b_pub.getNumSubscribers()) {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(0.1).sleep();
  }

  ros::Timer timer_a = n.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Messages A: " << message_a_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    message_a_pub.publish(msg);
    message_a_count++;
  });

  ros::Timer timer_b = n.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Messages B: " << message_b_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    message_b_pub.publish(msg);
    message_b_count++;
  });

  ros::spin();

  return 0;
}
