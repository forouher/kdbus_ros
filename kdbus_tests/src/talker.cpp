/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2014, Dariush Forouher
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/message_factory.h>

namespace kdbus_tests {

class talker : public nodelet::Nodelet
{
  ros::Publisher pub_;
  ros::Timer timer_;

  uint64_t size;
  double period;

  virtual void onInit();

  void timerCb(const ros::TimerEvent& event);

};

void talker::timerCb(const ros::TimerEvent& event)
{

    sensor_msgs::PointCloud2::Ptr m = ros::make_shared<sensor_msgs::PointCloud2>();

    m->data.resize(size);
    ros::Time begin = ros::Time::now();
    m->header.stamp = begin;
    pub_.publish(m);
}

void talker::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  int temp;
  nh.getParam("/talker/period", period);
  nh.getParam("/talker/size", temp);
  size = temp;

  ROS_INFO("Period=%f, size=%lu", period, size);

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("ptr", 1);
  timer_ = nh.createTimer(ros::Duration(period), &talker::timerCb, this);

}

}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kdbus_tests::talker,nodelet::Nodelet)
