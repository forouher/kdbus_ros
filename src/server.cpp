#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <boost/interprocess/detail/config_begin.hpp>
#include <boost/interprocess/detail/workaround.hpp>
#include <ros/boost_container.h>
#include <boost/interprocess/managed_shared_memory.hpp>

int main(int argc, char **argv)
{
  boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "ros_test", 4000000000);

  ros::init(argc, argv, "shm_server");
  ROS_INFO("Created shared memory segment...");

  ros::spin();

  return 0;
}
