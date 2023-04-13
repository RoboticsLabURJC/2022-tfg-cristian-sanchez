#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

#include "std_msgs/Float64MultiArray.h"

using namespace grid_map;

GridMap map({"elevation"});

void rf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{
  ROS_INFO("Entrando en el CALLBACK!...");
  int i = 0;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = msg->data[i];
    i++;
  }
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Subscriber sub = nh.subscribe("/rf_array", 1, rf_callback);

  // Create grid map.
  
  map.setFrameId("map");
  map.setGeometry(Length(20, 20), 1); // 100 x 100 cells
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  ROS_INFO("--------------");
  for (auto layer : map.getBasicLayers())
    ROS_INFO("layer: %s", layer.c_str());
  ROS_INFO("--------------");

    // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}