#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <heatmap_util/RvizFrissAction.h>

using namespace grid_map;

ros::Publisher publisher;
GridMap map({"elevation"});
int res;

void heatmapCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  int16_t world_sz = sqrt(msg->data.size()); // Square maps!
  map.setFrameId("map");
  map.setGeometry(Length(world_sz, world_sz), res);

  // Fill gridmap
  int i = 0;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = msg->data[i];
    i++;
  }

  // Publish gridmap
  ros::Time time = ros::Time::now();
  map.setTimestamp(time.toNSec());

  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);

  publisher.publish(message);
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");

  // Parameter extraction.
  if (ros::param::has("/resolution")) 
    nh.getParam("/resolution", res);
  else
    ROS_WARN("parameter not found...");

  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Subscriber heatmap_sub = nh.subscribe("/heatmap/map", 10, heatmapCallback);
  ros::spin();
}