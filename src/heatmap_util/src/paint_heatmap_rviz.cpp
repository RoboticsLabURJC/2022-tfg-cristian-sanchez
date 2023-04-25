#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <heatmap_util/RvizFrissAction.h>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  
  // Requesting map for Rviz to the action server
  actionlib::SimpleActionClient<heatmap_util::RvizFrissAction> heatmap_client("rviz_friss_action", true);
  ROS_INFO("Waiting for action server to start...");
  heatmap_client.waitForServer();
  ROS_INFO("Server started!");
  ROS_INFO("Sending get_data() to the server...");
  heatmap_util::RvizFrissGoal goal;
  goal.get_data = true;
  heatmap_client.sendGoal(goal);
  ROS_INFO("Sended! Waiting result...");
  heatmap_client.waitForResult();
  ROS_INFO("Result received!");

  if (heatmap_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    // Extract result from request
    const auto& result = heatmap_client.getResult()->data;
    std_msgs::Float64MultiArray data;
    data.data.insert(data.data.end(), result.begin(), result.end());
    int16_t world_sz = heatmap_client.getResult()->size;

    // Create gridmap
    GridMap map({"elevation"});
    map.setFrameId("map");    
    map.setGeometry(Length(world_sz, world_sz), 1);

    // Fill gridmap
    int i = 0;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = data.data[i];
      // ROS_INFO("Data: %f", data.data[i]);
      i++;
    }

    // Publish gridmap
    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
  } else {
    ROS_ERROR("Action failed: %s", heatmap_client.getState().toString().c_str());
    return 1;
  }
  return 0;
}