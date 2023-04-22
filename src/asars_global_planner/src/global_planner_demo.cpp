// Not sure how to integrate this yet to the system

#include "ros/ros.h"

#include "asars_global_planner/GenerateVisitingOrder.h"

void print_list(std::vector<geometry_msgs::Point> list) {
  for (const auto &point : list) {
    ROS_INFO("X:%f Y: %f Z: %f", point.x, point.y, point.z);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_demo_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<asars_global_planner::GenerateVisitingOrder>(
          "asars/generate_visiting_order");
  asars_global_planner::GenerateVisitingOrder srv;

  geometry_msgs::Pose robot_location;

  // Hard coded locations for some random items in the husky playpen demo
  std::vector<geometry_msgs::Point> victim_locations;

  geometry_msgs::Point victim1_location;
  victim1_location.x = -3.2989;
  victim1_location.y = 2.497;
  victim_locations.push_back(victim1_location);

  geometry_msgs::Point victim2_location;
  victim2_location.x = -4.221;
  victim2_location.y = -4.158;
  victim_locations.push_back(victim2_location);

  geometry_msgs::Point victim3_location;
  victim3_location.x = 6.199;
  victim3_location.y = 5.3456;
  victim_locations.push_back(victim3_location);

  /*  geometry_msgs::Point victim4_location;
 victim_locations.push_back(victim4_location);

 geometry_msgs::Point victim5_location;
 victim_locations.push_back(victim5_location); */

  srv.request.victim_locations = victim_locations;
  // srv.request.robot_location = robot_location;

  if (client.call(srv)) {
    ROS_INFO("Client successfully called");

    print_list(srv.response.victim_visiting_order);

  } else {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
