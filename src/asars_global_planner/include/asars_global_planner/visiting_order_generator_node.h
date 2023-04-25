#pragma once

#include "ros/ros.h"

#include "asars_global_planner/GenerateVisitingOrder.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


class VisitingOrderGenerator {
public:
  // service callback
  bool visiting_order_srv_cb(
      asars_global_planner::GenerateVisitingOrder::Request &req,
      asars_global_planner::GenerateVisitingOrder::Response &res);

  double calculate_distance(geometry_msgs::Point point1,
                            geometry_msgs::Point point2);

  void robot_pose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  ros::Publisher goal_reached_pub_;

};
