#include "nav_msgs/Path.h";
#include <actionlib/client/simple_action_client.h>
#include <asars_global_planner/visiting_order_generator_node.h>
#include <chrono>
#include <limits>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <std_msgs/Bool.h>

#include <move_base_msgs/MoveBaseActionResult.h>
using namespace std::chrono;

auto start = high_resolution_clock::now();
auto duration_ = duration_cast<milliseconds>(start - start);

bool VisitingOrderGenerator::visiting_order_srv_cb(
    asars_global_planner::GenerateVisitingOrder::Request &req,
    asars_global_planner::GenerateVisitingOrder::Response &res) {
  std::vector<geometry_msgs::Point> visiting_order;
  geometry_msgs::Point current_point;
  ROS_INFO("Global Planner Request received");

  std::vector<geometry_msgs::Point> unvisited_points = req.victim_locations;

  while (unvisited_points.size() != 0) {

    double minimum_distance = std::numeric_limits<double>::max();
    int min_index = 0;

    for (uint8_t i = 0; i < unvisited_points.size(); ++i) {
      auto distance = calculate_distance(current_point, unvisited_points[i]);

      if (distance < minimum_distance) {
        minimum_distance = distance;
        min_index = i;
      }
    }
    ROS_INFO("Min distance is %f", minimum_distance);

    visiting_order.push_back(unvisited_points[min_index]);
    current_point = unvisited_points[min_index];

    unvisited_points.erase(unvisited_points.begin() + min_index);
  }
  res.victim_visiting_order = visiting_order;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);
  ROS_INFO("Waiting for action server to start.");

  ac.waitForServer(); // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  ros::Duration(4).sleep(); // sleep for half a second

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";

  start = high_resolution_clock::now();
  // iterate over all the waypoints
  int index = 1;
  for (const auto &point : visiting_order) {
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal %d out of %d", index, visiting_order.size());
    ac.sendGoal(goal);
    index++;
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal Reached!");
    } else {
      ROS_ERROR("Move base failed");
    }
  }
  std_msgs::Bool goal_reached;
  goal_reached.data = true;
  goal_reached_pub_.publish(goal_reached);

  return true;
}

double VisitingOrderGenerator::calculate_distance(geometry_msgs::Point point1,
                                                  geometry_msgs::Point point2) {
  double distance;
  distance =
      sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) * 1.0);

  return distance;
}

void chatterCallback(const nav_msgs::Path::ConstPtr &msg) {
  auto stop = high_resolution_clock::now();

  duration_ += duration_cast<milliseconds>(stop - start);
  ROS_INFO("Duration is %d", duration_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visiting_order_generator_node");
  ros::NodeHandle n;

  VisitingOrderGenerator visiting_order_generator;
  ros::ServiceServer ss =
      n.advertiseService("asars/start_victim_visting",
                         &VisitingOrderGenerator::visiting_order_srv_cb,
                         &visiting_order_generator);
  ros::Subscriber sub =
      n.subscribe("/move_base/NavfnROS/plan", 10, chatterCallback);

  visiting_order_generator.goal_reached_pub_ = n.advertise<std_msgs::Bool>("asars_all_victims_reached", 1);

  ros::spin();
  ROS_INFO("Duration is %d", duration_);

  return 0;
}
