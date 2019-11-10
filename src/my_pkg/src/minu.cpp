#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char** argv)
{
  // Paneme ros toole
  ros::init(argc, argv, "minu");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Roboti grupp on sia5
  moveit::planning_interface::MoveGroupInterface move_group("sia5");

  
  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group.getCurrentPose();

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose next_waypoint = current_pose.pose;

  // Ta tuleb natuke ette ja vasakule
  next_waypoint.position.x += 0.04;
  next_waypoint.position.y -= 0.04;
  waypoints.push_back(next_waypoint);

  // Ta tuleb üles
  next_waypoint.position.z += 0.05;
  waypoints.push_back(next_waypoint);

  // Ta tuleb taha ja paremale
  next_waypoint.position.x -= 0.08;
  next_waypoint.position.y += 0.08;
  waypoints.push_back(next_waypoint);

  // Ta tuleb alla
  next_waypoint.position.z -= 0.05;
  waypoints.push_back(next_waypoint);

  // Ta tuleb natuke ette ja vasakule, home_pose
  next_waypoint.position.x += 0.04;
  next_waypoint.position.y -= 0.04;
  waypoints.push_back(next_waypoint);

  // Planeerime roboti liikumine
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  double movement_speed = 0.005;
  double jump_threshold = 0;
  double fraction = move_group.computeCartesianPath(waypoints, movement_speed, jump_threshold, my_plan.trajectory_);

  if (fraction == 1)
  {
    ROS_INFO("Planning OK. Proceeding.");
  }
  else
  {
    ROS_WARN("Error. Path was not planned");
    ros::shutdown();
    return 0;
  }

  ros::shutdown();
  return 0;
}
