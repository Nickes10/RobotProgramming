#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


cv::Mat global_map, map_with_laser;
double map_resolution = 0.0, map_width = 0.0, map_height = 0.0;
double robot_x = 0.0, robot_y = 0.0;
geometry_msgs::Quaternion robot_orientation;

// Initial and goal position of the robot
cv::Point initial_pose(-1, -1), goal_pose(-1, -1);

// Global publishers
ros::Publisher init_pose_pub;
ros::Publisher goal_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map_resolution = msg->info.resolution;
  map_width = msg->info.width;
  map_height = msg->info.height;
  global_map = cv::Mat(map_height, map_width, CV_8UC1);

  for (int y = 0; y < map_height; y++) {
    for (int x = 0; x < map_width; x++) {
      // Flip y so that the origin is at the bottom left as OpenCv wants
      int index = x + (map_height - y - 1) * map_width;
      int value = msg->data[index];
      
      if (value == 0) {  // free space --> set to White
        global_map.at<uchar>(y, x) = 255;
      } else if (value == 100) {  // occupied --> set to Black
        global_map.at<uchar>(y, x) = 0; 
      } else if (value == -1) {  // unknown --> set to Grey
        global_map.at<uchar>(y, x) = 127;
      } else { // any other value
        global_map.at<uchar>(y, x) = 75;
      }
    }
  }

}

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
  if (global_map.empty())
    return;

  // Convert pixel (x,y) to map coordinates
  double map_x = x * map_resolution;
  double map_y = (map_height - y) * map_resolution;  // flip y for OpenCV

  if (event == cv::EVENT_LBUTTONDOWN) {
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = map_x;
    init_pose.pose.pose.position.y = map_y;
    init_pose.pose.pose.position.z = 0.0;
    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = 0.0;
    init_pose.pose.pose.orientation.w = 1.0;

    ROS_INFO("Publishing initial pose: (%.2f, %.2f)", map_x, map_y);
    initial_pose = cv::Point(x,y);
    init_pose_pub.publish(init_pose);
  }
  if (event == cv::EVENT_RBUTTONDOWN) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = map_x;
    goal.pose.position.y = map_y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    ROS_INFO("Publishing goal pose: (%.2f, %.2f)", map_x, map_y);
    goal_pose = cv::Point(x,y);
    goal_pub.publish(goal);
  }
}


void positionCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
  if (global_map.empty()) {
      ROS_WARN("Map is not yet received!");
      return;
  }

  for (int i = 0; i < msg->transforms.size(); i++) {
    geometry_msgs::TransformStamped transform_data = msg->transforms[i]; 
      
    if (transform_data.header.frame_id == "odom") {
        robot_x = transform_data.transform.translation.x;
        robot_y = transform_data.transform.translation.y;
        robot_orientation = transform_data.transform.rotation;
    }
  }
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  if (global_map.empty()){
    ROS_WARN("Map is not yet received!");
    return;
  }

  cv::cvtColor(global_map, map_with_laser, cv::COLOR_GRAY2BGR);
  double robot_yaw = tf::getYaw(robot_orientation);
  
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    double range = msg->ranges[i];
    if (range > msg->range_min && range < msg->range_max) {
      double angle = msg->angle_min + i * msg->angle_increment;
      // Calculate laser endpoint in world coordinates
      double laser_x = robot_x + range * cos(robot_yaw + angle);
      double laser_y = robot_y + range * sin(robot_yaw + angle);
      // Convert to pixel coordinates 
      int px = static_cast<int>((map_width * map_resolution / 2 + laser_x) / map_resolution);
      int py = static_cast<int>((map_height * map_resolution / 2 - laser_y) / map_resolution);  // invert y for OpenCV
      cv::circle(map_with_laser, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
    }
  }

  // Visualize the robot in the OpenCv map
  int robot_pixel_x = static_cast<int>((map_width * map_resolution / 2 + robot_x) / map_resolution);
  int robot_pixel_y = static_cast<int>((map_height * map_resolution / 2 - robot_y) / map_resolution);
  cv::circle(map_with_laser, cv::Point(robot_pixel_x, robot_pixel_y), 10, cv::Scalar(255, 0, 0), -1);

  // Draw the initial pose (green) if it exists
  if (initial_pose.x != -1 && initial_pose.y != -1) {
    cv::rectangle(map_with_laser, initial_pose - cv::Point(6, 6), initial_pose + cv::Point(6, 6),
                  cv::Scalar(0, 255, 0), cv::FILLED); 
  }

  // Draw the goal pose (orange) if it exists
  if (goal_pose.x != -1 && goal_pose.y != -1) {
      cv::rectangle(map_with_laser, goal_pose - cv::Point(6, 6), goal_pose + cv::Point(6, 6),
                    cv::Scalar(0, 165, 255), cv::FILLED);
  }
  
  cv::imshow("Map", map_with_laser);
  cv::waitKey(30);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_rviz_node");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
  ros::Subscriber tf_sub = nh.subscribe("/tf", 1000, positionCallback);
  ros::Subscriber laser_sub = nh.subscribe("/base_scan", 1000, laserCallback);

  init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("Map", mouseCallback, nullptr); // Set Mouse click callback

  while (ros::ok()) {
    ros::spinOnce();
  }
  
  return 0;
}
