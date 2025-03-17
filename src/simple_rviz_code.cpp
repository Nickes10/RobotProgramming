#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>


cv::Mat global_map, map_with_laser;
double map_resolution = 0.0, map_width = 0.0, map_height = 0.0;
double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
geometry_msgs::Quaternion robot_orientation;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Map resolution: %f, width: %d, height: %d", 
           msg->info.resolution, msg->info.width, msg->info.height);

  map_resolution = msg->info.resolution;
  map_width = msg->info.width;
  map_height = msg->info.height;
  global_map = cv::Mat(map_height, map_width, CV_8UC1);

  ROS_INFO_STREAM("Displaying map of size: " << global_map.size());

  for (int y = 0; y < map_height; y++) {
    for (int x = 0; x < map_width; x++) {
      // Flip y so that the origin is at the bottom left as OpenCv wants
      int index = x + (map_height - y - 1) * map_width;
      int value = msg->data[index];
      
      if (value == 0) {  // free space --> set to White
        global_map.at<uchar>(y, x) = 255;
      } else if (value == 100) {  // occupied --> set to Black
        global_map.at<uchar>(y, x) = 0; 
      } else if (value == -1) {  // unknown --set to Grey
        global_map.at<uchar>(y, x) = 127;
      } else { // any other value
        global_map.at<uchar>(y, x) = 75;
      }
    }
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
        ROS_INFO("Transform x: %f", transform_data.transform.translation.x);
        robot_x = transform_data.transform.translation.x;
        robot_y = transform_data.transform.translation.y;
        robot_orientation = transform_data.transform.rotation;
        ROS_INFO("Robot x coordinate: %f, y coordinate: %f, orientation (z): %f, orientation (w): %f",
                     robot_x, robot_y, robot_orientation.z, robot_orientation.w);
    }
  }
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  if (global_map.empty())
    return;
  
  // Create a copy of the map to draw on
  cv::cvtColor(global_map, map_with_laser, cv::COLOR_GRAY2BGR);
  robot_yaw = tf::getYaw(robot_orientation);
  
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    double range = msg->ranges[i];
    if (range > msg->range_min && range < msg->range_max) {
      double angle = msg->angle_min + i * msg->angle_increment;
      // Calculate laser endpoint in world coordinates
      double laser_x = robot_x + range * cos(robot_yaw + angle);
      double laser_y = robot_y + range * sin(robot_yaw + angle);
      // Convert to pixel coordinates (
      int px = static_cast<int>((map_width * map_resolution / 2 + laser_x) / map_resolution);
      int py = static_cast<int>((map_height * map_resolution / 2 - laser_y) / map_resolution);  // invert y for OpenCV
      cv::circle(map_with_laser, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
    }
  }

  // Visualize the robot in the OpenCv map
  int robot_pixel_x = static_cast<int>((map_width * map_resolution / 2 + robot_x) / map_resolution);
  int robot_pixel_y = static_cast<int>((map_height * map_resolution / 2 - robot_y) / map_resolution);
  cv::circle(map_with_laser, cv::Point(robot_pixel_x, robot_pixel_y), 12, cv::Scalar(255, 0, 0), -1);
  
  cv::imshow("Map", map_with_laser);
  cv::waitKey(30);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_rviz_node");
  ros::NodeHandle nh; // To comunicate with ROS

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
  ros::Subscriber tf_sub = nh.subscribe("/tf", 1000, positionCallback);
  ros::Subscriber laser_sub = nh.subscribe("/base_scan", 1000, laserCallback);
  cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);

  while (ros::ok()) {
    ros::spinOnce();
  }
  
  return 0;
}
