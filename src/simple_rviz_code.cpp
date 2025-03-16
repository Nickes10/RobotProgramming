#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>

//std::mutex map_mutex; // Mutex map to avoid multiple change of the map at the same time
cv::Mat global_map;
double map_resolution = 0.0, map_width = 0.0, map_height = 0.0;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  //std::lock_guard<std::mutex> lock(map_mutex);
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
      
      if (value == 0) {  // free space
        global_map.at<uchar>(y, x) = 255;
      } else if (value == 100) {  // occupied
        global_map.at<uchar>(y, x) = 0;
      } else if (value == -1) {  // unknown
        global_map.at<uchar>(y, x) = 127;
      } else { // any other value
        global_map.at<uchar>(y, x) = 75;
      }
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_rviz_node");
  ros::NodeHandle nh; // To comunicate with ROS

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
  cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);

  while (ros::ok()) {
    ros::spinOnce();
    if (!global_map.empty()) {
      cv::imshow("Map", global_map);
      cv::waitKey(30);
    }
  }
  
  return 0;
}
