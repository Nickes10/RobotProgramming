# Simple RViz - Robot Programming course project

## Description
The objective of this project is to develop a ROS-based program for controlling a mobile robot while providing a basic visualization system similar to RViz. The application displays key elements such as a map obtained from the map server, laser scan data, and the robot's base represented as a circular shape. All components are rendered according to their respective transformations.  
Additionally, the program allows users to interact with the robot by setting the initial position by clicking with the left mouse button on the map, which sends an `/initialpose` message and displays a green marker at the selected location. Similarly, users can set a goal position by clicking on the map, publishing the target to the `/move_base/goal` topic, with an orange marker appearing at the chosen point.

### Setting Up the Workspace  

The following procedure assumes you have **_ROS Noetic_** already installed on your machine. 
Create a folder of your choice (in my case, it’s `/home/<usr>/Desktop/RobotProgramming`) and set up the catkin workspace inside it. The workspace consists of the `simple_rviz` package, which is this GitHub folder, and the `srrg` folder containing the required packages from [srrg catkin workspace](https://gitlab.com/srrg-software). 

To create the structure, navigate to the `RobotProgramming` directory and run the following code:
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```
Installed the necessary selected packages in the `srrg` folder as following:  
```bash
# Navigate to the workspace
cd catkin_ws/src

# Create the srrg directory for dependencies
mkdir srrg
cd srrg

# List of required SRRG packages
repos=(
    srrg2_config_visualizer
    srrg2_core
    srrg2_laser_slam_2d
    srrg2_navigation_2d
    srrg2_orazio
    srrg2_qgl_viewport
    srrg2_slam_interfaces
    srrg2_solver
    srrg_cmake_modules
    srrg_hbst
)

# Base URL for cloning the repositories
url_base="https://gitlab.com/srrg-software"

# Clone each required repository
for repository in "${repos[@]}"; do
    echo "Cloning $repository..."
    git clone "$url_base/$repository.git"
done

echo "SRRG dependencies have been successfully downloaded."
```
Then, **clone the GitHub repository** in the `src` folder: 
```bash
cd catkin_ws/src
git clone https://github.com/Nickes10/RobotProgramming.git
```

For the sake of simplicity Rename the folder writing this command:
```bash
mv RobotProgramming simple_rviz
```

Finally build the ROS workspace and source the ROS environment:
```bash
cd ~/catkin_ws/
catkin build

# Set up the ROS environment
source /opt/ros/noetic/setup.bash 
source devel/setup.bash
```
This setup ensures that the `simple_rviz` package is placed inside the `src` folder, while all required `srrg` dependencies are organized in a separate folder within the same workspace.

## **Run the Project**  

To launch your project, simply run in a terminal the roscore:
```bash
roscore
```

While in another terminal, run the following command:  
```sh
roslaunch simple_rviz launch_file.launch
```

Once the project is running, you'll be able to see:

- A **map**  displayed, showing the robot's environment.
- A **blue point** representing the **robot** on the map.
- **Laser scans** visualized as **red points** on the map.
  
You can interact with **RViz** to set the **initial pose** of the robot. To do this, simply **click on the map** with the **left mouse button** to set the robot's starting position. A **green point** will appear where you clicked to indicate the **initial pose**. This information will be published to the `/initialpose` topic.

Additionally, you can set the **goal pose** by clicking anywhere on the map. An **orange point** will be placed at the clicked location, and the goal will be sent to the `/move_base/goal` topic.