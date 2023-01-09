# Introduction to ROS

&nbsp;

## Theory

Slides can be found in the softwareII gdrive folder.

- What is ROS
- The ROS computational graph
- The ROS filesystem
- ROS tools

&nbsp;

## Environment setup

Fork and clone the following repository: [turtlesim](https://github.com/vinceHuyghe/turtlesim)

Follow the instructions in the README.md file for building, running and using the container.

## Exercises

Follow the tutorials in the links below. Complete the chapters listed under the heading.

- [Understanding ROS nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
  - 5 Roscore
  - 6 Using rosnode
  - 7 Using rosrun
  - 8 Review  
- [Understanding ROS topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
  - 1 Setup
  - 2 ROS topics
  - 4 Rostopic continued
  - 5 Using rqt_plot  
- [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
  - 1 ROS services
  - 2 Using rosservice
  - 3 Using rosparam

- [Roslaunch and launch files](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
  - 2.2 Using roslaunch
  - 2.3 The launch file
  - 2.4 The launch file explained
  - 2.5 roslaunching

- [Rosbag](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data)
  - 1 Recording data
  - 2 Examining and playing the bag file
  - 3 Recoding a subset of the data
  - 4 Limitations of rosbag

- Creating a catkin workspace and package
  - Creating a catkin workspace  

    These instructions assume that you have installed catkin tools and you have sourced your environment.  
    To create a catkin workspace, run the following commands in a terminal:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin build
    ```

    To source your new workspace, run the following command in the terminal:

    ```bash
    source devel/setup.bash
    ```

  - [Creating a catkin package](http://wiki.ros.org/catkin/Tutorials/create_a_pkg)
  - [CMakeList.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
    - 1 Overview
    - 2 Overall structure and ordering
    - 3 Cmake version
    - 4 Package name
    - 8 Messages, services and action targets
    - 11.1 Installing Python executable scrips
    - 11.3 installing roslaunch files or other resources

&nbsp;

## Assignment

- Create a package called `my_package` with a launch file called `my_launch.launch` that launches the following nodes:
- `roscore`
- `rosrun turtlesim turtlesim_node`
- `rosrun turtlesim turtle_teleop_key`

## Refences

- [ROS tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS video tutorials - ROS Noetic for beginners](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q)

&nbsp;
