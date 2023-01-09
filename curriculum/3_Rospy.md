# rospy

## rospy tutorial

prerequisite: create a catkin workspace and ros pkg

- create a catkin workspace

    ```shell
    mkdir -p catkin_ws/src
    ```

    reference: [Creating a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

- create a ros pkg

    navigate to the src folder of your workspace

    ```shell
    cd catkin_ws/src
    ```

    create a ros pkg

    ```shell
    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
    ```

    reference: [Creating a catkin package](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

publisher and subscriber

- [writing a simple publisher and subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- [examining the simple publisher and subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

simple service and client

- [writing a service and client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
- [examining the service and client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)

(parameters and logging)

- [using parameters](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters)
- [logging](http://wiki.ros.org/rospy_tutorials/Tutorials/Logging)

## Assignment

- Navigate to the src folder of your workspace

  ```shell
  cd dev_ws/src
  ```

- Create a ROS pkg

  ```shell
  catkin_create_pkg rospy_tutorial rospy turtlesim geometry_msgs
  ```

- turtle pose subscriber

  write a node that subscribes to the pose of the turtle and prints the position and orientation of the turtle

- turtle twist publisher

  write a node that publishes a twist message to the turtle

- add you newly created scripts to CMakeLists.txt

- compile your workspace `catkin build`

- test by rosrunning your scripts

### commit and push your code

```shell
git add .
git commit -m "turtle pose subscriber and twist publisher"
git push
```

## Recommended reading

- actionlib
  - [writing a simple action server using the execute callback](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29)
  - [writing a simple action client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29)

- Object oriented programming with ROS in python
  - [Object Oriented Programming with ROS in Python](https://roboticsbackend.com/oop-with-ros-in-python/)
