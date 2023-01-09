# Bridging between Rhino and ROS

[This](https://github.com/vinceHuyghe/gh_ros) repo contains the gh examples. See repo README for installation instructions.

- using compass fab
- (using roslibpy)

You will need to start the ROS bridge server on the ROS machine. This can be done by running the following command in a terminal:

```shell
 roslaunch rosbridge_server rosbridge_websocket.launch
```

## Prerequisites roslibpy

ROS prerequisites:

- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)

  ```shell
  sudo apt install ros-noetic-rosbridge-suite
  ```

  launch rosbridge server

  ```shell
  roslaunch rosbridge_server rosbridge_websocket.launch
  ```

Rhino roslibpy prerequisites:

- [roslibpy](https://roslibpy.readthedocs.io/en/latest/index.html)

    You will need to install roslibpy for Ironpython. The easiest way is to use [pip](https://pypi.org/project/pip/). You will need to add Ironpython to your windows  PATH. Open the windows command prompt as administrator and run the following command:

    ```shell
    ipy -X:Frames -m pip install --user roslibpy
    ```

    If this method does not work you can try the alternative method described [here](http://wiki.bk.tudelft.nl/toi-pedia/Installing_IronPython_modules_for_Grasshopper).

    Test your installation by running the following code in Rhino's PythonScript editor:

    ```python
    import roslibpy
    ```

## Network

Set your network to a static IP address ensuring that the IP address is in the same subnet as the ROS machine.

### turtle pose subscriber

### turtle twist publisher


