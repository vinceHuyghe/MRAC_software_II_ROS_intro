# Data Acquisition

## Sensors

### available sensors at IAAC

recommended sensors:

- Astra
- Astra embedded (astra mini)
- Realsense D405 (2x)

other sensors:

- Azure kinect
- Lepton pure thermal (2x)
- RPlidar A2M8 (2x)

### integrating sensors

- ros pkg (select correct stage in  Dockerfile)
- [udev rules](https://github.com/vinceHuyghe/moveit1_ur#udev-rules)
- static tf broadcaster in launch file

### running the sensors

- launch the appropriate ros node

    ``` roslaunch realsense2_camera rs_camera.launch ```

    ``` roslaunch astra_camera astra.launch ```

- inspect the published topics using rqt, rviz and ros cli tools
- inspect the launch files to see the available arguments

### industrial reconstruction

- pkg overview: Leveraging [Open3D](http://www.open3d.org/) for industrial reconstruction using RGB-D cameras and [TSDF](http://www.open3d.org/docs/latest/tutorial/t_reconstruction_system/integration.html) integration

- pkg configuration / launch files

#### reconstruction parameters

note: This package was ported from ROS2 to ROS1 and not all features have been ported, the archive player is not available in ROS1.

The parameters below should be carefully considered when running the reconstruction node. The bottleneck in the reconstruction is the amount of RAM available in your computer. You will therefore need to balance the parameters carefully according to your application. For example if you want to scan a large object you will have to reduce the resolution of the scanning.

- robot parameters
  - tcp speed
  - distance to target

- camera parameters
  - camera framerate
  - white balance, contrast, etc
  - camera clipping distance (see launch file)
  note: As the realsense D405 exclusively uses stereo cameras for depth perception (in contrast to many depth cameras that use IR) it is sensitive to lighting conditions.

- start reconstruction parameters

  - motion parameters

    - tracking frame: Camera tf frame where the image and depth image are relative to.
    - relative frame: Base tf frame that the TSDF mesh will be generated relative to.
    - translation_distance: Distance the tracking_frame must travel relative to the   relative_frame before another image is allowed to be added to the volume. Typically works best with 0 however this value can be increased to limit the amount of data being recorded
    - rotational_distance: Rotational distance the tracking_frame must rotate relative to the relative_frame before another image is allowed to be added to the volume. Typically works best with 0 however this value can be increased to limit the amount of data being recorded
  
  - TSDF parameters
  
    - voxel length: Controls the size of triangles created (Note: this is not in meters, in units associated with the camera).
    TODO: check this
    - sdf trunc: Controls how connected to make adjacent points. A higher value will connected more disparate data, a low value requires that points be very close to be connected.

    The relationship between the voxel length and the sdf trunc should in the range of 1:2 to 1:4.

  - RGBD parameters

    - depth scale: Scale of the data. Set to 1000 to get the output data in meters if the camera's default distance scale is in millimeters (such as for example for the realsense D405).
    - depth trunc: The distance at which data beyond is clipped and not used. Example: 1.0 would lead to anything greater than 1.0 meters away from the camera would be ignored. Its recommended to set the clipping of the camera in order to limit the amount of data being recorded. The same value as the camera clipping distance can then be used for reconstruction as well.

- stop reconstruction parameters

  - mesh filepath: The path to the file where the mesh will be saved. Include file extension in name.
