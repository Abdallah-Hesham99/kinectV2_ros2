# kinectV2_ros2
A ros2 wrapper for the libk4w2 that publishes rgb and depth data into the ros2 system along with camera_info.
check out https://github.com/yoshimoto/libk4w2/tree/master for info about the original open source library with its dependencies 
- clone the repo to your ros2_ws/src folder then ```cd ..```
- run ```colcon build --symlink-install --packages-select camera_publisher```
- open a new terminal and run ```ros2 run camera_publisher pub``` this file publishes the rgb and depth data as well as camera_info topic.
- use ```rviz2``` to visualize the output from the kinect2, you can convert to pointcloud2 using this package https://github.com/ricardodeazambuja/depthimage_to_pointcloud2
 
