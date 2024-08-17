# NDT C++

Minimum NDT

https://github.com/TakanoTaiga/ndt_cpp/assets/53041471/510656ef-73a8-49dd-b51f-698165e1922a



## RUN

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kazu-321/ndt_cpp_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 run ndt_cpp_ros2 ndt_cpp_ros2
```

## IO
### Input
- `/input/scan`:  sensor_msgs::msg::LaserScan
- `/input/map` :  nav_msgs::msg::OccupancyGrid

### Output
- `/output/pose`: geometry_msgs::msg::PoseStamped
- `/tf`:

<!-- ## Parameters
- `map_frame_id`: map frame id
- `base_frame_id`: base frame id -->

## LICENSE

NDT C++ specific code is distributed under Apache2.0 License.
The following extensions are included (*.cpp *.md)


NDT C++ specific data are distributed under MIT License.
The following extensions are included (*.txt)
