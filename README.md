# IveroSLAM_ros2

This repository wraps IveroSLAM in a ros package so we can publish results over ros. This runs exactly the same as the original IveroSLAM, except it will publish tracked images and point clouds to ros.

# Building and Running IveroSLAM_ros2

1. Follow install instructions in [IveroSLAM](https://github.com/Luxolis/IveroSLAM)
2. git clone https://github.com/Luxolis/IveroSLAM.git ~/ros2_ws/src
4. Replace line 22 in CMakeLists.txt: `set(ORB_SLAM3_DIR $ENV{HOME}/projects/IveroSLAM)` with the path to IveroSLAM if its not already correct
5. Clone https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1 to the repository
6. `colcon build` in the ros2_ws directory
7. Run the follow 3 commands to run: (ensuring a realsense is plugged in)
8. `ros2 launch iveroslam_ros2 ivero_rs_launch.py` launches the realsense camera
9. `ros2 run iveroslam_ros2 data_recorder` launches the data recorder
10. `ros2 run iveroslam_ros2 ivero_slam_ros`  launches slam

# Starting and Stopping SLAM

Although the system is "started" when the node is launched, we wait for a trigger to a service before we actually start slam:

`ros2 service call /ivero_slam/start std_srvs/srv/Trigger`

And to stop the scanning session and output the results to a zip file:

`ros2 service call /ivero_slam/stop std_srvs/srv/Trigger`
