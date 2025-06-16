## This package contains the steps to test the package. 

1. Build the package and Source the workspace:
   
   ``` bash
   cd ~/your_name_ws
   colcon build --symlink-install
   source install/setup.bash
    ```

2. To launch the robot URDF into the custom world in gazebo:
   ``` bash
   ros2 launch autobot launch_sim.launch.py world:=./src/autobot/worlds/warehouse.world
    ```

3. Open RViz with custom config: 
   ``` bash
   rviz2 -d src/autobot/config/main.rviz
    ```

4. Launch SLAM Toolbox (online async mode):
   ``` bash
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/autobot/config/mapper_params_online_async.yaml use_sim_time:=true
    ```
5. Launch Nav2 navigation stack:
   ``` bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
    ```
