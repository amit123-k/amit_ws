## This package contains the steps to test the package. 

1. Build the package and Source the workspace:
   
   ``` bash
   cd ~/your_name_ws
   colcon build --symlink-install
   source install/setup.bash
    ```


1. Simulate encoder ticks:
   ```bash
   ros2 run odom_computation wheel_tick_publisher
    ```

3. Run odometry node (computes /odom)
   ```bash
   ros2 run odom_computation odom_node
    ```

4. Verify odometry topic
   ```bash
   ros2 topic echo /odom
    ```

___________
## Bonus Task: Simulated IMU Publisher

1. Start publishing IMU data
   ```bash
   ros2 run odom_computation imu_publisher
    ```

2. Inspect published IMU readings
   ```bash
   ros2 topic echo /imu/data
    ```
