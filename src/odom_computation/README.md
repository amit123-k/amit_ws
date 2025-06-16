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
