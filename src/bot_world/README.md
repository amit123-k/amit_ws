This package contains the world model with urdf. 
### How to test the package

1. Build the package:
   
   ``` bash
   cd ~/your_name_ws
   colcon build 
    ```

2. Source the workspace:
   ``` bash
   source install/setup.bash
    ```

3. This should spawn bot in the Gazebo world model:
   ``` bash
   ros2 launch bot_world robot.launch.py
   ```
   
4. For the teleop the bot in gazebo:
   ``` bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
