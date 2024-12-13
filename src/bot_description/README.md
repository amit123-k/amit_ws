This package contains the robot description of the used model. 
### How to test the package

1. Build the package:
   
   ``` bash
   cd ~/your_name_ws
   colcon build --packages-select bot_description
    ```

2. Source the workspace:
   ``` bash
   source install/setup.bash
    ```

3. This should launch the robot model in RViz:
   ``` bash
   ros2 launch bot_description rviz.launch.py
    ```

4. To launch the robot URDF into the empty world in gazebo:
   ``` bash
   ros2 launch bot_description spawn.launch.py
    ```

5. For the teleop the bot in gazebo:
   ``` bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
