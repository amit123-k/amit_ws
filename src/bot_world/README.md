This package contains the world file of gazebo named as gazebo_world.world
The world can only be visualise by the following command. Before that navigate to the worlds folder and run this:
        
    gazebo gazebo_world.world

And to spawn the robot inside this world use this command:

    ros2 launch bot_world robot.launch.py

This is the final view of gazebo environment where the robot model can be operated through teleop command and its sensor informations can be visualized through different topics namely /camera_sensor (five different topics) and /scan topic for lidar sensor.

The robot can be run inside the world using the following command.(Teleop twist keyboard)

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 
