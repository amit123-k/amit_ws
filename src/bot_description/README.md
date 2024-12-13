This package contains the robot description of the used model. 

Inside urdf folder 

1. The bot_description.urdf.xacro file contains the physical properties of it for example the base_link, base_footprint, three wheels. It the holds the visual,collision and inertial property of every link. The gazebo_ros plugin for differential drive control is taken from open_source github repositories.

2. The lidar.xacro file holds the information for different sensors(camera, lidar) along with its gazebo properties. The plugins used here for configuring the sensors are also taken from gazebo_ros_plugins github repositories(open source).

3. The common_properties.xacro file is used to contain all the physical dimensions of all links along with the material properties both for gazebo and rviz. Also it calculates the inertial properties of all the physical models used here.

The xacro functionality is used to avoid the length and complexity of the code.

The launch folder contains two launch files

1. The spawn.launch.py file launches the robot in empty gazebo environment, but all its property can be seen. It activates three nodes namely robot_state_publisher node, launches gazebo and spawn_entity node that spawns the robot model in gazebo. To run it use the following command

       ros2 launch bot_description spawn.launch.py

2. The rviz.launch.py file launches the robot model in rviz environment. This launch file also activates 3 nodes namely robot_state_publisher, rviz node that launches rviz nad joint_state_publisher that portrays the joint information of the urdf. Use this joint_state_publisher_gui only if joint_states are not available in the environment, because the /joint_states topic gets active once the model is launched in gazebo. To run this launch file use following command.

       ros2 launch bot_description rviz.launch.py
