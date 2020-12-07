# manip_utils

# To set up:
1. Clone this repository to your ``catkin_ws/src/``
2. Clone this one too: ```git clone https://github.com/Kinovarobotics/ros_kortex```
3. Build your catkin workspace and source the ``~/.bashrc`` file

# To test:
1. Launch Gazebo: ```roslaunch kortex_gazebo spawn_kortex_robot.launch```
2. In a separate terminal, while Gazebo is running, execute a trajectory from file: ```roslaunch manip_utils cart_traj_from_file.launch```

# To download MoveIt! from source:
```cd ~/catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/HARPLab/moveit/kinetic-devel/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
catkin config --extend /opt/ros/kinetic --cmake-args -DCATKIN_ENABLE_TESTING=0 -DUSE_CONAN=OFF```
