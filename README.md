# raigor_simulation
basic modules for raigor's simulation

## raigor
#### raigor_bringup
The bringup module of the raigor robot. 

Try <code>roslaunch raigor_bringup with_moveit_eod.launch</code>, if you want to work with set moveit configuration.
While <code>without_moveit_demo.launch</code> offers another option without moveit configuration.
#### raigor_description

For the <code>.xacro</code> and <code>.urdf</code> file. The model mesh were also included.

#### raigor_gazebo

Some preset worlds.

## raigor_control

#### gazebo_continuous_track

This package would generate a plugin to control the tracked vehicle. With the plugin the tracked vehicle is able to get over with small obstacles, work in uneven terrain, even go up and down stairs without external aid.


#### raigor_cartesian_velocity_control

Inner controller for manipulator. This controller would generate the joint desired velocity when the corresponding end-effector velocity in cartesian space was received.

#### raigor_pid_control
 
Outer controller for manipulator. This controller would generate the end-effector desired cartesian velocity considering the current and goal end-effector's pose.

#### teleop_twist_keyboard

Base velocity control via keyboard.



## raigor_perception

For test.


## raigor_planning

#### misson_mode

Ros controller switch tool.

#### raigor_general_planning

A small interface of controller.

#### raigor_moveit_config

Moveit config file for the whole robot.

## tools

For information prinf and output.