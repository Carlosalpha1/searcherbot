# searcherbot
Searcherbot is a simple robot that is able to navigate to perform search and transport tasks

## searcherbot_bringup
It contains initial launch and config files to load the robot

## searcherbot_description
It contains urdf and xacro files of the robot

## searcherbot_examples
It contains some execution examples using behavior trees, ROS Navigation Stack, (more examples in process)

## searcherbot_navigation
It contains config files to be used with ROS Navigation Stack

### Example Usage:
~~~
$ roslaunch searcherbot_bringup sim_robot.launch stage:=(1|2 ...)
$ rosrun searcherbot_examples bumpgo
~~~

*IMPORTANT*: It is necessary this command in $HOME/.bashrc to avoid some Gazebo problems:
~~~
$ export GAZEBO_MODEL_PATH=<absolute_path_to_searcherbot>/searcherbot_bringup/models:$GAZEBO_MODEL_PATH
~~~
