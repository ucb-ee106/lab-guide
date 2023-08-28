---
title: "Setting Up ROS"
nav_order: 1
parent: "ROS"
layout: default
has_toc: false
has_children: false
---
# Sourcing ROS
In lab we make you look into the `~/.bashrc` file, which is run every time you open a new terminal. Why? Well if we look at the contents of the `~/.bashrc` file in the user's home directory, we see the following line in the bashrc file[^1]:
```
source /opt/ros/noetic/setup.bash
```
By sourcing your `~/.bashrc` file, you are implicity sourcing the ROS setup file. The `setup.bash` file is a script that sets up your environment variables for ROS. These enviroment varibles can then be used by programs. For example we can print out the following enviroment variables to see what ROS setup after running:
```
# Current ROS distribution
$ echo $ROS_DISTRO
noetic
# IP address of the ROS master
$ echo $ROS_MASTER_URI
http://localhost:11311
# Location of ROS packages
$ echo $ROS_PACKAGE_PATH
/opt/ros/noetic/share
```

But when we ask you to use the Sawyer robot, we instead ask you comment out `source /opt/ros/noetic/setup.bash`
and to uncomment the following line in your `~/.bashrc` file:
```
source opt/ros/eecsbot_ws/devel/setup.bash
```
This is because we have created a custom ROS workspace for the Sawyer robot. Under the hood, this script also setups up ROS noetic like `source /opt/ros/noetic/setup.bash`, but also contains packages that are specific to the Sawyer robot. We can see this by printing out the updated `ROS_PACKAGE_PATH` enviroment variable:
```
# Location of ROS packages
$ echo $ROS_PACKAGE_PATH
/opt/ros/eecsbot_ws/src:/opt/ros/noetic/share
```

# What is the bashrc file?
The bashrc is a special file in UNIX systems that is run everytime a new instance of a bash shell is opened. So we add the line `source /opt/ros/noetic/setup.bash` to the bashrc file so that everytime we open a new terminal, ROS is setup.

[^1]: The noetic in the path is the ROS version. In this case, it is ROS Noetic. If you are using a different version of ROS, then the path will be different. For example, if you are using ROS Melodic, then the path will be `/opt/ros/melodic/setup.bash`. Note we are using ROS1, which is frankly a bit outdated and is being replaced by ROS2, which makes some fundamental changes the architeture of ROS. New major releases for ROS are released roughly every two years.