---
title: "Aliases"
nav_order: 2
parent: "Miscellaneous"
layout: default
has_toc: false
has_children: false
---
# Aliases
Do you ever get tired of typing the same long command over and over again? Well, you can create an alias for it! An alias is a shortcut for a command. For example, if you want to create an alias for `source devel/setup.bash` you can add the following line to the end your `~/.bashrc` file:
```
alias sdevel="source devel/setup.bash"
```
Since we added this line to the `~/.bashrc` file, everytime we open a new terminal, the alias will be created. Now we can type `sdevel` instead of `source devel/setup.bash` to source our workspace.

## Useful Aliases
Here are some useful aliases that you can add to your `~/.bashrc` file:
```
# Source the current workspace
alias sdevel="source devel/setup.bash"
# Enable the Sawyer robot (please complete the relevant Sawyer lab before using this alias)
alias esawyer="rosrun intera_interface enable_robot.py -e"
# Run Sawyer's joint trajectory controller
alias sawyer_joint_traj_ctrl="rosrun intera_interface joint_trajectory_action_server.py"
# Launch the MoveIt for Sawyer with the gripper enabled
alias sawyer_moveit_gripper="roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true"
```