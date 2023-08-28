---
title: "ROS"
nav_order: 1
layout: default
has_toc: false
has_children: true
---
# Robot Operating System (ROS)
This section will explore the Robot Operating System (ROS). ROS is a framework for writing robot software. It provides a collection of tools, libraries, infrastructures and conventions to create robotic platforms. Through these ROS provides hardware abstraction, low level device control, message-passing between prcoesses, and package management[^1]. ROS is not an operating system, but rather a meta-operating system that runs on top of a normal operating system that provides C++ and Python libraries. In this class we will focus on using ROS with Python for it's ease of use, but in many applications C++ is used as it is signifgantly faster than Python. In fact in the real world, you may not use ROS at all in trade for custom middleware, due to it's issues with real time performance, but the concepts learned through ROS still apply. However, ROS is a great tool for learning about robotics and still is the prevalent tool in research labs.

[^1]: Much of this lab guide is inspired by and paraphrases from the official [ROS Wiki](http://wiki.ros.org) and is a great resource for learning more about ROS.
