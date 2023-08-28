---
title: "Computation Graph"
nav_order: 3
parent: "ROS"
layout: default
has_toc: false
has_children: false
---
# Computation Graph
## Nodes
A node is the process that performs computation in ROS and is meant to create modularity. Nodes are combined together into a graph and communicate with one another using topics and services. A typical ROS system will be comprised of many nodes. For example if you have a robotic arm that is trying to pick up a banana you might want a node to run the camera, a node to locate the pose of the banana, and a node to plan a path to the banana. Each of these nodes would communicate with one another using topics and services.
## Master
The ROS Master, intilized by running `roscore` in the terminal, provides naming and registration services for the rest of the nodes in the ROS system. In short the ROS Master allows invidual nodes to find one another. Once nodes locate each other through the ROS Master, they can communicate peer-to-peer. The ROS Master also runs a Parameter Server that allows nodes to store and retrieve parameters (similar to python script parameters) at runtime.
# Communication Interfaces
In the package file structure we saw the `msg/` and `srv/` directories but never explained them. These directories contain the message and service definitions for the package. Messages and services are the communication interfaces between ROS nodes.
## Topics and Messages
Messages are posted on topics and used for asynchronous communication
ROS topics are like bulletin boards where ROS nodes continuously pin messages for others to read. Here is an exmaple of ROS nodes communicating using messages and topics:  
> You have the following vision capture system setup (hardware):  
>> 1. A Stereo Camera
>> 2. Motorized Laser Pointer.

>Your goal is to use the most advanced AI algorithms available today to find a banana in the stereo camera image, and then estimate the depth of this banana in the image. Next, you want to point an actuated laser pointer at the banana using ROS as your middleware.    
> Let's break down the components in this robotics system:
> 1. There's a Stereo Camera component that collect depth images.
> 2. There's your high-tech AI algorithm component that locates the banana in the image, and calculates the target position at which the laser pointer should end up based on the location given by the AI component.
> 4. There's the PID controller component that controls the actual motors to the target position.  

> For each component, you have written the most optimized Python scrips to execute the respective tasks. The question now is, how should I piece all of this together? The answer to this is -- low and behold, ROS, topics (communication channels), and messags.

### Messages
ROS "Messages" are like mail envelopes, that are used when sending information from one node to another in a ROS computation graph via an interface called "Topics". Under the hood, ROS messages go through networking protocols to be delivered to a destination ROS node. In fact, ROS messages can also be sent through the Internet to another running ROS system if configured correctly! This is called cloud robotics, a field which will vaguely be covered in this course. Now in ROS messages, we store data. You can store and name different message fields. The type of data that we store in these messages vary, and as crazy as it sounds, message fields can also be defined in terms of other messages! To relate messages to something you might know, they're like structs in Object Oriented Programming, but they're constrained to the below primitive types and can be defined as arrays of these primitive types:
```
int8 : 8-bit signed integer.
uint8: 8-bit unsigned integer.
int16: 16-bit signed integer.
uint16: 16-bit unsigned integer.
int32: 32-bit signed integer.
uint32: 32-bit unsigned integer.
int64: 64-bit signed integer.
uint64: 64-bit unsigned integer.
float32: 32-bit floating point number.
float64: 64-bit floating point number.
string: Represents a sequence of characters.
bool: Represents a boolean value, either True or False.
time: Represents a timestamp, with two integer fields for seconds and nanoseconds.
duration: Represents a time duration, with two integer fields for seconds and nanoseconds.
char: Represents a single character.
byte: Represents a single byte.
```


There are many default ROS message packages already installed with ROS Neotic. As long as you import the message packages, as we will teach you later on, you can use these messages. Here are common packages and the messages they contain:
[Common Messages](http://wiki.ros.org/common_msgs). They contain message formats for Images, Position, PointClouds, etc. Almost everything that you can think of. Hence, before you create your own messages, always find if they already exist first. 

However, if you can't find what you need in the list of common messages, you have the option to create your own.

### Creating Your Own ROS Message
Here's an example of a message definition file containing positions, "x" and "y" will look like this:
``` 
int32 x
int32 y
```
where x and y are names that refers to the message fields. 
Exact steps on how to create your own messages will be in Lab2.

<!-- 1. Create a new directory named msg inside your package:
    ```
    mkdir ~/catkin_ws/src/my_custom_msgs/msg
    ```
2. Create a new .msg file inside this directory, let's call it Position_with_name.msg. 
    ```
    int32 x
    int32 y
    std_msgs/String[] name <!! Refer to the next portion on how to impor std_msgs>
    ```
3. Edit `package.xml`
    ```
    ...
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ...
    ```
4. Update CMakeList.txt -->
  
### Using A ROS Message Package
Say, right now, we're writing a stereo_camera_driver (package). After calling the APIs provided by the camera company using a Python script, we obtain the image as a cv2 array. Now, the next step is to wrap it up into an Image type message, and **Publish** it such that the all the other nodes in the ROS system can receive it as they **Subscribe** to the topic. 

**Setup**
To be able to do this, there is some setup involved. 
1. Editing `CMakeLists.txt` to find the package.  
    The packages that contains the `Image` message is `sensor_msgs`. Edit this part of the file:
    ```
    ...
    find_package(catkin REQUIRED COMPONENTS
        rospy
        sensor_msgs
    )
    ...
    ```
2. Edit `package.xml`.  
    In `package.xml`, add this line:
    ```
    ...
    <depend>rospy</depend>
    <depend>sensor_msgs</depend> <!-- Add this line -->
    ...
    ```
3. Wrap up Message in Python, and publish it into a topic channel named `/my_image_topic`
    ```
    import rospy
    from sensor_msgs.msg import Image
    import numpy as np
    from cv_bridge import CvBridge

    def image_publisher():
        pub = rospy.Publisher('/my_image_topic', Image, queue_size=10)
        rospy.init_node('image_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10 Hz
        
        bridge = CvBridge()

        while not rospy.is_shutdown():
            # Create a 480x640 black image
            image_np = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # Convert the NumPy image to sensor_msgs/Image
            image_msg = bridge.cv2_to_imgmsg(image_np, encoding="bgr8")

            rospy.loginfo("Publishing image")
            pub.publish(image_msg)
            
            rate.sleep()In the package file structure we s

    if __name__ == '__main__':
        try:
            image_publisher()
        except rospy.ROSInterruptException:
        pass
    ```
3. Go to the workspace root folder and build the package.
    ```
    catkin_make
    ```
4. Source your ROS environment and run the Python script:
    ```
    source devel/setup.bash
    rosrun your_package_name your_python_file.py
    ```

Done! 

Now that we have an idea on what messages and topics are, let's explore how ROS messages and topics will be used in our banana_locator. Assume that this is the ROS workspace of the banana_locator project:
```
Banana_locator_workspace/
│
├── CMakeLists.txt
│
├── package.xml
│
├── stereo_camera_driver/      # Package 1
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   └── ...
│
├── AI_locator_algorithm/      # Package 2
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   └── ...
|
├── PID_controller_algorithm/  # Package 3
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   └── ...
│
└── ...
```

Here's how the Packages will communicate with each other:
1. ROS will spawn a node based onthe stereo_camera_driver package. This node will publish a `sensor_msgs/Image` message on a topic it created, named `/banana_images`.
2. ROS will spawn a node based onthe AI_locator_algorithm package. This package will subscribe to the `/banana_images` topic, because that's the information required to pinpoint where the banana is in space. It will also publish a `geometry_msgs/Pose` message on a topic it created, named `/target_position`.
3. ROS will spawn a node based on the PID_controller_algorithm package. This node will subscribe to the `/target_position` topic, because that's the information required to actuate the laser pointer to point at the banana.

And there you go! You've correctly connected a ROS computation graph. Now, time for the tedious process of debugging :>

<!-- Let's draft out the communication graph:
TODO -->

<!-- ### Topics
TODO -->
<!-- ## Services
On the other hand services are used for synchronous communication. This means that the client node will send a request to the server node for information and then wait for the server node to respond before continuing. Similar to messages we define service types in a `.srv` file. An example service definition is shown below:
```
TODO
``` -->