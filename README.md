# Tutorial 1: Introduction to ROS2

#### Development of Inteligent Systems, 2024

The focus of the first exercise is to get familiar with the basics of the [ROS 2](http://www.ros.org) platform. You will learn to write your program within the system, to communicate with other programs and execute it properly. This exercise will introduce several important concepts that are crucial for further tutorials, so it is recommended that you refresh the topics after the end of the formal laboratory time at home.

## Setting up

To set up ROS 2 on your system, read the [official documentation](https://docs.ros.org/en/humble/index.html). In this course we will be using release **Humble** Hawksbill, which is a 4 year LTS release. It is advisable that you have at least one computer per group for development and visualization purposes.

The recommended operating systems are Ubuntu/Kubuntu/Lubuntu/etc. **22.04 LTS** [that support a Tier 1 native installation](https://www.ros.org/reps/rep-2000.html). Dual booting is generally the most hassle-free method if you have the option.

It's also possible to get ROS 2 installed on Windows 10 in several ways:
- as a [native install](https://docs.ros.org/en/iron/Installation/Windows-Install-Binary.html#)
- by installing [WSL](https://apps.microsoft.com/detail/9P9TQF7MRM4R?hl=en-us&gl=US) and [Ubuntu 22.04](https://apps.microsoft.com/detail/9PN20MSR04DW?hl=en-us&gl=US) from the Microsoft store
- via [Docker image](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- via [VMWare/Virtualbox Ubuntu 22.04 image](https://www.osboxes.org/ubuntu/)

Note that only the native install will likely be capable of running the Gazebo simulator with GPU acceleration, which is a requirement for realtime simulation.

Example code will be available for download as one [metapackage](https://docs.ros.org/en/humble/How-To-Guides/Using-Variants.html) (package that only contains other subpackages) per exercise.

## Concepts and terminology

ROS 2 is a complex distributed system that introduces a few concepts that are good to know under their established expressions:

- [Basic concepts](https://docs.ros.org/en/humble/Concepts/Basic.html): nodes, topics, parameters, launch files, cli tools
- [Intermediate concepts](https://docs.ros.org/en/humble/Concepts/Intermediate.html): coordinate frames, actions and tasks, message ontology
- [Advaned concepts](https://docs.ros.org/en/humble/Concepts/Advanced.html): build system, internal interfaces

More info on the most important concepts:

- [Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

- [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

- [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

- [Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

## Exploring ROS 2

Install the turtlesim package, if it's not already installed:

    sudo apt install ros-humble-turtlesim

All binary ROS packages are all typically available on apt following the ros-ros_version_name-package_name convention.

Open a new terminal window and run the command:

    ros2 run turtlesim turtlesim_node

The `ros2 run` command is the simplest way of running nodes. With the previous command we started the turtlesim_node which is located in the turtlesim package. In a third terminal run the command:

    ros2 run turtlesim draw_square

Now we have started the node draw_square from the turtlesim package. Now open another terminal window and try to find out what's going on in the ROS system with the following commands:

- `ros2 topic`
- `ros2 interface`
- `ros2 service`
- `ros2 param`
- `ros2 doctor --report`
- `ros2 run rqt_graph rqt_graph`

Note that by typing −h or −help after the command verb will print information about the usage of these commands.

Answer the following questions:

- Which **nodes** are currently active?
- What **topics** are currently active?
- What is the **message type** for each topic?
- What topics is each node **publishing** to?
- What topics is each node **subscribed** to?
- The used message types belong to which **packages**?
- Which **parameters** can be set on which nodes?
- Which **DDS middleware** is currently distributing messages?

Additionally, try to:
- Get a **visualization** of all the nodes and topics in the system.
- Get a printout of all the **packages** installed in the system.
- Get a printout of all the **messages** installed in the system.
- **Print** out the messages being published on each topic.
- **Publish** a message on each topic.

Explore the usage of other commands that are found in the [ROS 2 Cheatsheet](https://www.theconstructsim.com/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf). You can also find the full turtlesim documentation [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#prerequisites).

## Writing a package

Write a package that contains a simple program that you can run using the `ros2 run` command and outputs a string (e.g. "Hello from ROS!") to the terminal every second.

Use the following tutorials as a starting point:

- [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Now we can add two more nodes, one that sends a message and another one that retrieves it and prints it to the terminal.

- [Simple publisher and subscriber in C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Simple publisher and subscriber in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

------

### 🗎 Tips and tricks

For more useful code snippets, check out the [ROS 2 Cookbook](https://github.com/mikeferguson/ros2_cookbook).

ROS 2 is still under heavy development, as such any issues you encounter might not necessarily be entirely your fault. One example is the `SetuptoolsDeprecationWarning: setup.py install is deprecated.` warning for all python packages, which can be ignored.

Remember to run `colcon build` after every change or set up an alias for symlinking python scripts and config files e.g. `alias colcon_make='colcon build --symlink-install'` If you run into any build errors that don't make any sense try deleting the `build`, `log` and `install` directories and run a fresh build again. 

Here are some other useful colcon parameters:
- `--cmake-args=-DCMAKE_BUILD_TYPE=Release` (disable debugging, enable compile time optimization)
- `--executor sequential` (use single threaded compilation, takes longer but uses less memory which can be useful when compiling large projects on limited hardware)


When in doubt, reset the cache server: `ros2 daemon stop; ros2 daemon start`

![intro](figures/intro.png)
