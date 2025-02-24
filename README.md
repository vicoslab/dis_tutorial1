
# Tutorial 1: Introduction to ROS2

#### Development of Intelligent Systems, 2025

The focus of the first tutorial is to get familiar with the basics of the [ROS 2](http://www.ros.org) platform. You will learn to write your program within the system, to communicate with other programs and execute it properly. This tutorial will introduce several important concepts that are crucial for further tutorials, so it is recommended that you refresh the topics after the end of the formal laboratory time at home. After you explore the tutorial, you will need to submit two files as your **homework 1** on a link that will become available on Učilnica. The detailed instructions for the homework are at the end of this README.

## Setting up

To set up ROS 2 on your system, read the [official documentation](https://docs.ros.org/en/humble/index.html). In this course, we will be using release **Humble** Hawksbill, which is a 4 year LTS release.

The recommended operating systems are Ubuntu/Kubuntu/Lubuntu/etc. **22.04 LTS** [that support a Tier 1 native installation](https://www.ros.org/reps/rep-2000.html). Dual booting is generally the most hassle-free method if you have the option. We strongly recommend you to use one of the mentioned operating systems. At worse, at-least one of the team members should have it.

It's also possible to get ROS 2 installed on Windows 10 in several ways:
- as a [native install](https://docs.ros.org/en/iron/Installation/Windows-Install-Binary.html#)
- by installing [WSL](https://apps.microsoft.com/detail/9P9TQF7MRM4R?hl=en-us&gl=US) and [Ubuntu 22.04](https://apps.microsoft.com/detail/9PN20MSR04DW?hl=en-us&gl=US) from the Microsoft store
- via [Docker image](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- via [VMWare/Virtualbox Ubuntu 22.04 image](https://www.osboxes.org/ubuntu/)

Note that only the native install will likely be capable of running the Gazebo simulator with GPU acceleration, which is a requirement for real-time simulation. Please note that we might not be able to help you with issues you encounter with a Windows installation of ROS2.

Example code will be available for download as one [metapackage](https://docs.ros.org/en/humble/How-To-Guides/Using-Variants.html) (package that only contains other subpackages) per tutorial.

## Concepts and terminology

ROS 2 is a complex distributed system that introduces a few concepts that are good to know under their established expressions:

- [Basic concepts](https://docs.ros.org/en/humble/Concepts/Basic.html): nodes, topics, parameters, launch files, cli tools
- [Intermediate concepts](https://docs.ros.org/en/humble/Concepts/Intermediate.html): coordinate frames, actions and tasks, message ontology
- [Advanced concepts](https://docs.ros.org/en/humble/Concepts/Advanced.html): build system, internal interfaces

More info on the most important concepts:

- [Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
	- Nodes are standalone programs that perform some function, such as reading from a sensor or processing some data. They communicate with other nodes via topics. Nodes can subscribe to different topics, listen for data or events and react accordingly. They can also set up new topics to which they publish results.

- [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
	- Topics are data channels for communication between nodes. Each topic has a data type that defines the format of the data published to it. Nodes can subscribe to topics and receive data when it is published using a callback function.

- [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
	- Services are an on-demand method of node communication. A node can create a service that waits for requests and returns the response. The type of request and response message types is defined in a `.srv` file.

- [Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
	- Actions are used for more complex long-running tasks, such as navigation. The client sets a goal, and the action server provides feedback during the execution and notifies of the goal being reached. During execution, the action can also be cancelled if needed.

## Exploring ROS 2

Install the turtlesim package, if it's not already installed:

`sudo apt install ros-humble-turtlesim`

All binary ROS packages are all typically available on apt following the ros-ros_version_name-package_name convention.

Open a new terminal window and run the command:

`ros2 run turtlesim turtlesim_node`

The `ros2 run` command is the simplest way of running nodes. With the previous command we started the turtlesim_node which is located in the turtlesim package. In a third terminal, run the command:
`ros2 run turtlesim turtle_teleop_key`
This will allow you to control the turtle using the keyboard. Note that the terminal running the teleop requires the focus for the teleop to work.

Using the teleop node, messages are being sent to the turtlesim node. Open another terminal window and try to find out what's going on in the ROS system with the following commands:

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
- To which topics is each node **publishing**?
- To which topics is each node **subscribed**?
- What are the packages that define different **message types**?
- Which **parameters** can be set on which nodes?

Additionally, try to:
- Get a **visualization** of all the nodes and topics in the system.
- Get a printout of all the **packages** installed in the system.
- Get a printout of all the **messages** installed in the system.
- **Print** out the messages being published on each topic.
- **Publish** a message on each topic.
- **Set** the background color of turtlesim to a color of your choice.

Explore the usage of other commands that are found in the [ROS 2 Cheatsheet](https://www.theconstructsim.com/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf). You can also find the full turtlesim documentation [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#prerequisites).

## Writing a package

Write a package that contains a simple program that you can run using the `ros2 run` command and outputs a string (e.g. "Hello from ROS!") to the terminal every second.

Use the following tutorials as a starting point:

- [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Now we can add two more nodes, one that sends a message and another one that retrieves it and prints it to the terminal.

- [Simple publisher and subscriber in C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Simple publisher and subscriber in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Building a package

In order to build a custom package, you should move to the workspace root directory and run the following: `colcon build`. This will build all the packages within the `src` subdirectory. Since this could take a long time, the flag `--packages-select <package name>` can be used to only build selected packages. The build process will also create setup files in the `install` subdirectory. To make the newly built packages visible to ROS2, you should run the following: `source install/local_setup.bash`. Then, you will be able to run your custom nodes using the following command: `ros2 run <package_name> <node_name>`.

## Services

In the tutorial you have examples of creating a service and a client as well as defining a custom service interface. We define a custom service by specifying the structure of the request that the service will accept and the response that it will return. 

Use the following tutorials as a starting point:

- [Simple service and client in C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Simple publisher and subscriber in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

See the [interfaces doc](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html) for a reference of all the possible native data types. The `ros2 interfaces list` command shows all built messages, services, and actions, which are themselves types that can be used in custom interfaces.

Note that only `ament_cmake` packages are capable of building interfaces (i.e. services and messages), so if you have an `ament_python` package you'll need a separate one for message definitions.

------

### 🗎 Tips and tricks

For more useful code snippets, check out the [ROS 2 Cookbook](https://github.com/mikeferguson/ros2_cookbook).

Since running several different nodes in parallel requires multiple terminals, we recommend you to use a terminal emulator that is able to display several terminals in a single window. A good choice is [Terminator](https://gnome-terminator.org/). You can install it with `sudo apt install terminator`. It should also be preinstalled on the classroom computers. You can split the screen horizontally or verically with the shortcuts `ctrl + shift + e` and `ctrl + shift + o`, respectively.

ROS 2 is still under heavy development, and as such, any issues you encounter might not necessarily be entirely your fault. One example is the `SetuptoolsDeprecationWarning: setup.py install is deprecated.` warning for all python packages, which can be ignored.

In order for your packages to be visible to ROS2, you will need to run `source install/setup_local.bash` in your workspace directory after build. As this holds for all terminals, a good idea is to add the following to your `.bashrc` file: `source /home/<user>/ros2_workspace/install/local_setup.bash`

Running `colcon build` is necessary after any source code changes, but only for C++ nodes (or messages/services). If writing python nodes, and you build the package using the flag `--symlink-install`, the links to your python source files were created and changing your code should work without building the package again. If you run into any build errors that don't make any sense, try deleting the `build`, `log` and `install` directories and run the build again.

Here are some other useful colcon parameters:
- `--cmake-args=-DCMAKE_BUILD_TYPE=Release` (disable debugging, enable compile time optimization)
- `--executor sequential` (use single threaded compilation, takes longer but uses less memory which can be useful when compiling large projects on limited hardware)


When in doubt, reset the cache server: `ros2 daemon stop; ros2 daemon start`

# Homework 1

## Part 1

Open the file `homework1_answers.txt` from this repository, follow the instructions and include your answers to the questions in the marked slots in the same file. Then, upload the file on the available link on Učilnica.

## Part 2

After you have installed and tested ROS2, as well as set up your own workspace you should:

- Create a new package. In the package you can use C++ or Python.
- Create a custom message type, that has a string fields, an integer field, and a bool field.
- Create a custom service type, where the request contains a string field and an array of integers, and the response contains a string field and an integer field.
- Create a publisher node, that periodically sends a message on a certain topic. You should use the custom message you defined.
- Create a subscriber node, that recieves the message on the same topic and prints out its contents.
- Create a custom service node that accepts an array of integers and responds with the SUM of the recieved integers. Use the custom service you defined.
- Create a custom client node that generates random sequences of 10 integers, calls the service node, and prints out the response that it recieves from your service node.

Compress your package into a single .zip archive named `homework1_package.zip` and upload the file on the available link on Učilnica.