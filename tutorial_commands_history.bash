# These are some of the commands that were used during the tutorial. This file should not be run, it is to be used as a reference.
# Also there might be wrong commands.
cd ROS2
colcon build --help
colcon build --symlink-install
source install/setup.bash 
ros2 run dis_tutorial1 py_simple_publisher.py 
ros2 run dis_tutorial1 py_simple_subscriber.py 
ros2 node --help
ros2 node list
ros2 node info /py_simple_publisher_node 
ros2 topic list
ros2 topic echo /chat 
ros2 interface show std_msgs/msg/String 
cd ROS2
colcon build --symlink-install
ros2 node list
ros2 node info /py_simple_publisher_node 
ros2 topic list
ros2 topic info /chat 
ros2 topic pub /chat std_msgs/msg/String {data:"Test publication"}
ros2 topic echo /chat
ros2 run dis_tutorial1 py_simple_subscriber.py 
ros2 run dis_tutorial1 py_simple_publisher.py 