# GreenBot

This is the repository containing the ROS code for the GreenBot

# Setup

Do `catkin_make` on the workspace to install the whole thing. This should create the build/ and devel/ folders.

# Teleoperation

To perform teleoperation on the GreenBot, perform the following steps:

0. Make sure that the command center (main PC) and the Udoo board (GreenBot computer) are in the same network and can talk to each other.
   * Figure out the IP addresses (`ip a`) of each computer and ping each other.
   * On GreenBot, do `netcat -l <port e.g. 1234>` and on the main PC, do `netcat <greenbot ip address> <port>`. This should allow both computers to send messages to each other.
   * Once the IP addresses of both computers, edit the `ros_master_setup.sh` and `ros_node_setup.sh` files and populate the appropriate IP addresses
 
1. Source the `ros_master_setup` file on the main pc and Start ROS Master (`source config/ros_master_config.sh; roscore`)
   * It will be good to do the following here on both PCs: `source devel/setup.bash`
  
2. Start the `teleop_twist_keyboard.py` script on the main PC `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

3. Start the `teleop_greenbot.py` script on GreenBot `rosrun teleop_greenbot teleop_greenbot.py`

4. Mash some buttons!
