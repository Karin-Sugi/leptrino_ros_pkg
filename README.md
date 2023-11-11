This is to get data from leptrino FT sensor & publish it as a ros topic. This is only for
Leptrino 6 DOF force-torque sensors (**PFS080YS102U6S**). 
Other type may have troube with using. PLEASE check your sensor type&number and if you use another type, compare this scripts to your official sample scripts!

## How to use
1. Download/clone this repository
2. Check the port name of your comuter!
    1. run this pkg with `roslaunch leptrino_ros_pkg leptrino.launch` command
    2. if the pkg run successfully, fine. Your port name is the same with the default `/dev/ttyACM0`
    3. if the pkg failed to run, check your `/dev/` file and find your port name there!
    4. change the port name in 46 line of *leptrino_force_torque.cpp* file  
        `std::string g_com_port = "/dev/ttyACM0";`  
        change `/dev/ttyACM0` to `your port name`

## Node
* **leptrino**:
  This script has only one node named *leptrino*. This node convert Force&Torque data to ros message type (`geometry_msgs/WrenchStamped`) and publish it to ros topic.
  The topic name is...
  * */leptrino/force_torque*: when you run `roslaunch leptrino_ros_pkg leptrino.launch`
  * */force_torque*: when you run `rosrun leptrino_ros_pkg leptrino`