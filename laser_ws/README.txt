This code starts a laser on one machine and allows for another machine with another laser to publish data to a topic via wifi. 

How to start it up:

1. Find the IP address of the machine on which you will be running Roscore and Rviz. This will be called the host machine. 

2. On the other machine, which will be referred to as a client, enter the following in a terminal after you set up all the ROS environment variables:

export ROS_MASTER_URI="http://ROSCORE_MACHINE_IP_ADDR:11311"

3. Plug in the laser USB cables to the computer. Once each laser is hooked up to its computer, run the following line on both machines: "sh scripts/laser_initialize.sh".

4. On the host machine, do "rosrun urg_node urg_node" and then in another terminal run "rosrun laser_package detect_node"

5. On the client, run "roslaunch launch_files/multiple_detectors.launch" and then in another terminal run "roslaunch launch_files/multiple_lasers.launch"

6. Start up Rviz under another terminal on the host. Under Global Options, change the Fixed Frame variable from "map" to "/laser".

7. Click "Add" at the bottom and go to the topics list. Select /scan and repeat this with /scan0. Change the Size and Color Transformer to suitable values.
