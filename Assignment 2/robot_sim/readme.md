Robotic 3D Simulation with Gazebo and ROS 
================================

This is a simple robotic simulation with the aim of utilizing the main core features of ROS for creating robotic softwares.

Main Core Features of this Implementation
----------------------
* A command-line-based user interface to send positional goals to the robot
* Nodes communication based on custom messages and publisher-subscriber architecture
* Utilizing ROS services to perform short-running tasks like reporting the goal summary
* Utilizing the ROS action servers to perform long-running tasks like robot planning and navigation
* Using launch files to set global parameters and manage running of multiple nodes

Running the Simulation
----------------------

You can run the program with:
```bash
$ roslaunch robot_sim mylaunch.launch
```

After running the launch file, 4 terminal windows will be appaired as listed below:
* A main terminal for running the roscore, services and action servers, gazebo and rviz 
* A terminal that shows the user interface for setting a new positional goal for the robot or canceling it
* A terminal that prints the kinematic data of the robot like distance to target and average velocities in both x and y axises
* A terminal that runs a goal-summary service that prints the current number of reached or canceled goals of the robot

Package Name and Folders Structure
----------------------

The project published under the name 'robot_sim' and contains the following files and folders:
* Files & Folders:
  * **action:** `Contains .action files that describes structure of the request and response of the action server.`
  * **config:**  `Contains .rviz files that describes the runtime configurations of the RVIZ.`
  * **launch:** `Contains .launch files that set the global parameters and manage running of multiple nodes.`
  * **msg:** `Contains .msg files that describe the structure of custom messages.`
  * **script:**  `Contains the code of different nodes of the current ROS application:`
    * **bug_as.py:** `The main node of the application that runs an action server whose responsibility is running the planning algorithm.`
    * **go_to_point_service.py:** `The node is responsible for navigating the robot from current position to the target position by switching between the 'go_straight_ahead' or 'fix_yaw' motions.`
    * **goal_summary.py:** `The node is responsible for running a service to report the current number of reached or canceled goals.`
    * **kinematic_monitor.py:** `The node is responsible for printing the kinematic data like distance to target and average velocities in both x and y axis.`
    * **odometry.py:** `The node is responsible for publishing the position and velocity data of the robot.`
    * **user_interface.py:** `The node is responsible for showing an operation-select menu to the user for setting a new positional goal or cancelling the current goal of the robot.`
    * **wall_follow_service.py:** `The node is responsible for creating a service to find and follow a wall during the robot's motion.` 
  * **srv:** `Contains .srv files that describe structure of the request and response of the services.`
  * **urdf:** `Contains .gazebo files that describe a gazebo model like a robot and all its features.`
  * **world:** `Contains .world files that describe the robot's environment.`
  * **CMakeLists.txt:** `The file is used to build the project with the Catkin and CMake build systems.`
  * **package.xml:** `The file is used to provide the general information of the released package and also the build and execution dependencies of the application.`
*

Pseudocode of Node 1
----------------------
Node 1 is implemented by two basics nodes as listed below:
* **user-interface** node:
```python
initial the client variable by None

def showMenu():
  show the operation-select menu

  userSelect = input("Please select the operation from the menu:") 
  if(userSelect == 1):
    read the x element of the target position 
    read the y element of the target position 
    if(x , y are float numbers):
       sendGoal(client, x, y) to the action-server to run the planning algorithm
    else:
      show an error message
  if(userSelect == 2):
    cancelGoal(client) to cancel the current goal   
  if(userSelect == 3):
     sys.exit(1) to terminate the current node   
  else: 
     show an error message   


main():
   showWelcome() message to the user
   inital a ros node by rospy.init_node('client')
   create a client = createClient()

   while ros is okay:
      showMenu() to the user          
```

* **odometer** node:

```python
initial the odoMsg variable as a custom message of type OdoSensor()

main():
   
   inital a ros node by rospy.init_node('odemeter')
   create a subscriber on odom topic to receive the Odometry data
   create a publisher to publish the OdoSensor data on topic odometer
   configure the clock rate = rospy.Rate(10) of the node

   while ros is okay:
      publish(odoMsg) data
      rate.sleep() to pause the publishing process as configured
       
```