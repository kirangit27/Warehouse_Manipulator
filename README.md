# Warehouse_Manipulator
Pick and Place task done in an industrial environment using KUKA KR-16 model with the help of Gazebo, RViz and Moveit.

<a name="2.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;"> Environment Setup</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### Environment Setup
The project uses [ROS Noetic](http://wiki.ros.org/noetic) running on [Ubuntu 20.04.5 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/).

The following tools are used for simulation and motion planning:

* [Gazebo](http://gazebosim.org/): a physics based 3D simulator
* [RViz](http://wiki.ros.org/rviz): a 3D visualizer robot state visualization
* [MoveIt!](http://moveit.ros.org/): a ROS based software framework for motion planning, kinematics and robot control


##### Verify Project Tools

1\. Verify the version of gazebo installed with ROS (the project was tested on version 11.11.0)
```sh
$ gazebo --version
```

2\. If the installed gazebo version is an older one and in case gives some errors, update it as follows
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo11
```

##### Create ROS Workspace
3\. Create a [catkin](http://wiki.ros.org/catkin/conceptual_overview) workspace if haven't already
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

4\. Clone the project repository into the *src* directory of the catkin workspace
```sh
cd ~/catkin_ws/src
$ git clone https://github.com/kirangit27/Warehouse_Manipulator
```
5\. Unzip gazebo-pkgs.zip, general-message-pkgs.zip, and aws-robomaker-small-warehouse-world.zip. First two packages are used during grasping and the final one is used to setup the world environment in gazebo.
Instead of unzipping we can even clone these repositories into the *src* directory of the catkin workspace 
```sh
cd ~/catkin_ws/src
$ git clone https://github.com/JenniferBuehler/gazebo-pkgs
$ git clone https://github.com/JenniferBuehler/general-message-pkgs
$ git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
```

6\. Install missing dependencies if any
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

7\. Change the permissions of script files to turn them executable
```sh
$ cd ~/catkin_ws/src/Warehouse_Manipulator/kuka_prism/scripts
$ sudo chmod u+x plan.py
```

8\. Build the project
```sh
$ cd ~/catkin_ws
$ catkin_make
```

9\. source: Open [.bashrc file](https://unix.stackexchange.com/questions/129143/what-is-the-purpose-of-bashrc-and-how-does-it-work) (or  .zshrc) and add the following commands at the end 
```sh
source ~/catkin_ws/devel/setup.bash #use setup.zsh for zshrc
```

##### Simulation

10\. Open a fresh terminal and launch
```sh
$ roslaunch kuka_moveit_pkg kuka_moveit.launch
```
This should open Gazebo, RViz and rqt_image_view windows.

11\. Next to start the simulation, Open a fresh terminal and run the planner node
```sh
$ rosrun kuka_prism plan.py
```
