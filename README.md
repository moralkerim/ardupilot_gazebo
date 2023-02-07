# Ardupilot Gazebo Plugin & Models
- [Ardupilot Gazebo Plugin & Models](#ardupilot-gazebo-plugin--models)
  - [Requirements :](#requirements-)
  - [Disclamer :](#disclamer-)
  - [Repository Structure :](#repository-structure-)
- [Getting Started :](#getting-started-)
  - [How to Install :](#how-to-install-)
  - [HELP](#help)
    - [How to Launch :](#how-to-launch-)

## Requirements :
Native Ubuntu Xenial(16.04 LTS) able to run full 3D graphics.

**Note :** Virtual Machine such as VMWare Player does not support full 3D graphics.

but, possible solution is here

Type follow in the terminal,
````
echo "export SVGA_VGPU10=0" >> ~/.bashrc
source ~/.bashrc
````
solution retreived from here http://answers.gazebosim.org/question/13214/virtual-machine-not-launching-gazebo/

**Note :** This just enables running gazebo in virtual machine, does not guarantee the performance and Gazebo require much of CPU & GPU processing power depending on what you are running the simulation.

ArduPilot setup for SITL launch
Gazebo version 7 or later

## Disclamer :
This is a playground until I get some time to push the correct patch to gazebo master (I got hard time to work with mercurial..)!  
So you can expect things to not be up-to-date.  
This assume that your are using Ubuntu 16.04

## Repository Structure : 
**models_gazebo :** Gazebo Original models retrieved from OSRF bitbucket repository (you can find more in https://bitbucket.org/osrf/gazebo_models/src)

**models :** Ardupilot SITL compatible models.

**worlds :** Ardupilot SITL example worlds.

**src :** source files for Gazebo - ArduPilot Plugin

**include :** header files for Gazebo - ArduPilot Plugin


**Common :**
````
git clone https://github.com/Rovense/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
Set Path of Gazebo Models / Worlds...
Open up .bashrc
````
sudo gedit ~/.bashrc
````
Copy & Paste Followings at the end of .bashrc file
````
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
````

**Gazebo-ROS Connection :**
Install gazebo_ros package.

````
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
````

Test gazebo_ros with empty world.

````
roscore
rosrun gazebo_ros gazebo
````

To verify that the proper ROS connections are setup, view the available ROS topics:
````
rostopic list
````

You should see within the lists topics such as:

````
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
````

Now go to launch folder inside repository and copy ardupilot_world.launch to gazebo_ros package launch folder:

````
sudo mv ardupilot_world.launch /opt/ros/melodic/share/gazebo_ros/launch/
````

## HELP

### How to Launch :  
Launch Ardupilot Software In the Loop Simulation for each vehicle.
On new terminal, Launch Gazebo with basic demo world.


**COPTER (3DR IRIS)**
On 1st Terminal(Launch ArduCopter SITL)
````
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0
````
On 2nd Terminal(Launch Gazebo with demo 3DR Iris model)
````
roslaunch gazebo_ros ardupilot_world.launch

````

After launch you should see lidar topic with:

````
rostopic list
````

````
/scan

````

and camera topics:

````
/findikv1/camera1/camera_info
/findikv1/camera1/image_raw
/findikv1/camera1/image_raw/compressed
/findikv1/camera1/image_raw/compressed/parameter_descriptions
/findikv1/camera1/image_raw/compressed/parameter_updates
/findikv1/camera1/image_raw/compressedDepth
/findikv1/camera1/image_raw/compressedDepth/parameter_descriptions
/findikv1/camera1/image_raw/compressedDepth/parameter_updates
/findikv1/camera1/image_raw/theora
/findikv1/camera1/image_raw/theora/parameter_descriptions
/findikv1/camera1/image_raw/theora/parameter_updates
/findikv1/camera1/parameter_descriptions
/findikv1/camera1/parameter_updates

````

### View data in Rviz :  

Open rviz with (after opening gazebo world):

````
rosrun rviz rviz
````

-Change fixed frame to "camera_link".
-Click "Add" at the bottom left corner.
-Select "Camera".
-Select Image topic as "/findikv1/camera1/image_raw"

Image should be displayed at bottom left corner.



## REFERANCES
https://github.com/SwiftGust/ardupilot_gazebo

http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros

http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros

http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros
