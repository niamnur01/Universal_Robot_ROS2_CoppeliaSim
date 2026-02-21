# Universal Robots ROS2 CoppeliaSim
This project is a continuation of the work originally published by Hydran00. The original repository has been decommissioned, and this version includes subsequent modifications and updates.

This repository provides a ROS2 package, ur_coppeliasim, that implements an hardware interface of a Universal Robot using ROS2 controllers in CoppeliaSim.

## Installation

### Setup
- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)


### Prerequisites
#### 1. Install required packages:
```
sudo apt-get install xsltproc ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-hardware-interface
```  
```  
python3 -m pip install pyzmq cbor xmlschema
```  
#### 2. Install CoppeliaSim Edu:
- Download CoppeliaSim Edu from the [website](https://www.coppeliarobotics.com/downloads);
- Extract the downloaded folder in $HOME and rename it:
```
tar -xf ~/Downloads/CoppeliaSim_Edu*
mv ~/CoppeliaSim_Edu* ~/CoppeliaSim
```
#### 3. If you do not have one already set up, create a new ROS2 workspace (named ros2_ws):
```
mkdir -p ~/ros2_ws/src
```
#### 4. Clone this repo in your workspace src folder:  
```
cd ~/ros2_ws/src
git clone https://github.com/niamnur01/Universal_Robot_ROS2_CoppeliaSim.git ur_coppeliasim
```
#### 5. Build workspace:  
```
cd $HOME/ros2_ws/
colcon build
```

#### 6. Add required ROS2 message in CoppeliaSim:  
CoppeliaSim has a ros2 workspace internally, which is used by CoppeliaSim to build the required ROS2 messages. In this workspace we define the ROS2 messages we need:
  - Go to your CoppeliaSim folder and find the meta folder of the ros2 interface:
  ```
  cd ~/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/meta
  ```
  - Append required message definition to the file called ``interfaces.txt``:    
```
geometry_msgs/msg/Wrench  
geometry_msgs/msg/WrenchStamped  
std_msgs/msg/MultiArrayDimension  
std_msgs/msg/MultiArrayLayout 
std_msgs/msg/Float64MultiArray  
sensor_msgs/msg/JointState  
rosgraph_msgs/msg/Clock
``` 
- Alternatively, both steps can be obtained also with:
```
echo $'geometry_msgs/msg/Wrench\ngeometry_msgs/msg/WrenchStamped\nstd_msgs/msg/MultiArrayDimension\nstd_msgs/msg/MultiArrayLayout\nstd_msgs/msg/Float64MultiArray\nsensor_msgs/msg/JointState\nrosgraph_msgs/msg/Clock' >> ~/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt
```
#### 7. Build CoppeliaSim workspace
  - To build the Coppelia ros2 workspace we define the ``COPPELIASIM_ROOT_DIR`` environment variable:  
```
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim 
cd ~/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/ 
colcon build --symlink-install
```
  If you are encountering compilation errors you can try with this command:
```
VERBOSE=1 MAKEFLAGS=-j1 colcon build --symlink-install --event-handlers console_direct+ --parallel-workers 1
```

#### 8. (Optional) Install controller
We provide the functionalities of the simulator along with a [package](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2) which provides several Cartesian controllers. As this package will not be modified later on, we recommend to create another workspace for controllers in order to compile them just once. Alteratively, it can be compiled in the same workspace as before.
- Create a new workspace:
```
mkdir -p ~/controller_ws/src
```
- Clone the repo and install dependencies:
``` 
cd ~/controller_ws/src
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
```
- Download the following file required by the package compilation (we will not use this version of MuJoCo but the one included in CoppeliaSim, but in this way we prevent errors during the build):
``` 
cd; wget https://github.com/deepmind/mujoco/releases/download/2.1.1/mujoco-2.1.1-linux-x86_64.tar.gz
tar -xf mujoco-2.1.1-linux-x86_64.tar.gz
```
- Compile the controllers:
```
cd ~/controller_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```