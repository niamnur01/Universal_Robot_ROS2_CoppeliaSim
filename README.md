# Universal Robots ROS2 CoppeliaSim
A ros package that implements an hardware interface for using ros2 controllers in CoppeliaSim.
## Installation  
### Setup:  
- ROS2 Humble  
- Ubuntu 22.04  
### Pre requisites:
#### 1. Install required packages:
```
sudo apt-get install xsltproc ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-hardware-interface
```  
```  
pip install xmlschema
```  
#### 2. Install CoppeliaSim Edu:
- Download CoppeliaSim Edu from the [website](https://www.coppeliarobotics.com/downloads)
- move the folder in $HOME and rename it
```
cd $HOME/Downloads
mv CoppeliaSim_Edu* $HOME/
mv CoppeliaSim_Edu* CoppeliaSim
```
#### 3. Create a ROS2 workspace
```
cd $HOME
mkdir ros2_ws
cd ros2_ws
mkdir src
```
#### 4. Clone this repo in your workspace src folder:  
```
cd $HOME/ros2_ws/src
git clone https://github.com/Hydran00/Universal_Robot_ROS2_CoppeliaSim.git coppeliasim_HWInterface
```
#### 5. Build workspace:  
```
cd $HOME/ros2_ws/
colcon build
```

#### 6. Add required ROS2 message in CoppeliaSim:  
CoppeliaSim has a ros2 workspace internally, which is used by CoppeliaSim to build the required ROS2 messages.  
  In this workspace we can define the ROS2 messages that we need:
  - Go to your CoppeliaSim folder and find the meta folder of the ros2 interface:
  ```
  cd $HOME/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/meta
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
  This last two steps can be done also with:
  ```
  echo $'geometry_msgs/msg/Wrench\ngeometry_msgs/msg/WrenchStamped\nstd_msgs/msg/MultiArrayDimension\nstd_msgs/msg/MultiArrayLayout\nstd_msgs/msg/Float64MultiArray\nsensor_msgs/msg/JointState\nrosgraph_msgs/msg/Clock' >> $HOME/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt
  ```
#### 7. Build CoppeliaSim workspace
  - Now you can build the Coppelia ros2 workspace (for compile it you also have to define the ``COPPELIASIM_ROOT_DIR`` environment variable):  
```
export COPPELIASIM_ROOT_DIR=$HOME/CoppeliaSim``  
cd $HOME/CoppeliaSim/programming/ros2_packages/sim_ros2_interface/ 
colcon build --symlink-install
```
  If you are encountering compilation errors you can try with this command:
```
VERBOSE=1 MAKEFLAGS=-j1 colcon build --symlink-install --event-handlers console_direct+ --parallel-workers 1
```

#### 8. Optional: Install controller
If you want to use the controller used in the video below you can follow these steps:
(I recommend to create another workspace for controllers in order to compile them just once)
- Create a new workspace:
```
cd $HOME
mkdir controller_ws
cd controller_ws
mkdir src
```
- Clone the repo and install dependencies:
``` 
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
```
- Download required file
``` 
cd $HOME
wget https://github.com/deepmind/mujoco/releases/download/2.1.1/mujoco-2.1.1-linux-x86_64.tar.gz
tar -xf mujoco-2.1.1-linux-x86_64.tar.gz
```
- Compile the controllers
```
cd $HOME/controller_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running the simulation  
*For this demo you need to follow point 8.*  
#### 0. Source terminals
Remember to always source both the controller_ws and ros2_ws workspaces:
```
source $HOME/ros2_ws/install/setup.bash
source $HOME/controller_ws/install/setup.bash
```
If you want, you can automate this procedure copying these two lines in the .bashrc so that they are executed as soon as you start a terminal. This can be done with:
```
echo $'source $HOME/ros2_ws/install/setup.bash  \nsource $HOME/controller_ws/install/setup.bash' >> $HOME/.bashrc
```

#### 1. Load the world
Open CoppeliaSim and load ``coppelia_world.ttt`` which is under ``$HOME/ros2_ws/src/coppeliasim_HWInterface`` through ``File->Open_Scene`` then click the play button. 
#### 2. Run the hardware interface + cartesian motion controller:
```
ros2 launch coppeliasim_HWInterface ur_coppelia_HWInterface.launch.py
```
In this video we are using a cartesian controller (https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2)



