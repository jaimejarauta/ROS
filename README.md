# ROS and Mountain Car Dynamics Simulator

## Project Description
This project aims to simulate a mountain car scenario within the ROS/Gazebo simulation environment. It focuses on developing a simulation where a car must navigate up a steep hill, incorporating elements such as varying landscape height, friction, kinetic energy, and simple control mechanisms to accurately represent the dynamics of a mountain car.

## Contributors
- **Professor**: Dr. Richard B. Sowers
- **Student Researcher**: Jaime Jarauta Gastelu

## Installation Requirements
- ROS Noetic
- Gazebo 11

## Documentation
For further details and documentation, visit [here](https://uillinoisedu-my.sharepoint.com/:w:/g/personal/jaimej2_illinois_edu/EZ0EjiUl5otDqpld2Sg_8F0BGazftnWFsAoBPXAnMlxfsQ?e=6txSLb).

## Repository Overview
### ROS_V1: Initial Version
- **Hill/Model**: STL model of the V1 hill, including an Excel workbook of formulas used.
- **Other Worlds**: Original ROS package attempt to create a pub/sub network (not functional) and other worlds. `my_mesh.world` includes the hill object.
- **cartest.py**: Original Python script provided.
- **gazebo-11 files**: Standard Gazebo files and some additional models.

#### How to Run
```bash
roscore
cd ~/catkin_ws/
# Open a new terminal
source devel/setup.bash
rosrun car_test talker.py
```

### ROS_V2: Controlled Differential Car
- Incorporates a differential car controllable via terminal using `twist.teleop`.
- **diff_drive_ws**: Contains the latest and functional ROS package for the controlled car.

#### How to Run
```bash
cd diffDrive_ws
catkin_make
# Open a new terminal
source devel/setup.bash
roslaunch simulation_environment diff_drive.launch
# Open a new terminal
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# Optional
rostopic list
```

### ROS_V3: Automated Movement and ML Controls
Focuses on automating the movement process and applying ML controls to the car.

#### ROS_V3.1
- Launches an empty Gazebo world; running the Python script moves the car to a pre-determined (x,y) position.

#### ROS_V3.2
- Launches a Gazebo world with the hill file; runs the same Python script as ROS_V3.1 considering gravity, mass, and other parameters.

#### ROS_V3.3
- Includes a Python script applying forward/backward acceleration based on the car's movement direction.
- Includes as well a simple PID control in order to prevent the car from turning as it reaches speeds close to zero.
- Available in two versions:
  - **catkin_ws_flat**: Runs the script without the hill to observe linear speed changes without external perturbations.
  - **catkin_ws**: Runs the script with the hill.

#### How to Run (for each version, open each line in a new terminal window)
```bash
cd ~/catkin_ws # or cd ~/catkin_ws_flat for the flat version
roscore

source devel/setup.bash
roslaunch car_test car.launch

# If you want to see the linear speed values
source devel/setup.bash
rostopic echo /car/diff_drive_controller/odom/twist/twist

source devel/setup.bash
rosrun car_test code.py
```
