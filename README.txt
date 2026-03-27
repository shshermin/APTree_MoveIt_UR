How to setup and run the robot.
#connection to the C# code
cd /home/shermin/ws_moveit
source install/setup.bash
pip install flask
python3 /mnt/c/Users/sherk/Documents/BehaviorTreeMainProject/APTreeExecutionEngine/python_service/moveit_bridge_service.py



#terminal 1
cd /home/shermin/ws_moveit
source install/setup.bash

source /opt/ros/jazzy/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.100 kinematics_params_file:=/home/shermin/ws_moveit/src/hello_moveit/config/robot_calibration.yaml

#terminal 2
cd /home/shermin/ws_moveit
source install/setup.bash
ros2 launch hello_moveit demo.launch.py ur_type:=ur10e
ros2 launch hello_moveit demo.launch.py ur_type:=ur10e end_effector_type:=gripper
ros2 launch hello_moveit demo.launch.py ur_type:=ur10e end_effector_type:=nailgun
ros2 launch hello_moveit demo.launch.py ur_type:=ur10e end_effector_type:=both

#Terminal 3
cd /home/shermin/ws_moveit
source install/setup.bash
#only flange no gripper
python3 src/hello_moveit/scripts/move_to_task.py   
# With gripper — plans to fingertip
python3 src/hello_moveit/scripts/move_to_task.py --end_effector_type gripper --both_loaded --x -0.257 --y -0.974 --z -0.009 --yaw 130

# With nailgun — plans to nozzle
python3 src/hello_moveit/scripts/move_to_task.py --end_effector_type nailgun --both_loaded --x -0.257 --y -0.974 --z -0.009


# on the real robot: 


# building a package


Setting up the port connection in pawershel for WSL2

######using mirroring in poweshell

#Step 1: Create the file and add content in powershell

# This command creates the file and adds the necessary lines for Mirrored Networking
Set-Content -Path "$env:USERPROFILE\.wslconfig" -Value "[wsl2]`nnetworkingMode=mirrored`nlocalhostForwarding=true"

#Step 2: Verify the file content in powershell

Get-Content "$env:USERPROFILE\.wslconfig"


#Step 3: Important Restart in powershell

wsl --shutdown

#step 4: Step 4: Confirm it worked (in wsl)


hostname -I which should be your pc IP: the ethernet one 

