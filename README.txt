How to setup and run the robot.

terminal 1
cd /home/shermin/ws_moveit
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.1.100

terminal 2
cd /home/shermin/ws_moveit
source install/setup.bash
ros2 launch hello_moveit demo.launch.py ur_type:=ur10

Terminal 3
cd /home/shermin/ws_moveit
source install/setup.bash
python3 src/hello_moveit/scripts/move_to_task.py



# building a package



Setting up the port connection in pawershel for WSL2

using mirroring in poweshell

#Step 1: Create the file and add content

# This command creates the file and adds the necessary lines for Mirrored Networking
Set-Content -Path "$env:USERPROFILE\.wslconfig" -Value "[wsl2]`nnetworkingMode=mirrored`nlocalhostForwarding=true"

#Step 2: Verify the file content

Get-Content "$env:USERPROFILE\.wslconfig"


#Step 3: Important Restart

wsl --shutdown

#step 4: Step 4: Confirm it worked (in wsl)


hostname -I which should be your pc IP: the ethernet one 

