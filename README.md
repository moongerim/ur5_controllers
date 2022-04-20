do not forget to check the bashrc file

install `ursim-3.6.1.30595`
## MPC testing process 20.04.2022
./start-ursim.sh
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
rosrun human_vrep human_sim
rosrun human_vrep test_human.py
rosrun mpc_low move_low
rosrun mpc_low one_way.py
rosrun mpc_high move_high

## How to control the UR5 robot on URSIM using MPC
./start-ursim.sh
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
rosrun human_vrep human_sim
rosrun human_vrep human_spheres.py
rosrun mpc_low move_low
rosrun mpc_low mpc_urscript.py
rosrun mpc_high move_high


## Pytorch train:
source ./venv/bin/activate.sh
cd workspaces/ur5_mpc_ursim/src/nn_train
./pytorch_train


## Pytorch test
./start-ursim.sh

roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1

cd workspaces/ur5_mpc_ursim/src/nn_train
python human_spheres.py
./control.py
source ./venv/bin/activate
cd workspaces/ur5_mpc_ursim/src/nn_train
./pytorch_test.py


## acceleration test
./start-ursim.sh
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
rosrun test_acceleration test_acceleration.py

# data collect using different init poses
./start-ursim.sh
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
rosrun human_vrep human_sim
rosrun human_vrep human_spheres
rosrun mpc_low move_low
rosrun mpc_low init_designer.py
rosrun mpc_high move_high