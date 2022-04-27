## BASHRC check
1. gedit ~/.bashrc
2. edit path
3. source ~/.bashrc

## Testing with Real robot
1. robot ip parameters: ip: 192.168.1.2/ mask:255.255.255.0/ gateway:192.168.1.1
2. computer ip parameters: ip: 192.168.1.1 / mask:255.255.255.0/ gateway: 192.168.1.1

## URSIM install
install `ursim-3.6.1.30595`

## MPC testing process 20.04.2022
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
3. rosrun human_vrep human_sim
4. rosrun human_vrep test_human.py
5. rosrun mpc_low move_low
6. rosrun mpc_low one_way.py
7. rosrun mpc_high move_high

## Pytorch train:
1. source ./venv/bin/activate.sh
2. cd workspaces/ur5_mpc_ursim/src/nn_train
3. ./pytorch_train

## Pytorch test
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
3. cd workspaces/ur5_mpc_ursim/src/nn_train
4. python test_human.py
5. ./control.py
6. source ./venv/bin/activate
7. cd workspaces/ur5_mpc_ursim/src/nn_train
8. ./pytorch_test.py

## acceleration test
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
3. rosrun test_acceleration test_acceleration.py

# data collect using different init poses
1. ./start-ursim.sh
2. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=127.0.0.1
3. rosrun human_vrep human_sim
4. rosrun human_vrep human_spheres
5. rosrun mpc_low move_low
6. rosrun mpc_low init_designer.py
7. rosrun mpc_high move_high