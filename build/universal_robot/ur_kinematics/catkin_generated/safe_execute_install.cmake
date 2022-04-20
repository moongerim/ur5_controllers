execute_process(COMMAND "/home/robot/workspaces/ur5_controllers/build/universal_robot/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robot/workspaces/ur5_controllers/build/universal_robot/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
