execute_process(COMMAND "/home/docker/bind_mount/build/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/docker/bind_mount/build/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
