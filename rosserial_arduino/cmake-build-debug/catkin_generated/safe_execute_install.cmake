execute_process(COMMAND "/home/starsky/stealingfire/ws/wiredbot_ws/src/rosserial_arduino/cmake-build-debug/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/starsky/stealingfire/ws/wiredbot_ws/src/rosserial_arduino/cmake-build-debug/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
