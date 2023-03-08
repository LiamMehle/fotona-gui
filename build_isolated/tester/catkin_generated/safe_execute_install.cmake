execute_process(COMMAND "/home/liam/ros/new_catkin/build_isolated/tester/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/liam/ros/new_catkin/build_isolated/tester/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
