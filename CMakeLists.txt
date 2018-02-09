cmake_minimum_required(VERSION 2.8.3)
project(pos2bag)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/pos2bag_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
