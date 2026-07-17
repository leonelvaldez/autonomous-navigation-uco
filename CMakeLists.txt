cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation_uco)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

install(DIRECTORY poi/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/poi
)

catkin_install_python(PROGRAMS scripts/send_nav_goal.py scripts/presence_detector.py scripts/dynamic_costmap_updater.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
