cmake_minimum_required(VERSION 2.8.3)
project(robocup_challenge)

find_package(catkin REQUIRED)

catkin_package(
)

install(PROGRAMS
  scripts/utils_hb.py
  scripts/clean_up.py
  scripts/look_object.py
  scripts/detection.py
  scripts/manipulation.py
  scripts/gpd_server.py
  scripts/go_and_get_it.py
  scripts/go_to.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
