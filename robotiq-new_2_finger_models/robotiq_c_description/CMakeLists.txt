cmake_minimum_required(VERSION 2.8.3)
project(robotiq_c_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY model/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)

install(DIRECTORY robot/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robot)
