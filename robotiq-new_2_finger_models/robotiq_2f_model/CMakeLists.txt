cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_model)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY model/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)
