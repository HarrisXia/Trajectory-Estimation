cmake_minimum_required(VERSION 2.8.3)
project(csv2bag)

add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  message_generation
  roscpp
  geometry_msgs
  sensor_msgs
  eigen_conversions
)


# add_message_files(DIRECTORY msg FILES)
# generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs
  )

# include boost
# find_package(Boost REQUIRED)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
#${Boost_INCLUDE_DIR}
)

# add_definitions("-std=c++0x -Wall -Werror")

add_executable(${PROJECT_NAME} src/csv2bag.cpp)
target_link_libraries(${PROJECT_NAME}
  #${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
