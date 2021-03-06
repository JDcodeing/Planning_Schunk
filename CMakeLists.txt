cmake_minimum_required(VERSION 2.8.3)
project(planning_test)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(FMGPlanner src/FMGPlanner.cpp)
target_link_libraries(FMGPlanner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS FMGPlanner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(collision_check src/collision_check.cpp)
target_link_libraries(collision_check ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS collision_check DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
target_link_libraries(planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
target_link_libraries(planning_scene_ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning_api_tutorial src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_pipeline_tutorial src/planning_pipeline_tutorial.cpp)
target_link_libraries(planning_pipeline_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_pipeline_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(kinematic_test src/kinematic_test.cpp)
target_link_libraries(kinematic_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS kinematic_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(myplan src/myplan.cpp)
target_link_libraries(myplan ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myplan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(add_obstacle_test src/add_obstacle_test.cpp)
target_link_libraries(add_obstacle_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_obstacle_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(generate_traj src/generate_traj.cpp)
target_link_libraries(generate_traj ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS generate_traj DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(visualstate src/visualstate.cpp)
target_link_libraries(visualstate ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS visualstate DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(simpletraj src/simpletraj.cpp)
target_link_libraries(simpletraj ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS simpletraj DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

