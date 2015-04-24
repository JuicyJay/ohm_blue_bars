cmake_minimum_required(VERSION 2.8.3)
project(ohm_cortex)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    genmsg
    actionlib_msgs
    actionlib
    #ohm_exploration
    #ohm_frontier_exploration
    ohm_srvs
    #ohm_perception
    ohm_actors
    ohm_autonomy
    ohm_teleop
    tf
    )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
#find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${EIGEN_INCLUDE_DIRS}
		    )



add_service_files(
    FILES
    Force.srv
)



generate_messages()

catkin_package()

## Declare a cpp executable
              
add_executable(simon_statemachine
               src/ohm_simon_statemachine.cpp
               src/Context.cpp


               src/states/Teleoperated.cpp
               src/states/Manipulator.cpp
               )
               




## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(statemachine ohm_cortex_generate_messages_cpp ohm_path_plan)

## Specify libraries to link a library or executable target against
#target_link_libraries(statemachine
#   ${catkin_LIBRARIES}
#)

target_link_libraries(simon_statemachine
   ${catkin_LIBRARIES}
)

#target_link_libraries(frontier_statemachine
#   ${catkin_LIBRARIES}
#)




install(TARGETS simon_statemachine 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
