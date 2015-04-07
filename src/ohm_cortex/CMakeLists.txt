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
    ohm_exploration
#    ohm_frontier_exploration
    ohm_srvs
    ohm_perception
    ohm_actors
    ohm_autonomy
    ohm_teleop
    tf
    )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${EIGEN_INCLUDE_DIRS}
		    )

set(QT_HEADERS src/viz/main_window.hpp
               src/viz/Node.h
               src/viz/Link.h
                )

set(QT_FORMS   src/viz/ui/main_window.ui
                )

set(QT_SOURCES src/viz/main_window.cpp
               src/viz/Node.cpp
               src/viz/Link.cpp
               src/viz/StateViz.cpp
                )

QT4_WRAP_CPP(QT_MOC_HPP ${QT_HEADERS}
                )

QT4_WRAP_UI( QT_FORMS_HPP ${QT_FORMS}
                )

add_service_files(
    FILES
    Force.srv
)



generate_messages()

catkin_package()

## Declare a cpp executable
add_executable(statemachine
               src/ohm_statemachine.cpp
               src/Context.cpp
               src/states/StatePing.cpp
               src/states/StatePong.cpp
               src/states/Teleoperated.cpp
               src/states/RejectVictim.cpp
               src/states/ConfirmVictim.cpp
               src/states/Waiting.cpp
               src/states/Inspect.cpp
               src/states/Approach.cpp
               src/states/Explore.cpp
               src/states/Drive.cpp
               src/states/Init.cpp
               src/states/TargetStack.cpp
               src/states/Teleoperated.cpp
               src/states/Manipulator.cpp
#               src/states/FrontierExplore.cpp
               src/states/FoundVictimCandidate.cpp
               src/states/LookAtTarget.cpp
               src/GetTransformation.cpp
               )
               
add_executable(simon_statemachine
               src/ohm_simon_statemachine.cpp
               src/Context.cpp
               src/states/StatePing.cpp
               src/states/StatePong.cpp
               src/states/Teleoperated.cpp
               src/states/RejectVictim.cpp
               src/states/ConfirmVictim.cpp
               src/states/Waiting.cpp
               src/states/Inspect.cpp
               src/states/Approach.cpp
               src/states/Explore.cpp
               src/states/Drive.cpp
               src/states/Init.cpp
               src/states/TargetStack.cpp
               src/states/Teleoperated.cpp
               src/states/Manipulator.cpp
#               src/states/FrontierExplore.cpp
               src/states/FoundVictimCandidate.cpp
               src/states/LookAtTarget.cpp
               src/GetTransformation.cpp
               )

add_executable(viz                   src/viz/viz_node.cpp           ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(statemachine ohm_cortex_generate_messages_cpp ohm_path_plan)

## Specify libraries to link a library or executable target against
target_link_libraries(statemachine
   ${catkin_LIBRARIES}
)

target_link_libraries(simon_statemachine
   ${catkin_LIBRARIES}
)

target_link_libraries(viz ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS statemachine
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
