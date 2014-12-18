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
    )
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)


include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
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

## Declare a cpp library
add_library(ohm_cortex
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
)


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
               )

add_executable(viz                   src/viz/viz_node.cpp           ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(statemachine ohm_statemachine_generate_messages_cpp ohm_path_plan)

## Specify libraries to link a library or executable target against
target_link_libraries(statemachine
   ${catkin_LIBRARIES}
   #ohm_path_plan
#   ohm_cortex
)

target_link_libraries(viz ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS statemachine
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
