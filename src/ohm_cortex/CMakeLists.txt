cmake_minimum_required(VERSION 2.8.3)
project(ohm_cortex)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
 
 
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}) 

set(QT_HEADERS src/viz/main_window.hpp
               src/viz/Node.h
               src/viz/Link.h
                )                

set(QT_FORMS   src/viz/ui/main_window.ui
                )

set(QT_SOURCES src/viz/main_window.cpp
               src/viz/Node.cpp
               src/viz/Link.cpp
                )

QT4_WRAP_CPP(QT_MOC_HPP ${QT_HEADERS}
                )
                
QT4_WRAP_UI( QT_FORMS_HPP ${QT_FORMS}
                )

                  

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
  src/states/Init.cpp
)


## Declare a cpp executable
add_executable(ohm_statemachine_node src/ohm_statemachine.cpp)
add_executable(viz                   src/viz/viz_node.cpp           ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ohm_statemachine_node ohm_statemachine_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ohm_statemachine_node
   ${catkin_LIBRARIES}
   ohm_cortex
)

target_link_libraries(viz ${QT_LIBRARIES} ${catkin_LIBRARIES})

