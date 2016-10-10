project(RobotInterfaceROS)

find_package(XBotCoreModel REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
                roscpp
                sensor_msgs
                custom_messages
                custom_services
                )
include_directories(include ${XBotCoreModel_INCLUDE_DIRS} 
                            ${CMAKE_SOURCE_DIR}/IXBotInterface/include
                            ${catkin_INCLUDE_DIRS}
                            )
# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotInterfaceROS_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

add_library(XBotInterfaceROS SHARED ${XBotInterfaceROS_INCLUDES} 
                                    src/RobotInterfaceROS.cpp)
                                 
target_link_libraries(XBotInterfaceROS XBotInterface
                                       yaml-cpp )
