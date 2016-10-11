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

add_library(RobotInterfaceROS STATIC ${XBotInterfaceROS_INCLUDES} 
                                    src/RobotInterfaceROS.cpp)
                                 
target_link_libraries(RobotInterfaceROS XBotInterface
                                       yaml-cpp )
                                       
add_executable(ros_test_exec src/test_ros.cpp)
target_link_libraries(ros_test_exec RobotInterfaceROS)

                                       
########################################################################

set(VARS_PREFIX "RobotInterfaceROS")

set(RobotInterfaceROS_MAJOR_VERSION 0)
set(RobotInterfaceROS_MINOR_VERSION 1)
set(RobotInterfaceROS_PATCH_VERSION 0)
set(RobotInterfaceROS_VERSION ${RobotInterfaceROS_MAJOR_VERSION}.${RobotInterfaceROS_MINOR_VERSION}.${RobotInterfaceROS_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(RobotInterfaceROS PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(RobotInterfaceROS PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS RobotInterfaceROS
        EXPORT RobotInterfaceROS
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all RobotInterfaceROS dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS RobotInterfaceROS)


include(InstallBasicPackageFiles)
install_basic_package_files(RobotInterfaceROS VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)