 #
#  Copyright (C) 2016 IIT-ADVR
#  Author: Arturo Laurenzi, Luca Muratore
#  email: arturo.laurenzi@iit.it, luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

project(XBotInterface)

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroInstallLib)

# find first YCM in order to have ${YCM_MODULE_PATH} defined for 3rd party software like Eigen3
find_package(YCM REQUIRED)

find_package(XBotCoreModel REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(Xenomai QUIET)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp=0.5.1)
message(STATUS "YAML_CPP Version: ${YAML_CPP_VERSION}")

message("test " ${kdl_parser_INCLUDE_DIRS})

message(WARNING "Consider activating the tests!")
# option(BUILD_TESTS "Compile XBotInterface tests" TRUE)
option(BUILD_TESTS "Compile XBotInterface tests" OFF)

set(ENABLE_XENO CACHE BOOL "ON")


include_directories(include ${XBotCoreModel_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${eigen_conversions_INCLUDE_DIRS}
                            ${YAML_CPP_INCLUDE_DIRS}
                            )



# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotInterface_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)


add_library(XBotInterface SHARED ${XBotInterface_INCLUDES}
                                 src/XBotInterface.cpp
                                 src/RobotInterface.cpp
                                 src/KinematicChain.cpp
                                 src/RobotChain.cpp
                                 src/ModelChain.cpp
                                 src/Joint.cpp
                                 src/Hand.cpp
                                 src/ModelInterface.cpp
                                 src/GenericSensor.cpp
                                 src/ForceTorqueSensor.cpp
                                 src/ImuSensor.cpp
                                 src/ControlMode.cpp
                                 src/bprinter/table_printer.cpp
                                 src/Logger.cpp
                                 src/RtLog.cpp
                                 )


target_include_directories(XBotInterface PUBLIC
                            ${XBotCoreModel_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${bprinter_INCLUDE_DIRS}
                            ${eigen_conversions_INCLUDE_DIRS}
                            )

target_link_libraries(XBotInterface PUBLIC ${XBotCoreModel_LIBRARIES}
                                    ${orocos_kdl_LIBRARIES}
                                    ${eigen_conversions_LIBRARIES}
                                    ${bprinter_LIBRARIES}
                                    ${YAML_CPP_LIBRARIES}
                                    matio pthread
                                    )
                                    
if( ${ENABLE_XENO} )
  if( ${Xenomai_FOUND})
  set_xeno_flags(XBotInterface)
  endif()
endif()




########################################################################

library_install(XBotInterface 1 0 0)

# doc
optional_build(doc doc ON)
# examples
optional_build(examples examples ON)

#######################
# Add Testing target  #
#######################
if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()
