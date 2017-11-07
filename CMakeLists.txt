 #
#  Copyright (C) 2016 IIT-ADVR
#  Author: Luca Muratore
#  email: luca.muratore@iit.it
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

include(ExternalProject)
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

# find first YCM in order to have ${YCM_MODULE_PATH} defined for 3rd party software like Eigen3
find_package(YCM REQUIRED)

find_package(XBotCoreModel REQUIRED)
find_package(sharedlibpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(eigen_conversions REQUIRED)

find_package(Xenomai QUIET)

option(BUILD_TESTS "Compile XBotInterface tests" TRUE)



include_directories(include ${XBotCoreModel_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${sharedlibpp_INCLUDE_DIRS}
                            ${bprinter_INCLUDE_DIRS}
                            )



# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotInterface_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)
#

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
                                 src/ConfigHelper.cpp
                                 src/Logger.cpp
                                 src/RtLog.cpp
                                 )


target_include_directories(XBotInterface PUBLIC
                            ${XBotCoreModel_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${sharedlibpp_INCLUDE_DIRS}
                            ${bprinter_INCLUDE_DIRS}
                            )

target_link_libraries(XBotInterface PUBLIC ${XBotCoreModel_LIBRARIES}
                                    ${sharedlibpp_LIBRARIES}
                                    ${orocos_kdl_LIBRARIES}
                                    ${eigen_conversions_LIBRARIES}
                                    ${bprinter_LIBRARIES}
                                    matio pthread
                                    )
                                    
if(${Xenomai_FOUND})
set_xeno_flags(XBotInterface)
endif()


add_executable(test_exec src/main_test.cpp)
target_link_libraries(test_exec XBotInterface)

# Enable srdf generation from xacro
set(ENABLE_SRDF_FROM_XACRO_GENERATION CACHE BOOL "ON")
if ( ${ENABLE_SRDF_FROM_XACRO_GENERATION} )
    # xacro
    find_package(xacro REQUIRED)
    # iterate over xacro files
    file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/configs/srdf/*.xacro)
    foreach(it ${xacro_files})
        # remove .xacro extension
        string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
        set(output_filename ${CMAKE_MATCH_1})

        # create a rule to generate ${output_filename} from {it}
        xacro_add_xacro_file(${it} ${output_filename})

        list(APPEND srdf_files ${output_filename})
    endforeach(it)

    # add an abstract target to actually trigger the builds
    add_custom_target(srdf_from_xacro_files ALL DEPENDS ${srdf_files})
endif()

########################################################################

set(VARS_PREFIX "XBotInterface")

set(XBotInterface_MAJOR_VERSION 0)
set(XBotInterface_MINOR_VERSION 1)
set(XBotInterface_PATCH_VERSION 0)
set(XBotInterface_VERSION ${XBotInterface_MAJOR_VERSION}.${XBotInterface_MINOR_VERSION}.${XBotInterface_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XBotInterface PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XBotInterface PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}")

install(TARGETS XBotInterface
        EXPORT XBotInterface
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XBotInterface)


include(InstallBasicPackageFiles)
install_basic_package_files(XBotInterface VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)


include(AddUninstallTarget)

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
