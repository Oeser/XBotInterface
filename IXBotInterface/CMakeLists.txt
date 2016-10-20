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


project(IXBotCoreInterface)

find_package(XBotCoreModel REQUIRED)
find_package(RobotInterfaceROS REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sharedlib++ REQUIRED)

include_directories(include ${XBotCoreModel_INCLUDE_DIRS}
                            ${RobotInterfaceROS_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR}
                            ${kdl_parser_INCLUDE_DIRS}
                            ${sharedlib++_INCLUDE_DIRS}
                            )      
                            
# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotInterface_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

add_library(XBotInterface SHARED ${XBotInterface_INCLUDES} 
                                 src/IXBotInterface.cpp
                                 src/RobotInterface.cpp
                                 src/KinematicChain.cpp
                                 src/Joint.cpp
                                 src/ModelInterface.cpp)
                                 
target_link_libraries(XBotInterface ${XBotCoreModel_LIBRARIES}
                                    yaml-cpp
                                    ${sharedlib++_LIBRARIES}
                                    )

add_executable(test src/main_test.cpp)
target_link_libraries(test XBotInterface)

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
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

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