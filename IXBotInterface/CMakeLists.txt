project(IXBotCoreModel)

find_package(XBotCoreModel REQUIRED)

include_directories(include ${XBotCoreModel_INCLUDE_DIRS})
# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotInterface_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

add_library(XBotInterface SHARED ${XBotInterface_INCLUDES} 
                                 src/IXBotInterface.cpp
                                 src/Joint.cpp
                                 src/KinematicChain.cpp)
                                 
target_link_libraries(XBotInterface ${XBotCoreModel_LIBRARIES})

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