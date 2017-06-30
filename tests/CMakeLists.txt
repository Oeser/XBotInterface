cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME XBot_tests)
project(${PROJECTNAME})

#TESTS THAT USES GAZEBO
set(BUILD_SIMULATION_TESTS ON CACHE BOOL "Flag to compile tests that uses Gazebo")

if (BUILD_SIMULATION_TESTS)
    add_definitions( -DBUILD_SIMULATION_TESTS=true )
    #Gazebo
    find_package(Gazebo   REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    link_directories(${GAZEBO_LIBRARY_DIRS})
    include_directories(${GAZEBO_INCLUDE_DIRS})

    set(XBOT_SIMULATION_TESTS_VISUALIZATION ON CACHE BOOL "Flag to show simulation tests")
    if(XBOT_SIMULATION_TESTS_VISUALIZATION)
        add_definitions( -DOXBOT_SIMULATION_TESTS_VISUALIZATION=true )
    else()
        add_definitions( -DXBOT_SIMULATION_TESTS_VISUALIZATION=false )
    endif()
else()
    add_definitions( -DXBOT_COMPILE_SIMULATION_TESTS==false )
endif()

option(SHOW_TEST_RESULTS "After running tests, show result plots" TRUE)

# compite GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

# if(TRUE OR YCM_EP_DEVEL_MODE_OpenSoT)
# NOTE why we do not see XBotInterface here???
add_custom_command( TARGET XBotInterface POST_BUILD
                    COMMAND ${CMAKE_CTEST_COMMAND}
                    MAIN_DEPENDENCY XBotInterface
                    DEPENDS
                    testJoint
                    testKinematicChain
                    testXBotInterface
                    testModelInterface
                    )
# endif()

# gTests TODO add libs
SET(TestLibs XBotInterface ${GTEST_BOTH_LIBRARIES})

add_definitions(-DXBOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

# Unit tests related to Joint class
add_executable(testJoint     testJoint.cpp)
target_link_libraries(testJoint ${TestLibs})
add_dependencies(testJoint GTest-ext XBotInterface)
add_test(NAME XBot_testJoint COMMAND testJoint)

# Unit tests related to KinematicChain class
add_executable(testKinematicChain     testKinematicChain.cpp)
target_link_libraries(testKinematicChain ${TestLibs})
add_dependencies(testKinematicChain GTest-ext XBotInterface)
add_test(NAME XBot_testKinematicChain COMMAND testKinematicChain)

# Unit tests related to XbotInterface class
add_executable(testXBotInterface     testXBotInterface.cpp)
target_link_libraries(testXBotInterface ${TestLibs})
add_dependencies(testXBotInterface GTest-ext XBotInterface)
add_test(NAME XBot_testXBotInterface COMMAND testXBotInterface)

# Unit tests related to ModelInterface class
add_executable(testModelInterface     testModelInterface.cpp)
target_link_libraries(testModelInterface ${TestLibs})
add_dependencies(testModelInterface GTest-ext XBotInterface)
add_test(NAME XBot_testModelInterface COMMAND testModelInterface)


find_package(MATLAB QUIET)
find_package(PythonInterp)
if(MATLAB_FOUND AND PYTHONINTERP_FOUND AND SHOW_TEST_RESULTS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_CURRENT_BINARY_DIR}/../CTestCustom.cmake)
endif()

add_custom_target(copy_test_plotter_m ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.m" "${CMAKE_CURRENT_BINARY_DIR}/")

add_custom_target(copy_test_plotter_py ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.py" "${CMAKE_CURRENT_BINARY_DIR}/")
