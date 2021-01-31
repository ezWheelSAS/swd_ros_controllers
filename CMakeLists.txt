#
#                     Copyright (C) 2019 EZ-Wheel S.A.S.
#
# -----------------------------------------------------------------------------

# CMake version required ----------------------------------
cmake_minimum_required(VERSION 3.0)
project(ezw_ros_controllers)

if(NOT DEFINED PRJ_VERSION)
    set(PRJ_VERSION "1.0.0")
endif(NOT DEFINED PRJ_VERSION)


# ROS find packages ------------------------------------------------------------
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)


#------------------------------------------------------------------------------
# Build definition (to do before calling 'project()')
#------------------------------------------------------------------------------

# Build type
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# CASE OF C PROJECT
set(CMAKE_C_STANDARD 99)
if(NOT CMAKE_C_FLAGS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wformat-security " CACHE STRING "C99 compilation flags" FORCE)
endif(NOT CMAKE_C_FLAGS)

# CASE OF CPP PROJECT
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_CXX_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wformat-security" CACHE STRING "C++11 compilation flags" FORCE)
endif(NOT CMAKE_CXX_FLAGS)


# The code needs C++14 to work!
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#------------------------------------------------------------------------------
# Project definition, variable and dependencies
#------------------------------------------------------------------------------

set(CMAKE_PROJECT_NAME ${PROJECT_NAME})
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_PROJECT_BRIEF "EZW Wheel-sMC ROS package")

# if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#     set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/image CACHE PATH "" FORCE)
# endif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set(PKG_CONFIG_PATH ${CMAKE_SOURCE_DIR}/../image/lib/pkgconfig CACHE PATH "")
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${PKG_CONFIG_PATH}")
message(STATUS "Pkgconfig extra path are: $ENV{PKG_CONFIG_PATH} ")

# Allow CTest
enable_testing()

# Allow subdirectory test and config
option(ENABLE_TESTS "Enable Tests" 1)

# find_package(Doxygen)
# option(ENABLE_DOCS "Build API documentation" ${DOXYGEN_FOUND})

# Module dependencies used as pkgconfig or direct filepath/dirpath
set(LIST_MODULES_PKG_CONFIG_NAME "ezw-types CommonAPI-DBus ezw-smc-service ezwLog ezw-io-service-client ezw-canopen-service-client ezw-common ezw-tools" CACHE STRING "Lists of pkgconfig modules dependencies" FORCE)

unset(__pkg_config_checked_EZW_TYPES CACHE)
unset(__pkg_config_checked_CAPI CACHE)
unset(__pkg_config_checked_EZW_LOG CACHE)
unset(__pkg_config_checked_EZW_SMC_SERVICE_CLIENT CACHE)
unset(__pkg_config_checked_EZW_IO_SERVICE_CLIENT CACHE)
unset(__pkg_config_checked_EZW_CANOPEN_SERVICE_CLIENT CACHE)
unset(__pkg_config_checked_EZW_TOOLS CACHE)
unset(__pkg_config_checked_EZW_COMMON CACHE)

set(EZW_TYPES_PKG_CONFIG_NAME "ezw-types" CACHE STRING "ezw-types pkg config name")
set(CAPI_PKG_CONFIG_NAME "CommonAPI-DBus" CACHE STRING "Module CAPI_PKG_CONFIG pkg config name")
set(EZW_SMC_SERVICE_CLIENT_PKG_CONFIG_NAME "ezw-smc-service-client" CACHE STRING "ezw-smc-service-client pkg config name")
set(EZW_LOG_PKG_CONFIG_NAME "ezwLog" CACHE STRING "ezwLog pkg config name")
set(EZW_IO_SERVICE_CLIENT_PKG_CONFIG_NAME "ezw-io-service-client" CACHE STRING "ezw-io-service-client pkg config name")
set(EZW_CANOPEN_SERVICE_CLIENT_PKG_CONFIG_NAME "ezw-canopen-service-client" CACHE STRING "Module ezw-canopen-service-client pkg config name")
set(EZW_TOOLS_PKG_CONFIG_NAME "ezw-tools" CACHE STRING "Module EZW_TOOLS_CONFIG pkg config name")
set(EZW_TOOLS_PKG_CONFIG_NAME "ezw-common" CACHE STRING "Module EZW_COMMON_CONFIG pkg config name")

#------------------------------------------------------------------------------
# Project version
#------------------------------------------------------------------------------

# Get the latest abbreviated commit hash of the working branch
execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(REGEX REPLACE
    "^([\\.0-9]+).*"
    "\\1"
    tmp_VERSION
    "${PRJ_VERSION}"
)

string(REGEX REPLACE
    "^([0-9]+)\\..*"
    "\\1"
    tmp_SOVERSION
    "${tmp_VERSION}"
)

# Major version is for product/marketing purpose
# Minor version is incremented everytime it gets public
set(MAJ_MIN_VERSION ${tmp_VERSION} CACHE STRING "Major and minor version" FORCE)

# Release version is incremented everytime it gets use inside the company
set(RELEASE_VERSION ${tmp_SOVERSION} CACHE STRING "Release version" FORCE)

# Build version is generated using the hash of the current commit build with
set(BUILD_VERSION ${GIT_COMMIT_HASH} CACHE STRING "Build version" FORCE)

#------------------------------------------------------------------------------
# pkgconfig
#------------------------------------------------------------------------------

find_package(PkgConfig REQUIRED)
# if(PKG_CONFIG_FOUND)
#     # Produce a pkg-config file for DBus Client project
#     configure_file (
#         ${CMAKE_SOURCE_DIR}/${CMAKE_Pezw/diff_drive_controller/ROJECT_NAME}-client.pc.in
#         ${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}-client.pc
#         @ONLY
#     )
#     install (
#         FILES ${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}-client.pc
#         DESTINATION lib/pkgconfig
#     )
# endif (PKG_CONFIG_FOUND)

# Find ROS related packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

#------------------------------------------------------------------------------
# Project configuration
#------------------------------------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR}/include)

if (HAS_CONFIG)
    message(STATUS "Adding config from ${CMAKE_SOURCE_DIR}/config/")
    include_directories(${CMAKE_SOURCE_DIR}/config)
endif (HAS_CONFIG)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Use project pkg config
pkg_check_modules(EZW_TYPES REQUIRED ezw-types)
pkg_check_modules(CAPI REQUIRED CommonAPI-DBus)
pkg_check_modules(EZW_SMC_SERVICE_CLIENT REQUIRED ezw-smc-service-client)
pkg_check_modules(EZW_LOG REQUIRED ezwLog)
pkg_check_modules(EZW_IO_SERVICE_CLIENT REQUIRED ezw-io-service-client)
pkg_check_modules(EZW_CANOPEN_SERVICE_CLIENT REQUIRED ezw-canopen-service-client)
pkg_check_modules(EZW_COMMON REQUIRED ezw-common)
pkg_check_modules(EZW_TOOLS REQUIRED ezw-tools)

# Here are some values data generated by CMake if the
# pkg required is found
#   <XPREFIX>_FOUND          ... set to 1 if module(s) exist
#   <XPREFIX>_LIBRARIES      ... only the libraries (w/o the '-l')
#   <XPREFIX>_LIBRARY_DIRS   ... the paths of the libraries (w/o the '-L')
#   <XPREFIX>_LDFLAGS        ... all required linker flags
#   <XPREFIX>_LDFLAGS_OTHER  ... all other linker flags
#   <XPREFIX>_INCLUDE_DIRS   ... the '-I' preprocessor flags (w/o the '-I')
#   <XPREFIX>_CFLAGS         ... all required cflags
#   <XPREFIX>_CFLAGS_OTHER   ... the other compiler flags

# Add include dirs. The case not configured will be empty
message (STATUS "
Add inc directory:
EZW_TYPES_INCLUDE_DIRS : ${EZW_TYPES_INCLUDE_DIRS}
CAPI_INCLUDE_DIRS : ${CAPI_INCLUDE_DIRS}
EZW_SMC_SERVICE_CLIENT_INCLUDE_DIRS : ${EZW_SMC_SERVICE_CLIENT_INCLUDE_DIRS}
EZW_LOG_INCLUDE_DIRS : ${EZW_LOG_INCLUDE_DIRS}
EZW_IO_SERVICE_CLIENT_INCLUDE_DIRS : ${EZW_IO_SERVICE_CLIENT_INCLUDE_DIRS}
EZW_CANOPEN_SERVICE_CLIENT_INCLUDE_DIRS : ${EZW_CANOPEN_SERVICE_CLIENT_INCLUDE_DIRS}
EZW_COMMON_INCLUDE_DIRS : ${EZW_COMMON_INCLUDE_DIRS}
EZW_TOOLS_INCLUDE_DIRS : ${EZW_TOOLS_INCLUDE_DIRS}
-------------------" )

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EZW_TYPES_INCLUDE_DIRS}
    ${CAPI_INCLUDE_DIRS}
    ${EZW_SMC_SERVICE_CLIENT_INCLUDE_DIRS}
    ${EZW_LOG_INCLUDE_DIRS}
    ${EZW_IO_SERVICE_CLIENT_INCLUDE_DIRS}
    ${EZW_CANOPEN_SERVICE_CLIENT_INCLUDE_DIRS}
    ${EZW_COMMON_INCLUDE_DIRS}
    ${EZW_TOOLS_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# Add library dirs. The case not configured will be empty
message (STATUS "
Add library directory:
CAPI_LIBRARY_DIRS : ${CAPI_LIBRARY_DIRS}
EZW_SMC_SERVICE_CLIENT_LIBRARY_DIRS : ${EZW_SMC_SERVICE_CLIENT_LIBRARY_DIRS}
EZW_LOG_LIBRARY_DIRS : ${EZW_LOG_LIBRARY_DIRS}
EZW_IO_SERVICE_CLIENT_LIBRARY_DIRS : ${EZW_IO_SERVICE_CLIENT_LIBRARY_DIRS}
EZW_CANOPEN_SERVICE_CLIENT_LIBRARY_DIRS : ${EZW_CANOPEN_SERVICE_CLIENT_LIBRARY_DIRS}
EZW_COMMON_LIBRARY_DIRS : ${EZW_COMMON_LIBRARY_DIRS}
EZW_TOOLS_LIBRARY_DIRS : ${EZW_TOOLS_LIBRARY_DIRS}
------------------------------" )

link_directories(
    ${EZW_LOG_LIBRARY_DIRS}
    ${EZW_IO_SERVICE_CLIENT_LIBRARY_DIRS}
    ${CAPI_LIBRARY_DIRS}
    ${EZW_SMC_SERVICE_CLIENT_LIBRARY_DIRS}
    ${EZW_CANOPEN_SERVICE_CLIENT_LIBRARY_DIRS}
    ${EZW_COMMON_LIBRARY_DIRS}
    ${EZW_TOOLS_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

# Add required definitions from pkgconfig
message (STATUS "
Add definitions:
CAPI_CFLAGS : ${CAPI_CFLAGS}
CAPI_LDFLAGS : ${CAPI_LDFLAGS}
EZW_SMC_SERVICE_CLIENT_CFLAGS : ${EZW_SMC_SERVICE_CLIENT_CFLAGS}
EZW_SMC_SERVICE_CLIENT_LDFLAGS : ${EZW_SMC_SERVICE_CLIENT_LDFLAGS}
EZW_LOG_CFLAGS : ${EZW_LOG_CFLAGS}
EZW_LOG_LDFLAGS : ${EZW_LOG_LDFLAGS}
EZW_IO_SERVICE_CLIENT_CFLAGS : ${EZW_IO_SERVICE_CLIENT_CFLAGS}
EZW_IO_SERVICE_CLIENT_LDFLAGS : ${EZW_IO_SERVICE_CLIENT_LDFLAGS}
EZW_CANOPEN_SERVICE_CLIENT_CFLAGS : ${EZW_CANOPEN_SERVICE_CLIENT_CFLAGS}
EZW_CANOPEN_SERVICE_CLIENT_LDFLAGS : ${EZW_CANOPEN_SERVICE_CLIENT_LDFLAGS}
EZW_COMMON_CFLAGS : ${EZW_COMMON_CFLAGS}
EZW_COMMON_LDFLAGS : ${EZW_COMMON_LDFLAGS}
EZW_TOOLS_CFLAGS : ${EZW_TOOLS_CFLAGS}
EZW_TOOLS_LDFLAGS : ${EZW_TOOLS_LDFLAGS}
------------------------------" )

add_definitions(
    ${CAPI_CFLAGS}
    ${CAPI_LDFLAGS}
    ${EZW_SMC_SERVICE_CLIENT_CFLAGS}
    ${EZW_SMC_SERVICE_CLIENT_LDFLAGS}
    ${EZW_LOG_CFLAGS}
    ${EZW_LOG_LDFLAGS}
    ${EZW_IO_SERVICE_CLIENT_CFLAGS}
    ${EZW_IO_SERVICE_CLIENT_LDFLAGS}
    ${EZW_CANOPEN_SERVICE_CLIENT_CFLAGS}
    ${EZW_CANOPEN_SERVICE_CLIENT_LDFLAGS}
    ${EZW_COMMON_CFLAGS}
    ${EZW_COMMON_LDFLAGS}
    ${EZW_TOOLS_CFLAGS}
    ${EZW_TOOLS_LDFLAGS}
)

#------------------------------------------------------------------------------
# Sub-directories
#------------------------------------------------------------------------------

# Check options required for main build

# Main build
# add_subdirectory(src)

# Builds dependent on main build
# if(ENABLE_TESTS)
#     message(STATUS "TESTS enabled")
#     add_subdirectory(tests)
# else()
#     message(STATUS "TESTS disabled")
# endif(ENABLE_TESTS)

# if(ENABLE_DOCS)
#     message(STATUS "DOCS enabled")
#     add_subdirectory(docs)
# else()
#     message(STATUS "DOCS disabled")
# endif (ENABLE_DOCS)

#------------------------------------------------------------------------------
# Gencov custom command
#------------------------------------------------------------------------------

# set(COVERITY_FILES
#   " ${CMAKE_SOURCE_DIR}/build-coverity/output/summary.txt"
# )

# add_custom_command(OUTPUT ${COVERITY_FILES}
#     COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR} mkdir -p build-coverity
#     COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR} cov-configure --gcc
#     COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR} cov-build --dir build-coverity make -j $(nproc)
#     COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR} cov-analyze --dir build-coverity --all --aggressiveness-level low --misra-config MISRA_c2012_MM.config
#     COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR} cov-commit-defects --dir build-coverity --host coverity3.freescale.net --user committer --password CoverityGuest --stream lib${PROJECT_NAME}
#     COMMAND ${CMAKE_COMMAND} -E touch ${COVERITY_FILES}
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     COMMENT "run coverity"
# )
# add_custom_target(coverity DEPENDS ${COVERITY_FILES})


#------------------------------------------------------------------------------
# ROS generate messages
#------------------------------------------------------------------------------
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ezw_ros_controllers
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
# #   libezw
# #   libezw/CommonAPI
# #   libezw/CommonAPI/DBus
#   ${catkin_INCLUDE_DIRS}
# )

file(GLOB_RECURSE HEADERS include/*.hpp)
file(GLOB_RECURSE SOURCES_DRIVE_CTRL src/drive_controller/*.cpp)
file(GLOB_RECURSE SOURCES_DIFF_CRTL src/diff_drive_controller/*.cpp)
file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(drive_controller ${SOURCES_DRIVE_CTRL} ${COMMON_SOURCES} ${HEADERS})
add_dependencies(drive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(diff_drive_controller ${SOURCES_DIFF_CRTL} ${COMMON_SOURCES} ${HEADERS})
add_dependencies(diff_drive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(drive_controller
    ${catkin_LIBRARIES}
    ${EZW_SMC_SERVICE_CLIENT_LIBRARIES}
)

# Specify libraries to link a library or executable target against
target_link_libraries(diff_drive_controller
    ${catkin_LIBRARIES}
    ${EZW_SMC_SERVICE_CLIENT_LIBRARIES}
)

## Fake target to display files in QtCreator
FILE(GLOB_RECURSE OTHER_FILES
    "launch/*"
)

add_custom_target(other_files SOURCES ${OTHER_FILES})

#############
## Install ##
#############

install(TARGETS drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS diff_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

install(TARGETS drive_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS diff_drive_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ${CMAKE_CURRENT_SOURCE_DIR}/launch/drive_controller.launch
   ${CMAKE_CURRENT_SOURCE_DIR}/launch/diff_drive_controller.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)