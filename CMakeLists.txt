PROJECT(roboy_managing_node_fpga C CXX)

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

# Set CMake Policy to suppress the warning in CMake version 3.3.x
IF (POLICY CMP0043)
    CMAKE_POLICY(SET CMP0043 OLD)
ENDIF()

# we are cross compiling for arm (make sure to be in embedded_command_shell.sh)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)

set(compiler_path /usr/bin)
set(CMAKE_C_COMPILER ${compiler_path}/arm-linux-gnueabihf-gcc-5)
set(CMAKE_CXX_COMPILER ${compiler_path}/arm-linux-gnueabihf-g++-5)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/cmake/options.cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

OPTION(CFG_IP_STACK "Is an IP stack available (support of SDO/UDP)" OFF)
OPTION(CFG_INCLUDE_MN_REDUNDANCY "Use MN redundancy functions (if compiled into the libraries)" OFF)

#==============================================================================
# Dependencies
#==============================================================================

SET(OBJDICT CiA302-4_MN)        # Set object dictionary to use
FIND_OPLK_LIBRARY("mn")         # Find suitable openPOWERLINK library

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ros
find_package(catkin REQUIRED COMPONENTS roboy_communication_middleware roscpp std_msgs controller_manager common_utilities)
include(findros)
catkin_package( CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} )

# Set generic sources and include directories for this demo
SET(SOURCES
        src/main.cpp
        src/myoMaster.cpp
        ${COMMON_SOURCE_DIR}/obdcreate/obdcreate.c
        ${COMMON_SOURCE_DIR}/eventlog/eventlog.c
        ${COMMON_SOURCE_DIR}/eventlog/eventlogstring.c
        ${COMMON_SOURCE_DIR}/system/system-linux.c
        ${COMMON_SOURCE_DIR}/pcap/pcap-console.c
        ${CONTRIB_SOURCE_DIR}/console/printlog.c
        ${CONTRIB_SOURCE_DIR}/getopt/getopt.c
        ${CONTRIB_SOURCE_DIR}/console/console-linux.c
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN/obdpi.c
        )

INCLUDE_DIRECTORIES(
        include/
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/powerlink/output
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN
        ${OPLK_BASE_DIR}/stack/proj/linux/liboplkmndrv-pcap
)

include(findpcap)

ADD_DEFINITIONS(-DNMT_MAX_NODE_ID=254)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_PDO)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_SDO_ASND)

IF(CFG_IP_STACK)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_IP)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_SDO_UDP)
ENDIF(CFG_IP_STACK)

ADD_DEFINITIONS(-DCONFIG_INCLUDE_CFM)

IF(CFG_INCLUDE_MN_REDUNDANCY)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_NMT_RMN)
ENDIF(CFG_INCLUDE_MN_REDUNDANCY)

#ADD_DEFINITIONS(-DCONFIG_OBD_USE_STORE_RESTORE)
#ADD_DEFINITIONS(-DCONFIG_DLL_PRES_CHAINING_CN)
#ADD_DEFINITIONS(-DCONFIG_INCLUDE_MASND)

################################################################################
# Setup compile definitions depending on configuration

# PCAP is used for directlink or userspace daemon
IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    ADD_DEFINITIONS(-DCONFIG_USE_PCAP)
    SET (DEMO_SOURCES ${DEMO_SOURCES} ${COMMON_SOURCE_DIR}/pcap/pcap-console.c)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

IF (CFG_KERNEL_STACK_DIRECTLINK)
    UNSET (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    OPTION (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

IF (CFG_KERNEL_STACK_PCIE_INTF)
    ADD_DEFINITIONS(-D__LINUX_PCIE__)
ENDIF()

IF (CFG_KERNEL_STACK_ZYNQ_INTF)
    ADD_DEFINITIONS(-D__LINUX_ZYNQ__)
ENDIF()


################################################################################
# Setup the architecture specific definitions

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(linux.cmake)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    include (windows.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} is not supported!")
ENDIF()

#==============================================================================
# Application
#==============================================================================

ADD_EXECUTABLE(roboy_managing_node_fpga ${SOURCES} ${DEMO_ARCH_SOURCES} )
SET_PROPERTY(TARGET roboy_managing_node_fpga
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
OPLK_LINK_LIBRARIES(roboy_managing_node_fpga)
add_dependencies(roboy_managing_node_fpga roboy_communication_middleware_gencpp)
TARGET_LINK_LIBRARIES(roboy_managing_node_fpga ${pcap_LIBRARY} -lpthread -lrt ${ros_LIBRARIES})#

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        roboy_managing_node_fpga
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )