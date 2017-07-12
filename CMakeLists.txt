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

################################################################################
# Setup project files and definitions

SET(OBJDICT CiA401_CN)              # Set object dictionary to use
FIND_OPLK_LIBRARY("cn")             # Find suitable openPOWERLINK library

# ros
find_package(catkin REQUIRED COMPONENTS common_utilities roboy_communication_middleware std_msgs)
include(findros)
include_directories(${catkin_INCLUDE_DIRS} )
catkin_package( CATKIN_DEPENDS message_runtime common_utilities roboy_communication_middleware)
include_directories(include/roboy_managing_node_fpga
                    usr/include/arm-linux-gnueabihf
        )

include(findpcap)

# Set generic sources and include directories for this demo
SET(SOURCE
        src/main.c
        src/app.c
        src/event.c
        ${CONTRIB_SOURCE_DIR}/console/printlog.c
        ${CONTRIB_SOURCE_DIR}/getopt/getopt.c
        ${COMMON_SOURCE_DIR}/eventlog/eventlog.c
        ${COMMON_SOURCE_DIR}/eventlog/eventlogstring.c
        ${COMMON_SOURCE_DIR}/obdcreate/obdcreate.c
        ${COMMON_SOURCE_DIR}/arp/arp.c
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN/obdpi.c
    )

SET(ALT_DEVICE_FAMILY soc_cv_av)
add_definitions(-D${ALT_DEVICE_FAMILY})
MESSAGE(STATUS "fpga include dir: $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}")
MESSAGE(STATUS "objectdic dir: ${OPLK_BASE_DIR}/apps/common/objdicts/${OBJDICT}")
INCLUDE_DIRECTORIES(
    include/
        $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}
        $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include
        $ENV{SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include/${ALT_DEVICE_FAMILY}
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/powerlink/output
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/include
        ${OPLK_BASE_DIR}/stack/proj/linux/liboplkmndrv-pcap
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/openPowerLink/apps/common/objdicts/CiA302-4_MN
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/openPowerLink/hardware/boards/de10-nano-soc/Nios_Access_DDR3/quartus
    )

# PResChaining is not supported by non-openMAC targets
#ADD_DEFINITIONS(-DCONFIG_DLL_PRES_CHAINING_CN)
ADD_DEFINITIONS(-DNMT_MAX_NODE_ID=0)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_PDO)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_SDO_ASND)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_MASND)

IF(CFG_IP_STACK)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_IP)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_SDO_UDP)
ENDIF(CFG_IP_STACK)

IF(CFG_STORE_RESTORE)
    ADD_DEFINITIONS(-DCONFIG_OBD_USE_STORE_RESTORE)
ENDIF(CFG_STORE_RESTORE)

################################################################################
# Setup compile definitions depending on configuration

# PCAP is used for directlink of userspace daemon
IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    ADD_DEFINITIONS(-DCONFIG_USE_PCAP)
    SET (SOURCE ${SOURCE} ${COMMON_SOURCE_DIR}/pcap/pcap-console.c)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

# Setup compile definitions depending on configuration
IF (CFG_KERNEL_STACK_DIRECTLINK)
    UNSET (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    OPTION (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)


################################################################################
# Setup the architecture specific definitions

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(linux.cmake)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    include (windows.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} is not supported!")
ENDIF()

################################################################################

################################################################################
# Set the executable

ADD_EXECUTABLE(roboy_managing_node_fpga ${SOURCE} ${DEMO_ARCH_SOURCES} )
SET_PROPERTY(TARGET roboy_managing_node_fpga
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
OPLK_LINK_LIBRARIES(roboy_managing_node_fpga)
add_dependencies(roboy_managing_node_fpga roboy_communication_middleware_gencpp)
TARGET_LINK_LIBRARIES(roboy_managing_node_fpga ${pcap_LIBRARY} ${pcap_udp_LIBRARY} -lpthread -lrt ${ros_LIBRARIES})#

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        roboy_managing_node_fpga
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )