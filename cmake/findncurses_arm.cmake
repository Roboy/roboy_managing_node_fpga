include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ncurses_arm/include)
set(ncurses_LIBRARIES
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/libncurses.so.5.9
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/libncursesw.so.5.9
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/libtinfo.so.5.9
        )
message(STATUS "ncurses lib: ${ncurses_LIBRARIES}" )