include_directories(${CMAKE_CURRENT_SOURCE_DIR}/usr/include)
set(pcap_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libpcap.so)
message(STATUS "pcap lib: ${pcap_LIBRARY}" )
set(rt_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/librt.so.1
        )
set(pthread_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libpthread_nonshared.a
        )