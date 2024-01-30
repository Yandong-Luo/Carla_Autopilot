cmake_minimum_required(VERSION 3.16)
project(Autopilot)

set(CMAKE_CXX_STANDARD 14)

set(CARLA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/carla_lib)

set(RPCLIB ${CMAKE_CURRENT_SOURCE_DIR}/rpclib)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME}
        main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CARLA_LIB}/include ${RPCLIB}/include ./include/ )
target_link_directories(${PROJECT_NAME} PRIVATE ${CARLA_LIB}/lib ${RPCLIB}/lib)
target_compile_options(${PROJECT_NAME} PRIVATE -isystem ${CARLA_LIB}/include/system)

target_link_libraries(${PROJECT_NAME} PRIVATE
        -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic
        -lpng
        -ltiff
        -ljpeg
        -lRecast
        -lDetour
        -lDetourCrowd -pthread)


#add_subdirectory("src/libcarla/cmake")
#add_subdirectory("src/example")
add_subdirectory("src/utils")
# add_subdirectory("src/router")
add_subdirectory("src/planner")
#add_subdirectory("src/controller")
        
