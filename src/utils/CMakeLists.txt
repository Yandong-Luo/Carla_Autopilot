cmake_minimum_required(VERSION 3.1.6)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/bin)

include_directories(${CARLA_LIB}/include ${PROJECT_ROOT_DIR}/include ${RPCLIB}/include)
link_directories(${CARLA_LIB}/lib ${RPCLIB}/lib)

add_library(utils common.cpp )

target_link_libraries(utils)