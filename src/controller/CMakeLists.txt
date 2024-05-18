# cmake_minimum_required(VERSION 3.1.6)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/bin)

# include_directories(${CARLA_LIB}/include ${PROJECT_ROOT_DIR}/include ${RPCLIB}/include)
# link_directories(${CARLA_LIB}/lib ${RPCLIB}/lib)

# add_library(planner planner.cpp)

# target_link_libraries(planner utils)
# target_include_directories(planner PUBLIC .)

include_directories(${CARLA_LIB}/include ${PROJECT_ROOT_DIR}/include ${RPCLIB}/include)
# link_directories(${CARLA_LIB}/lib ${RPCLIB}/lib)
# 编译成静态库, libsub.a
# add_library(controller ${CMAKE_CURRENT_SOURCE_DIR}/planner.cpp)
target_link_libraries(controller planner utils)