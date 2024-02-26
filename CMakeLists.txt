# cmake_minimum_required(VERSION 3.16)
# project(Autopilot)

# set(CMAKE_CXX_STANDARD 14)

# set(CARLA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/carla_lib)

# set(RPCLIB ${CMAKE_CURRENT_SOURCE_DIR}/rpclib)

# set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# #add_subdirectory("src/libcarla/cmake")
# #add_subdirectory("src/example")
# add_subdirectory("src/utils")
# # add_subdirectory("src/router")
# add_subdirectory("src/planner")
# #add_subdirectory("src/controller")

# add_executable(${PROJECT_NAME}
#         main.cpp)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CARLA_LIB}/include ${RPCLIB}/include ./include/ )
# target_link_directories(${PROJECT_NAME} PRIVATE ${CARLA_LIB}/lib ${RPCLIB}/lib)
# target_compile_options(${PROJECT_NAME} PRIVATE -isystem ${CARLA_LIB}/include/system)

# target_link_libraries(${PROJECT_NAME}  PRIVATE
#         -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic
#         -lpng
#         -ltiff
#         -ljpeg
#         -lRecast
#         -lDetour
#         -lDetourCrowd -pthread
#         -utils, -planner)



        
cmake_minimum_required(VERSION 3.16)  # cmake版本最低要求
project(Autopilot)  # 设置工程名称

set(CMAKE_CXX_STANDARD 14)  # 指定 C++ 版本
set(CMAKE_BUILD_TYPE Debug)  # 调试使用Debug，可以查看中间变量；发布使用Release，运行速度快

message("${PROJECT_SOURCE_DIR}=" ${PROJECT_SOURCE_DIR})

set(CARLA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/carla_lib)

set(RPCLIB ${CMAKE_CURRENT_SOURCE_DIR}/rpclib)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# 这里设置好路径后，进入子模块的cmake时不用再次设置
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)  # 设置可执行文件的输出目录
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)	   # 设置库文件的输出目录

# 编译utils，生成静态库
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/utils)

# 编译planner，生产静态库
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/planner)

# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/add ADD_SRC_LIST)
# add_library(utils ${ADD_SRC_LIST})

# 编译sub，生成静态库
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sub SUB_SRC_LIST)
# add_library(planner ${SUB_SRC_LIST})

# 编译mul，生成动态库
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/source/mul MUL_SRC_LIST)
# add_library(mul SHARED ${MUL_SRC_LIST})

# 添加头文件路径，用于编译可执行文件
include_directories(./include)

# 编译main，生成可执行文件
add_executable(Autopilot ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

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
        -lDetourCrowd -pthread
        utils
        planner)