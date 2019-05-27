# 声明CMake的最低要求版本
cmake_minimum_required(VERSION 2.8)

# 定义项目(工程)名称,同时定义了以下默认变量
# PROJECT_SOURCE_DIR 或 HELLO_SOURCE_DIR, 代指CMake开始构建的根目录(通常是项目根目录)
# PROJECT_BINARY_DIR 或 HELLO_BINARY_DIR, 代指CMake的编译目录(即执行cmake命令的目录)
PROJECT(FlightSim)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  

# opencv
find_package(OpenCV  3.3.1 EXACT REQUIRED
            PATHS /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
# 添加lib的查找目录
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED COMPONENTS)
# SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} Eigen3::Eigen)
include_directories(${EIGEN3_INCLUDE_DIR})

# set refer to mrsid lib 
SET(MRSID_LIB_DIR /li/lib/)
SET(MRSID_LIBS ${MRSID_LIB_DIR}libltidsdk.so  ${MRSID_LIB_DIR}libltidsdk.so.9 
     ${MRSID_LIB_DIR}libtbb.so  ${MRSID_LIB_DIR}libtbb.so.2)
SET(MRSID_INCLUDE_DIR /li/include/mrsid_raster/)
SET(MRSID_BIN_DIR /li/bin)

SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${MRSID_LIBS}) 
include_directories(${MRSID_INCLUDE_DIR})
# message("add mrsid include directory: ${MRSID_INCLUDE_DIR}")

# shplib
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} /usr/local/lib/libshp.so)
include_directories(/usr/local/include/  )

ADD_SUBDIRECTORY(src)
