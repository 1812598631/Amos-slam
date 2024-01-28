cmake_minimum_required(VERSION 2.8)#cmake版本
project(orb_yolact)#项目名称

IF(NOT CMAKE_BUILD_TYPE)#设置cmake编译差不是故意
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# 添加 C++11 or C++0x 支持
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#链接OPENCV库，这里有两个版本选择 3.4.10会报错
#find_package(OpenCV 3.4.10 QUIET)
find_package(OpenCV 4.5.1 QUIET)
link_directories(${OpenCV_LIBRARY_DIRS})
#slam相关三方库
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#c++调用python库所需要的PythonLibs静态链接库
find_package(PythonLibs 3.7 REQUIRED)
#如果找到链接库的路径则终端输出Python Lib Found! Are:******
message(STATUS "Python Lib Found! Are: " ${PYTHON_LIBRARIES})
#设置c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#设置可执行文件的存放目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#设置头文件的存放目录
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/include)

#这里的参数要设置为自己电脑上安装yolact的路径，基本改一下用户名和虚拟环境名就可以了
#set(PYTHON_INCLUDE_DIRS "/home/run/anaconda3/envs/yolact-env/include/python3.7m/")
set(PYTHON_LIBRARIES "/home/run/anaconda3/envs/yolact-env/lib/libpython3.7m.so")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(
        ${PYTHON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include
)
#python头文件路径导入 修改如上
include_directories(BEFORE "/home/run/anaconda3/envs/yolact-env/lib/python3.7/site-packages/numpy/core/include/numpy")
include_directories(BEFORE "/home/run/anaconda3/envs/yolact-env/lib/python3.7/site-packages/numpy/core/include")
link_directories(/home/run/anaconda3/envs/yolact-env/lib/python3.7/site-packages/numpy/core/lib)


add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/yolact.cc
        src/cluster.cc)

target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PYTHON_LIBRARIES}
        ${Boost_LIBRARIES}
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#测试yolact接口
#add_executable(yolact_test src/c++test.cpp)
#target_link_libraries(yolact_test ${PROJECT_NAME})

#测试ORB_SLAM2 RGB-D TUM数据集
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})