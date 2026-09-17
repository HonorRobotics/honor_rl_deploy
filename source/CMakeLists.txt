cmake_minimum_required(VERSION 3.10)
project(motion_intelligence)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(Glog REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(interaction_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads REQUIRED)

# Include header directories
include_directories(include)
include_directories(interaction_msgs/install/interaction_msgs/include/interaction_msgs)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/cnpy)

# Set the ONNX Runtime path
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.22.0")
else()
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0")
endif()
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0")

# Include the ONNX Runtime headers
include_directories(${ONNXRUNTIME_DIR}/include)

# Collect source files
file(GLOB SOURCES 
    "src/common/*.cpp"
    "src/command/*.cpp"
    "src/controller/vita_boy/*.cpp"
    "src/state_machine/*.cpp"
    "src/estimator/*.cpp"
    "src/hardware/*.cpp"
    "thirdparty/cnpy/cnpy.cpp"
)

add_executable(${PROJECT_NAME} src/controller.cpp ${SOURCES}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs nav2_util cv_bridge)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    yaml-cpp
    interaction_msgs::interaction_msgs__rosidl_typesupport_cpp
    ${ONNXRUNTIME_LIB}
    glog
    ${OpenCV_LIBS}
    curl
    ZLIB::ZLIB
    dl
)

install(DIRECTORY config DESTINATION . USE_SOURCE_PERMISSIONS)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
install(DIRECTORY ${ONNXRUNTIME_DIR}/ DESTINATION thirdparty USE_SOURCE_PERMISSIONS)
install(DIRECTORY ../scripts DESTINATION . USE_SOURCE_PERMISSIONS)
