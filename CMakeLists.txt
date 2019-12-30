cmake_minimum_required(VERSION 3.2)
project(edgetpu_roscpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  abseil_cpp
  roscpp
  rospy
  cv_bridge
  image_transport
  jsk_perception
  jsk_recognition_msgs
  jsk_topic_tools
  nodelet
  sensor_msgs
  )

# Tensorflow Lite
set(TENSORFLOW_COMMIT 5d0b55dd4a00c74809e5b32217070a26ac6ef823)
set(TENSORFLOW_DIR ${CMAKE_CURRENT_BINARY_DIR}/tensorflow)
if (NOT EXISTS ${TENSORFLOW_DIR})
  execute_process(
    COMMAND git clone https://github.com/tensorflow/tensorflow.git ${TENSORFLOW_DIR})
  execute_process(
    COMMAND git checkout ${TENSORFLOW_COMMIT}
    WORKING_DIRECTORY ${TENSORFLOW_DIR}
    )
  execute_process(
    COMMAND bash "download_dependencies.sh"
    WORKING_DIRECTORY ${TENSORFLOW_DIR}/tensorflow/lite/tools/make
    )
  execute_process(
    COMMAND bash "build_lib.sh"
    WORKING_DIRECTORY ${TENSORFLOW_DIR}/tensorflow/lite/tools/make
    )
endif()
include_directories(${TENSORFLOW_DIR})
include_directories(${TENSORFLOW_DIR}/tensorflow/lite/tools/make/downloads/flatbuffers/include)
link_directories(${TENSORFLOW_DIR}/tensorflow/lite/tools/make/gen/linux_x86_64/lib)


# EdgeTPU Native C++ library
set(EDGETPU_DIR ${CMAKE_CURRENT_BINARY_DIR}/edgetpu)
set(EDGETPU_COMMIT 25cad8d9c91753d9edd9f23b86f487749d5a84cc)
if (NOT EXISTS ${EDGETPU_DIR})
  execute_process(
    COMMAND git clone https://github.com/tongtybj/edgetpu.git ${EDGETPU_DIR})
  execute_process(
    COMMAND git checkout ${EDGETPU_COMMIT}
    WORKING_DIRECTORY ${EDGETPU_DIR}
    )
endif()
include_directories(${EDGETPU_DIR})
include_directories(${EDGETPU_DIR}/libedgetpu)
link_directories(${EDGETPU_DIR}/libedgetpu/direct/k8) # right now only support x86 arch

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES edgetpu_ros
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(EDGETPU_SOURCE_DIR ${EDGETPU_DIR}/src/cpp)

# download and install sample data
add_custom_target(install_test_data ALL
  COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)
add_custom_target(link_test_data ALL
  COMMAND ln -nfs ${EDGETPU_DIR}/test_data ${PROJECT_SOURCE_DIR}/test/data/edgetpu
  DEPENDS install_test_data
  )
add_custom_target(link_scripts ALL
  COMMAND ln -nfs ${EDGETPU_DIR}/scripts ${PROJECT_SOURCE_DIR}/scripts/edgetpu
  )


# test of src/cpp/examples/minimal
add_executable(edgetpu_minimal
  ${EDGETPU_SOURCE_DIR}/examples/minimal.cc
  ${EDGETPU_SOURCE_DIR}/examples/model_utils.cc)
target_link_libraries(edgetpu_minimal tensorflow-lite :libedgetpu.so.1.0 pthread)

# test of src/cpp/examples/classify_image with OpenCV
add_library(edgetpu_engine
  ${EDGETPU_SOURCE_DIR}/error_reporter.cc
  ${EDGETPU_SOURCE_DIR}/basic/basic_engine.cc
  ${EDGETPU_SOURCE_DIR}/basic/basic_engine_native.cc
  ${EDGETPU_SOURCE_DIR}/basic/edgetpu_resource_manager.cc
  ${EDGETPU_SOURCE_DIR}/classification/engine.cc
  ${EDGETPU_SOURCE_DIR}/detection/engine.cc
  ${EDGETPU_SOURCE_DIR}/posenet/posenet_decoder.cc
  ${EDGETPU_SOURCE_DIR}/posenet/posenet_decoder_op.cc
  ${EDGETPU_SOURCE_DIR}/examples/label_utils.cc
  src/utils/image_resize.cpp)
target_link_libraries(edgetpu_engine tensorflow-lite :libedgetpu.so.1.0 pthread ${catkin_LIBRARIES})

add_executable(classify_image src/classify_image.cpp)
target_link_libraries(classify_image edgetpu_engine glog ${catkin_LIBRARIES})

add_executable(object_detection src/object_detection.cpp)
target_link_libraries(object_detection edgetpu_engine glog  ${catkin_LIBRARIES})

macro(arp_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

arp_add_nodelet(src/deep_object_detection.cpp "edgetpu_rospp/DeepObjectDetecion" "deep_object_detection")
arp_add_nodelet(src/single_object_tracking_by_deep_detection.cpp "edgetpu_rospp/SingleObjectDeepTrackingDetection" "single_object_tracking_by_deep_detection")

add_library(${PROJECT_NAME} SHARED ${edgetpu_roscpp_nodelet_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} edgetpu_engine glog)

