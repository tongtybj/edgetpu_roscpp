cmake_minimum_required(VERSION 3.2)
project(edgetpu_roscpp)

add_compile_options(-std=c++11)

include(ExternalProject)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  )

#find_package(PkgConfig)
#pkg_check_modules(GLOG QUIET libglog)

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


# Abseil
set(ABSEIL_INSTALL_DIR ${PROJECT_SOURCE_DIR}/absl)
ExternalProject_Add(ABSEIL
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG 43ef2148c0936ebf7cb4be6b19927a9d9d145b8f
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${ABSEIL_INSTALL_DIR}
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${ABSEIL_INSTALL_DIR}/include)
#include(${ABSEIL_INSTALL_DIR}/lib/cmake/absl/abslConfig.cmake) # we can not load .cmake when this target does not exist.
link_directories(${ABSEIL_INSTALL_DIR}/lib)
set(ABSEIL_LIBRARIES
  absl_synchronization
  absl_malloc_internal
  absl_graphcycles_internal
  absl_stacktrace
  absl_symbolize
  absl_debugging_internal
  absl_dynamic_annotations
  absl_demangle_internal
  absl_spinlock_wait
  absl_time
  absl_civil_time
  absl_time_zone
  absl_base
  )


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
  src/utils/image_resize.cpp)
target_link_libraries(edgetpu_engine tensorflow-lite :libedgetpu.so.1.0 pthread ${catkin_LIBRARIES})
add_dependencies(edgetpu_engine ABSEIL)

add_executable(classify_image
  src/classify_image.cpp
  ${EDGETPU_SOURCE_DIR}/examples/label_utils.cc
  )
target_link_libraries(classify_image edgetpu_engine glog ${ABSEIL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(classify_image ABSEIL)

add_executable(object_detection
  src/object_detection.cpp
  ${EDGETPU_SOURCE_DIR}/examples/label_utils.cc
  )
target_link_libraries(object_detection edgetpu_engine glog ${ABSEIL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(object_detection ABSEIL)
