cmake_minimum_required(VERSION 3.20)
project(sh_dog_deploy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(MUJOCO QUIET mujoco)
if(NOT MUJOCO_FOUND)
  find_path(MUJOCO_INCLUDE_DIRS mujoco/mujoco.h HINTS /usr/include /usr/local/include)
  find_library(MUJOCO_LIBRARIES mujoco HINTS /usr/lib /usr/local/lib /lib)
  if(NOT MUJOCO_INCLUDE_DIRS OR NOT MUJOCO_LIBRARIES)
    message(FATAL_ERROR "MuJoCo headers/library not found")
  endif()
endif()
pkg_check_modules(GLFW3 QUIET glfw3)
pkg_check_modules(YAMLCPP QUIET yaml-cpp)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp REQUIRED)
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ENV ONNXRUNTIME_ROOT /usr/local
  PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS ENV ONNXRUNTIME_ROOT /usr/local PATH_SUFFIXES lib lib64)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime C++ headers/library not found")
endif()

add_library(sh_dog_policy_runtime
  deploy/runtime/src/policy_bundle.cpp
  deploy/runtime/src/policy_runtime.cpp)
target_include_directories(sh_dog_policy_runtime PUBLIC
  deploy/runtime/include ${ONNXRUNTIME_INCLUDE_DIR} ${YAMLCPP_INCLUDE_DIRS})
target_link_directories(sh_dog_policy_runtime PUBLIC ${YAMLCPP_LIBRARY_DIRS})
target_link_libraries(sh_dog_policy_runtime PUBLIC
  OpenSSL::Crypto Threads::Threads ${ONNXRUNTIME_LIBRARY})
if(YAMLCPP_FOUND)
  target_link_libraries(sh_dog_policy_runtime PUBLIC ${YAMLCPP_LIBRARIES})
else()
  target_link_libraries(sh_dog_policy_runtime PUBLIC yaml-cpp)
endif()
target_compile_options(sh_dog_policy_runtime PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sh_dog_sim2sim
  deploy/sim2sim/src/main.cpp
  deploy/sim2sim/src/mujoco_backend.cpp)
target_include_directories(sh_dog_sim2sim PRIVATE
  deploy/sim2sim/include ${MUJOCO_INCLUDE_DIRS})
target_link_directories(sh_dog_sim2sim PRIVATE ${MUJOCO_LIBRARY_DIRS})
target_link_libraries(sh_dog_sim2sim PRIVATE sh_dog_policy_runtime ${MUJOCO_LIBRARIES})
if(GLFW3_FOUND)
  target_compile_definitions(sh_dog_sim2sim PRIVATE SH_DOG_WITH_GLFW=1)
  target_include_directories(sh_dog_sim2sim PRIVATE ${GLFW3_INCLUDE_DIRS})
  target_link_directories(sh_dog_sim2sim PRIVATE ${GLFW3_LIBRARY_DIRS})
  target_link_libraries(sh_dog_sim2sim PRIVATE ${GLFW3_LIBRARIES})
endif()
target_compile_options(sh_dog_sim2sim PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME mujoco_model_loads
    COMMAND sh_dog_sim2sim --check-model
      ${CMAKE_SOURCE_DIR}/assets/sh_dog/mujoco/scene.xml)
endif()
