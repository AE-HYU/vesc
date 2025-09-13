cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

# C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)

# ★ Eigen3 찾기
# eigen3_cmake_module 사용 중이건, 시스템 모듈이건 둘 다 대응
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom.cpp
)

# ★ Eigen include 경로를 명시적으로 추가 (환경에 따라 필요)
#   - 우분투 libeigen3-dev 기본 경로는 /usr/include/eigen3 이고,
#     이게 들어가야 #include <Eigen/Dense> 가 동작합니다.
#   - EIGEN3_INCLUDE_DIRS 변수가 없는 환경을 대비한 fallback도 포함.
if(NOT EIGEN3_INCLUDE_DIRS)
  # 일부 환경에서는 대소문자 다른 변수로 노출되기도 합니다.
  if(DEFINED Eigen3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
  endif()
endif()

message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}        # ★ 요 줄이 핵심
)

# ament가 링크/전파 처리
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
)

# register nodes as components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::AckermannToVesc
  EXECUTABLE ackermann_to_vesc_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::VescToOdom
  EXECUTABLE vesc_to_odom_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
)
