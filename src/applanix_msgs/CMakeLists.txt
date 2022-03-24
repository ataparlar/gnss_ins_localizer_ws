cmake_minimum_required(VERSION 3.10.0)
project(applanix_msgs)

include("cmake/macros.cmake")

set(CMAKE_CXX_STANDARD 17)

set(SERVICE_FILES
  "srv/GetMgrsZone.srv"
  "srv/GetUtmZone.srv"
  "srv/SetOrigin.srv"
)

set(MESSAGE_FILES
  "msg/ChannelStatusGroup.msg"
  "msg/EventGroup.msg"
  "msg/GNSSStatusGroup3.msg"
  "msg/GpsTimeGsof.msg"
  "msg/LLA.msg"
  "msg/NavigationPerformanceGroup2.msg"
  "msg/NavigationPerformanceGsof50.msg"
  "msg/NavigationSolutionGroup1.msg"
  "msg/NavigationSolutionGsof49.msg"
  "msg/NED.msg"
  "msg/RawDmiDataGsof52.msg"
  "msg/StatusGsof.msg"
  "msg/TimeDistanceGroup.msg"
  "msg/TimeTaggedImuDataGroup4.msg"
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGE_FILES}
  ${SERVICE_FILES}
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME}_conversions INTERFACE)
target_include_directories(${PROJECT_NAME}_conversions INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(
  include
)

ament_export_interfaces(
  export_${PROJECT_NAME}
)

ament_package()
