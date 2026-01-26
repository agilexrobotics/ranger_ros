cmake_minimum_required(VERSION 3.0.2)
project(agx_bringup)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  tf
  message_generation
  std_msgs
  nav_msgs
)

add_message_files(
  FILES
  MotorLow.msg
  MotorHigh.msg
  MotorLowArray.msg
  MotorHighArray.msg
  FaultBit.msg
  SystemStatus.msg
  SystemStatusArray.msg
  ChassisMotion.msg
  LightControlStatus.msg
  RemoteControlStatus.msg
  SteeringAngles.msg
  WheelSpeeds.msg
  MotionModeFeedback.msg
  FrontWheelOdometry.msg
  BackWheelOdometry.msg
  BMSStatus.msg
  BMSAlarmWarning.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  node/agx_bringup_node.cpp
  src/bring_up.cpp
  src/can/chassis_status.cpp
  src/can/chassis_service.cpp
  src/can/socket_can.cpp
  src/can/frame/feedback/base_frame.cpp
  src/can/frame/feedback/frame_211.cpp
  src/can/frame/feedback/frame_221.cpp
  src/can/frame/feedback/frame_231.cpp
  src/can/frame/feedback/frame_241.cpp
  src/can/frame/feedback/frame_25x.cpp
  src/can/frame/feedback/frame_26x.cpp
  src/can/frame/feedback/frame_271.cpp
  src/can/frame/feedback/frame_281.cpp
  src/can/frame/feedback/frame_291.cpp
  src/can/frame/feedback/frame_311.cpp
  src/can/frame/feedback/frame_312.cpp
  src/can/frame/feedback/frame_361.cpp
  src/can/frame/feedback/frame_362.cpp
  src/can/frame/feedback/frame_371.cpp
  src/can/frame/handler/base_handler.cpp
  src/can/frame/handler/can_handler.cpp
  src/can/frame/handler/handler_111.cpp
  src/can/frame/handler/handler_121.cpp
  src/can/frame/handler/handler_141.cpp
  src/can/frame/handler/handler_423.cpp
  src/can/frame/handler/handler_441.cpp
  src/can/frame/handler/handler_461.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

