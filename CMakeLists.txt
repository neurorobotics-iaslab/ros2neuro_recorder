cmake_minimum_required(VERSION 3.8)
project(ros2neuro_recorder)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2neuro_msgs REQUIRED)
find_package(ros2neuro_data REQUIRED)
find_package(rclcpp REQUIRED)
find_library(XDFIO_LIBRARIES xdffileio)
find_path(XDFIO_INCLUDE_DIRS xdfio.h)

# ---- add the library for recorder
add_library(${PROJECT_NAME}
  src/Writer.cpp
  src/XDFWriter.cpp
  src/DummyWriter.cpp
  src/FactoryWriter.cpp
  src/Recorder.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${XDFIO_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${XDFIO_LIBRARIES}
)
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "ros2neuro_data"
  "ros2neuro_msgs"
)
ament_export_libraries(${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- add cpp file for recorder node
add_executable(recorder src/recorder.cpp)
target_include_directories(recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(recorder
  ${PROJECT_NAME})
target_compile_features(recorder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  recorder
  "ros2neuro_msgs"
  "ros2neuro_data"
  "rclcpp"
)
install(TARGETS recorder
    DESTINATION lib/${PROJECT_NAME})

# ---- cpp file for testing 
add_executable(test_writerfactory src/test_writerfactory.cpp)
target_include_directories(test_writerfactory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_writerfactory
  ${PROJECT_NAME})
target_compile_features(test_writerfactory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_writerfactory
  "ros2neuro_msgs"
  "ros2neuro_data"
  "rclcpp"
)
install(TARGETS test_writerfactory
  DESTINATION lib/${PROJECT_NAME})



# --- final installations/exporting
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

# ---- to remove unused-parameters warnings
if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-parameter) 
  target_compile_options(test_writerfactory PRIVATE -Wno-unused-parameter) 
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-compare) 
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-variable) 
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-but-set-variable) 
endif()


ament_package()



