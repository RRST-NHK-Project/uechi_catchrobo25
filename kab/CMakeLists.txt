cmake_minimum_required(VERSION 3.8)
project(kab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/subscribe_lambda_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscribe_lambda_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)







ament_package()


