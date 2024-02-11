cmake_minimum_required(VERSION 3.8)
project(servo)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(servo_msgs REQUIRED)
find_package(device_kit REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(blaze REQUIRED)

pkg_check_modules(gpiodcxx REQUIRED IMPORTED_TARGET libgpiodcxx)

# Position regulator executable
add_library(${PROJECT_NAME} SHARED
    src/PositionRegulatorAds1115.cpp
    src/PositionRegulatorAs5600.cpp
    src/Drive.cpp
)
target_link_libraries(${PROJECT_NAME}
    device_kit::device_kit
    PkgConfig::gpiodcxx
    blaze::blaze
)
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "angles"
    "servo_msgs"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
    "servo::PositionRegulatorAds1115"
    "servo::PositionRegulatorAs5600"
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
