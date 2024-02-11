cmake_minimum_required(VERSION 3.8)
project(servo_msgs)

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/DutyCycle.msg"
    "msg/MagneticSensorStatus.msg"
    "msg/As5600Angle.msg"
    DEPENDENCIES
        std_msgs
)

ament_package()
