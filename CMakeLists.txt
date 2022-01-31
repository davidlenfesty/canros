cmake_minimum_required(VERSION 3.8)
project(canros)

find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

execute_process(COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate.py)

file(GLOB MESSAGES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB SERVICES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGES}
    ${SERVICES}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
