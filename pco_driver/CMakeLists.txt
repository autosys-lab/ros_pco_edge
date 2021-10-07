cmake_minimum_required(VERSION 3.5)
project(pco_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DLINUX -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(pco_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(include)

SET(PCO_CLASSES
        src/pco_classes/Cpco_com_usb.cpp
        src/pco_classes/Cpco_grab_usb.cpp
        src/pco_classes/Cpcolog.cpp
        src/pco_classes/file12.cpp
        )

add_executable(camera_grab src/pco_camera_grab.cpp ${PCO_CLASSES})
target_include_directories(camera_grab PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_grab
        pthread
        usb-1.0
        )
ament_target_dependencies(camera_grab
        pco_common
        )

#add_executable(pco_driver src/pco_driver.cpp ${PCO_CLASSES})
#target_include_directories(pco_driver PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
#target_link_libraries(pco_driver
#        pthread
#        usb-1.0
#        )
#ament_target_dependencies(pco_driver
#        pco_common
#        rclcpp
#        image_transport
#        )

################################################

add_library(pco_camera_driver SHARED
        src/pco_driver.cpp
        ${PCO_CLASSES}
        )
target_compile_definitions(pco_camera_driver
        PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(pco_camera_driver
        pthread
        usb-1.0
        )
ament_target_dependencies(pco_camera_driver
        pco_common
        rclcpp
        rclcpp_components
        sensor_msgs
        camera_info_manager
        image_transport
        )

rclcpp_components_register_nodes(pco_camera_driver "PCODriver")
set(node_plugins "${node_plugins}PCODriver;$<TARGET_FILE:pco_camera_driver>\n")

add_executable(pco_camera_driver_node
        src/pco_driver_node.cpp
        )

target_link_libraries(pco_camera_driver_node
        pco_camera_driver
        )

ament_target_dependencies(pco_camera_driver_node
        rclcpp
        )

################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
        pco_camera_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS
        pco_camera_driver_node
        DESTINATION lib/${PROJECT_NAME}
        )

# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
