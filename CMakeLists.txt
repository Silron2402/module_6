cmake_minimum_required(VERSION 3.0.2)
project(module_6)
set(CMAKE_CXX_STANDARD 17)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

# указываем в проекте все дериктории с заголовочными файлами
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)


# добавляем библиотеки для сборки
# библиотека систем управления
add_library(control_sys_libs STATIC src/flight_controller.cpp)
add_library(transform STATIC src/transform.cpp)
add_library(tech_vis STATIC src/fiducial_navigation.cpp)
add_library(cam_params STATIC src/cam_reader.cpp)
add_library(aruco_desk STATIC src/aruco_matrix.cpp)

# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(circle_fly_example src/circle_fly_example.cpp)
add_executable(flight_controller_node src/flight_controller_node.cpp)
# пример с полетом БЛА(взаимодействие с аппаратом, управление целевым положением)
#add_executable(go_to_example examples/simple_flight_example.cpp)


# libraris link for visual navigation system
target_link_libraries(control_sys_libs  ${catkin_LIBRARIES})
target_link_libraries(circle_fly_example ${catkin_LIBRARIES})
target_link_libraries(flight_controller_node  ${catkin_LIBRARIES} control_sys_libs transform tech_vis cam_params aruco_desk)
#target_link_libraries(go_to_example  ${catkin_LIBRARIES})

