# Источники совместимости и интерфейсов

Проверено 5 октября 2026 года. Реальный набор установленных пакетов сохраняется
в `artifacts/dependencies-arm64.lock`, базовый образ — в `artifacts/base-image.txt`.

- [Матрица ROS / Gazebo и установка ros_gz](https://gazebosim.org/docs/fortress/ros_installation/): Humble + Fortress.
- [Платформы Humble REP-2000](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027): Ubuntu Jammy, ARM64.
- [ros_gz Humble и типы моста](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge): NavSatFix, PointCloud2, LaserScan, TFMessage.
- [NavSat Fortress](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/navsat/NavSat.cc): система NavSat, а не плагин Classic gazebo_ros_gps.
- [NavigateToPose Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action): пустой result; исход определяется action status, а не error_code новых версий.
- [Nav2 Humble params](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml): основа конфигурации, лицензия Apache-2.0. В `docs/nav2_humble_defaults.yaml` сохранён исходный набор параметров без комментариев.
- [DWB ObstacleFootprint Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp): проверка ориентированного footprint.
- [VoxelLayer Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/voxel_layer.cpp): трёхмерное очищение depth-слоя.
- [Выбор RMW в Humble](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst): Cyclone DDS через RMW_IMPLEMENTATION.

Электроника: ссылки на производителей приведены в `electronics.md`.
