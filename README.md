# Автономный проезд А → Б в ROS 2 и Gazebo

Дифференциальный робот проходит GPS-waypoints в мире со статическими
препятствиями. Nav2 строит и исполняет маршрут, AMCL локализует робота по
лидару на известной карте. Лидар и depth-камера обновляют независимые слои
препятствий. Низкий объект не нанесён на карту и находится ниже плоскости
лидара: его обнаруживает depth-камера.

Проект создан для тестового задания ROS/Gazebo. Реальное оборудование,
его драйверы и полевые испытания в проект не входят.
[Репозиторий на GitHub](https://github.com/dasferwer/ROS-Gazebo-Robot).

## Стек

Проверенная среда: macOS 27.2 ARM64, Docker Desktop 29.7.2 с Linux aarch64,
Ubuntu 22.04.5 LTS в контейнере, Python 3.10, ROS 2 Humble, Ignition Gazebo
Fortress 6.18.0, Nav2 1.1.20, ros_gz 0.244.26. Используется Cyclone DDS.
Официальная совместимость и интерфейсы: [источники](docs/sources.md).
Базовый ROS-образ закреплён digest в Dockerfile. Фактические версии всех
Debian-пакетов сохранены в `docker/dependencies-arm64.lock`. Сборка сравнивает с ним весь набор пакетов
и завершается ошибкой при расхождении. Docker-конфигурация закреплена на
Linux ARM64; на x86_64 Docker использует эмуляцию.

Контейнер работает с программным OpenGL через Mesa/Xvfb. Gazebo и RViz
доступны в браузере через noVNC. Нативный Linux x86_64 предусмотрен пакетами
Humble, но отдельный прогон на x86_64 не выполнялся.

## Установка и запуск

На Mac нужен запущенный Docker Desktop с Compose v2. Linux-среда и зависимости
устанавливаются при сборке образа. Проверенный Docker VM имел 12 CPU и 8 ГБ RAM;
параллельные приложения на хосте влияют на скорость программного рендеринга.

Из корня репозитория:

```bash
./scripts/run.sh
```

Команда собирает образ, запускает новый контейнер, выполняет `colcon build`
и поднимает всю симуляцию с автоматическим маршрутом. Откройте
[Gazebo и RViz](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Порт привязан к localhost. Остановка — Ctrl+C в терминале запуска.

Отдельные команды:

```bash
docker compose build
docker compose run --rm robot bash -lc 'source /opt/ros/humble/setup.bash; colcon build --symlink-install'
docker compose run --rm --service-ports robot bash scripts/launch.sh
```

Без окон:

```bash
./scripts/run.sh gui:=false
```

Нативный Ubuntu 22.04 с установленным Humble:

```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rmw-cyclonedds-cpp python3-yaml python3-pytest
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source install/setup.bash
ros2 launch ab_robot demo.launch.py report_dir:="$PWD/artifacts/current"
```

При повторном запуске используют новый контейнер. Несколько симуляций с
одним ROS_DOMAIN_ID одновременно запускать не следует. Параметры launch:
`gui`, `autostart_route`, `route_file`, `report_dir`.

## Мир и модель

Старт А: (-4, -3), зелёная зона. Финиш Б: (4, 3), синяя зона. Стены,
центральный блок, два дополнительных блока и низкое препятствие заданы в
`config/scene.json`. `scripts/generate_scene.py` генерирует мир и карту
из одной геометрии. Пол плоский; цветные зоны имеют только visual, без collision.

Xacro содержит размеры, массу, тензоры инерции, collision корпуса, колёс и
опорных шаров. DiffDrive вращает два колеса; опорные шары имеют малое трение.
Параметры модели и сенсоров — `src/ab_robot/config/robot.yaml`.
Лидар: 360 лучей, 10 Гц, 8 м; depth: 160×120, 5 Гц, 5 м; GPS: 5 Гц.
Это параметры симуляции, не заявленные характеристики Astra или GPS-модуля.

## Узлы и данные

| Узел или система | Назначение |
|---|---|
| Fortress Physics, DiffDrive, Sensors, NavSat, Contact | Физика, колёса, сенсоры, контакты |
| ros_gz_bridge | Преобразование сообщений Gazebo ↔ ROS |
| robot_state_publisher | TF геометрии модели по URDF и joint_states |
| map_server, amcl | Известная карта и локализация по лидару |
| planner_server, controller_server, bt_navigator | NavFn, DWB, исполнение NavigateToPose |
| velocity_smoother | Ограничение ускорений и сглаживание скорости |
| gps_waypoints | GPS-привязка, последовательные цели, результаты и тайм-ауты |
| sensor_watchdog | Остановка при устаревших сенсорах, TF, команде, контакте или завершении |
| simulation_audit | Проверка данных, TF, физической позиции и контактов |

| Топик | Тип / потребитель |
|---|---|
| `/scan` | LaserScan → AMCL, lidar_layer обеих costmap |
| `/depth/points` | PointCloud2 → depth_layer обеих costmap, watchdog |
| `/depth` | Image → просмотр и запись |
| `/gps/fix` | NavSatFix → GPS-привязка маршрута, watchdog |
| `/odom` | Odometry → Nav2, velocity_smoother |
| `/cmd_vel` → `/cmd_vel_safe` | Twist → watchdog → мост → DiffDrive |
| `/clock` | Clock → все ROS-узлы с use_sim_time |
| `/tf`, `/tf_static` | TFMessage → согласованное дерево |
| `/plan`, `/local_plan` | Пути Nav2 → RViz |
| `/route_status` | JSON в String, transient local → статус и watchdog |
| `/ground_truth` | Истинная поза симулятора → только проверка результата |
| `/contacts/body`, `/contacts/left`, `/contacts/right` | Контакты → проверка и остановка |

GPS реализован системой NavSat Fortress и мостом в NavSatFix. Это эквивалент
`gazebo_ros_gps` из исходного задания для выбранного Ignition-стека. Плагины
Gazebo Classic не используются.

TF:

```text
map ─ AMCL → odom ─ DiffDrive → base_footprint
                                └─ robot_state_publisher → base_link
                                   ├─ lidar_link
                                   ├─ gps_link
                                   ├─ camera_link → camera_optical_frame
                                   └─ колёса
```

`base_footprint` лежит на земле, `base_link` на высоте 0.2 м. Каждый TF имеет
одного издателя. `camera_link`: x вперёд, y влево, z вверх. Живой PointCloud2
Fortress имеет именно эти оси; `camera_optical_frame` нужен для optical-представления
изображения (x вправо, y вниз, z вперёд). Нельзя назначать облаку optical frame
без преобразования его координат. `/ground_truth` не участвует в управлении,
AMCL или GPS-преобразовании.

## Карта и навигация

Статическая карта известного стенда упрощает воспроизводимость: отдельный
этап SLAM не нужен. AMCL получает начальную позу А и уточняет её по scan.
Карта 14×12 м с разрешением 0.05 м. Низкий объект намеренно не нанесён на карту.

Глобальный NavFn строит путь. DWB проверяет ориентированный footprint через
ObstacleFootprint. Лидар использует ObstacleLayer, depth — отдельный VoxelLayer:
иначе луч лидара над низким объектом может стереть его 2D-отметку. Итоговые
стоимости объединяются по максимуму и расширяются InflationLayer.

Обе costmap и collision checker recovery behaviors используют `map`;
локальная карта остаётся rolling window. Для этого статического стенда это
согласует текущую позу, путь и collision checker в одной frame.
Выбор сделан после диагностического GUI-прогона с устаревшим `map→odom` в
буфере controller; это обход наблюдавшегося сбоя, а не доказанное устранение
его причины в DDS. Подробности и повторные проверки — в [журнале](docs/ledger.md).

Параметры находятся в `config/nav2.yaml`: скорость до 0.35 м/с, поворот
до 0.8 рад/с, inflation radius 0.7 м, допуск Nav2 0.18 м и 0.2 рад.
Итоговый критерий проверки по физической позиции — ошибка не более 0.25 м.
Движение прекращается после завершения маршрута. Контакт защёлкивает остановку
до нового запуска; контакты колёс/опор с полом разрешены.
Watchdog закрывает привод при отсутствии или возрасте TF `map→base_link`
более 0.5 с; отдельно контролируется продвижение stamp по steady clock.
Параметр ROS `tf_timeout_s` задаёт этот предел. Исполнитель отменяет активную
цель и сохраняет failure, если локализация потеряна.
Для отмены при потере TF используется `tf_failure_timeout_s=2.0`: краткая
задержка допускает восстановление, пока watchdog уже удерживает нулевую скорость.
Кроме собственного TF-buffer, watchdog проверяет stamp и центр опубликованного
footprint local costmap. Во время миссии разница поз более 0.25 м или возраст
более 0.5 с сразу закрывает привод; устойчивый отказ 0.3 с защёлкивает остановку
и завершает миссию через `/safety_fault`. Параметры:
`controller_pose_timeout_s`, `controller_pose_tolerance_m`.

## GPS-waypoints

Опорная точка мира: WGS84 55.75° N, 37.61° E, высота 0, ENU, heading=0.
Оси мира и map: x на восток, y на север. Курс yaw отсчитывается против часовой
стрелки от востока. Изменение `heading_deg` требует согласовать ENU→map yaw.

`config/route.yaml` задаёт datum, известную позицию старта в map, допустимое
отклонение GPS и список точек:

```yaml
waypoints:
  - latitude: 55.74997664374203
    longitude: 37.61
    yaw: 0.0
```

Рабочие значения берутся из файла конфигурации; пример показывает формат.
Исполнитель ждёт active-состояние Nav2, TF и обязательные сенсоры. Пять живых
GPS-измерений неподвижного старта задают фактическую географическую привязку
к А; до этого выполняется проверка ожидаемой точки в пределах 0.5 м.
Географические цели переводятся в локальные east/north по радиусам кривизны
WGS84 в опорной широте, затем поворотом и переносом в map. Аппроксимация
ограничена радиусом 1 км, высота для плоской навигации не используется.

Этот выбор использует GPS для географической привязки, а не для высокоточной
локализации. Ориентация задана соглашением симулятора; `navsat_transform_node`
и выдуманный источник IMU не нужны. На реальном роботе потребуется отдельно
оценить yaw и трансформацию ENU→map; известная поза старта не переносится
автоматически на полевой сценарий.

Каждая цель отправляется через Humble NavigateToPose. У Humble пустой result:
успех проверяется по action status и расстоянию в map. Тайм-аут включает
ожидание принятия цели; отказ, abort, cancel, устаревшие сенсоры/TF и контакт
завершают маршрут ошибкой. Тайм-ауты и watchdog используют steady clock,
поэтому остановка симуляционного времени не создаёт бесконечного ожидания.

## Проверки и демонстрация

Фактические результаты и матрица критериев: [проверки](docs/verification.md).
Готовая демонстрация: `artifacts/final-video/demo.mp4`, rosbag —
`artifacts/final-video/rosbag/`.
[Скачать видео](https://github.com/dasferwer/ROS-Gazebo-Robot/releases/download/v0.1.0/demo.mp4)
можно из [релиза](https://github.com/dasferwer/ROS-Gazebo-Robot/releases/tag/v0.1.0).
Rosbag сохранён локально и передаётся отдельно.
Команды тестирования внутри контейнера:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ab_robot:$PYTHONPATH python3 -m pytest -q src/ab_robot/test
python3 scripts/validate_model.py
python3 scripts/graph_evidence.py artifacts/current/graph.json
python3 scripts/check_run.py artifacts/current
```

`check_run.py` требует живые сенсоры, их потребителей, единственного издателя
каждого TF, данные контактных сенсоров, отсутствие контактов/пересечения
геометрии, работу depth на низком объекте и ошибку Б ≤ 0.25 м.
SAT-проверка геометрии использует консервативный прямоугольник footprint;
её дополняют физические контактные сенсоры корпуса, опор и обоих колёс.

Для недостижимой цели внутри центрального блока:

```bash
ros2 launch ab_robot demo.launch.py gui:=false \
  route_file:="$(ros2 pkg prefix ab_robot)/share/ab_robot/config/unreachable.yaml" \
  report_dir:=/ws/artifacts/unreachable
```

Сценарий видео: показать зелёную А и синюю Б, карту и сенсоры в RViz;
дать роботу самостоятельно пройти две GPS-точки; показать объезд центрального
и низкого препятствий; завершить запись после достижения Б.

Для новой записи запустить симуляцию с `autostart_route:=false`. Во втором
терминале того же контейнера:

```bash
bash scripts/record.sh /ws/artifacts/new-demo
```

В третьем терминале:

```bash
ros2 run ab_robot waypoints --ros-args -p use_sim_time:=true \
  -p route_file:=/ws/install/ab_robot/share/ab_robot/config/route.yaml \
  -p report_file:=/ws/artifacts/new-demo/route.json
```

После завершения остановить запись Ctrl+C: MP4 и rosbag закрываются корректно.
Запуск новой миссии после ошибки/контакта выполняется в новом контейнере.
Для воспроизведения bag не запускать живую симуляцию:

```bash
ros2 bag info artifacts/final-video/rosbag
ros2 bag play artifacts/final-video/rosbag --clock --topics \
  /tf /tf_static /odom /scan /depth /depth/points /gps/fix \
  /plan /ground_truth /global_costmap/costmap /route_status
rviz2 -d src/ab_robot/config/demo.rviz --ros-args -p use_sim_time:=true
```

Для RobotModel запустить publisher только описания модели; его TF перенаправлены
в другие топики, потому что исходный TF уже проигрывается из bag:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro src/ab_robot/urdf/robot.urdf.xacro)" \
  -r /tf:=/replay_unused_tf -r /tf_static:=/replay_unused_tf_static
```

Время воспроизведения задаёт `--clock`; список `--topics` исключает записанный `/clock` и управляющие
команды, поэтому время имеет одного издателя.

## Ограничения и сдача

Программный OpenGL медленнее аппаратного; видео показывает скорость
конкретного стенда. Используются идеальный GPS и известный начальный yaw.
Полевой GPS, шум, рельеф, динамические препятствия и аппаратные датчики
не проверялись. APT-репозитории меняются: сборка проверяет полный manifest и не принимает
другие версии молча. Если lock-проверка не проходит, используйте сохранённый
проверенный образ или отдельно обновите lock и повторите интеграционные проверки.
Для переноса точного образа:

```bash
docker save ab-robot:humble | gzip > artifacts/ab-robot-humble-arm64.tar.gz
# На другом компьютере:
gunzip -c artifacts/ab-robot-humble-arm64.tar.gz | docker load
```

`artifacts/` не коммитится. Видео приложено к релизу на GitHub,
а rosbag сохранён в локальной папке проекта.
Оценку работодателя проект не предсказывает.

Бонусные вопросы: [подключение, питание и проверки без ROS](docs/electronics.md).
