# Фактические проверки

Проверено 5–6 октября 2026 года. Mac ARM64 → Docker Linux ARM64 →
Ubuntu 22.04.5, ROS 2 Humble, Fortress 6.18.0, Nav2 1.1.20,
ros_gz 0.244.26, Cyclone DDS. Версии: `docker/dependencies-arm64.lock`.
Базовый образ закреплён digest; сборка сравнивает весь установленный
Debian manifest с lock. Финальная сборка образа прошла.

## Сборка и тесты

Чистая сборка в новых каталогах:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base artifacts/release-log build \
  --build-base artifacts/release-build \
  --install-base artifacts/release-install --symlink-install
PYTHONPATH=src/ab_robot:$PYTHONPATH python3 -m pytest -q src/ab_robot/test
python3 scripts/validate_model.py
```

- Один пакет собран без ошибок и предупреждений, 0.89 с: [build](evidence/build.txt).
- **28 тестов пройдены, без пропусков**: [tests](evidence/tests.txt).
- Проверены WGS84, ENU→map, границы/NaN, последовательность целей,
  отказ/тайм-аут, sensor/command watchdog и различение пола/препятствий.
- Реальные ROS-узлы создаются. Startup timeout проверен при
  `use_sim_time=true` без `/clock`. Проверены отмена поздно принятой цели,
  сохранение failure после закрытия контекста, краткая и длительная потеря TF.
- Watchdog отвергает старый TF и расхождение позы costmap при свежем собственном
  TF. Ошибка остаётся защёлкнутой после нового статуса running.
- Проверки доказательств отвергают пустой/неполный TF, второго родителя,
  второго издателя TF и второго издателя времени.
- Xacro реально конвертирован в SDF; collision references контактных сенсоров
  совпадают: [model](evidence/model.txt).

Модульные тесты не заменяют проезд в симуляции.

## Два финальных проезда

Оба прогона относятся к окончательной конфигурации: шаг физики 0.01 с,
100 Гц clock/joint_states/контактов, независимые lidar/depth layers,
local costmap и recovery collision checker в map, проверки TF и позы costmap.
Каждый запуск выполнялся в новом контейнере.

GUI и запись:

```bash
./scripts/run.sh autostart_route:=false report_dir:=/ws/artifacts/release-run
# В том же контейнере после готовности GUI:
bash scripts/record.sh /ws/artifacts/final-video
ros2 run ab_robot waypoints --ros-args -p use_sim_time:=true \
  -p route_file:=/ws/install/ab_robot/share/ab_robot/config/route.yaml \
  -p report_file:=/ws/artifacts/release-run/route.json
```

Повтор с автоматическим стартом:

```bash
./scripts/run.sh gui:=false report_dir:=/ws/artifacts/release-repeat
```

| Измерение | GUI + видео | Повтор |
|---|---:|---:|
| Маршрут | SUCCEEDED, обе GPS-цели | SUCCEEDED, обе GPS-цели |
| Wall time исполнителя | 41.908 с | 47.303 с |
| Ошибка физической позиции в Б | 0.171887 м | 0.099526 м |
| Допуск | 0.25 м | 0.25 м |
| Контакты с препятствиями | 0 | 0 |
| Пересечения footprint | 0 | 0 |
| Минимальное SAT-разделение | 0.442004 м | 0.460165 м |
| GPS, lidar, depth | живые сообщения и потребители | живые сообщения и потребители |
| TF | 11 рёбер, по одному GID | 11 рёбер, по одному GID |
| Clock | один издатель, без регрессий | один издатель, без регрессий |

SAT-разделение — зазор проекций по разделяющим осям, а не точное евклидово
расстояние. Результат независимо проверяют ground-truth поза и реальные
контактные сенсоры корпуса/опор и обоих колёс. Контакты с полом допускаются.

Доказательства: [GUI audit](evidence/release-gui/audit.json),
[GUI route](evidence/release-gui/route.json),
[GUI checks](evidence/release-gui/checks.json),
[GUI graph](evidence/release-gui/graph.json);
[повтор audit](evidence/release-repeat/audit.json),
[повтор route](evidence/release-repeat/route.json),
[повтор checks](evidence/release-repeat/checks.json),
[повтор graph](evidence/release-repeat/graph.json).
**Все 12 критериев check_run прошли в обоих запусках.**

GID сообщений TF получает C++-сборщик через публичный rclcpp MessageInfo.
Python rclpy Humble не предоставляет publisher GID в subscription callback.
Сборщик вызывается отдельно и исключён из основной сборки COLCON_IGNORE.

## Роль сенсоров

LaserScan получают AMCL и lidar_layer обеих costmap. PointCloud2 получают
local_costmap и global_costmap. Низкий объект отсутствует на статической карте
и находится ниже лидара; его клетка отмечалась lethal после наблюдения depth
в обоих прогонах. Число обновлений сохранено в audit.json. Лидар не очищает
независимый depth voxel layer.

Пять живых GPS-измерений старта задают географическую привязку к А.
Координаты двух целей вычислены из широты/долготы и сохранены в route.json.
GPS не подменён непосредственно заданными локальными целями.

## Отказы и недостижимая цель

```bash
./scripts/run.sh gui:=false \
  route_file:=/ws/install/ab_robot/share/ab_robot/config/unreachable.yaml \
  report_dir:=/ws/artifacts/release-unreachable
```

Цель внутри центрального блока: action status **6 / ABORTED**, failure за
**21.404 с**, при goal timeout 35 с. Последняя команда привода
нулевая; контактов и пересечений нет.
[Route](evidence/release-unreachable/route.json),
[audit](evidence/release-unreachable/audit.json),
[checks](evidence/release-unreachable/checks.json).

Диагностический прогон при 500 Гц воспроизвёл старую позу local costmap,
несмотря на свежие TF у другого подписчика. Новая защита завершила миссию
за **3.739 с**, удержав нулевую скорость до движения/контакта:
[route](evidence/controller-stale/route.json),
[audit](evidence/controller-stale/audit.json),
[trace](evidence/controller-stale/trace.json),
[checks](evidence/controller-stale/checks.json).
Это проверка конкретного отказа, а не утверждение о покрытии всех отказов.

## Видео и rosbag

- `artifacts/final-video/demo.mp4`: **83.334 с**, 1600×900, 15 fps, H.264.
  Непрерывная реальная запись Linux-дисплея, без изменения скорости.
  Gazebo и RViz видны вместе. Осмотрены кадры 2, 28 и 60 с:
  А, объезд центрального/низкого объектов и Б. Полное декодирование ffmpeg
  завершилось без ошибок.
- `artifacts/final-video/rosbag/`: SQLite3, **237.4 MiB**,
  **46 652 сообщения**, **82.990 с симуляционного времени**.
  Записаны сенсоры, TF/clock, plan/local_plan, команды, costmap/footprints,
  статус, ground truth и контакты.
- Проверены сообщения записи: Image — `camera_optical_frame`,
  PointCloud2 — `camera_link`, как требует используемая система Fortress.
- Replay окончательной записи выполнен в отдельном ROS_DOMAIN_ID=85:
  `/clock` публиковал только rosbag2_player, все 11 рёбер TF имели по одному
  GID. RSP публиковал описание, его TF перенаправлены в другие топики.

[Метаданные и проверки видео/bag](evidence/video.txt),
[Replay graph](evidence/replay-final-graph.json). Команды — в README.

## Соответствие заданию

Это факты для оценки, а не обещание баллов работодателя.

| Критерий | Подтверждение |
|---|---|
| Модель и сенсоры, 2 балла | Xacro→SDF, инерция/collisions, привод, живые scan/depth/NavSatFix |
| Мир, 1 балл | Плоский пол, статические препятствия, зелёная А и синяя Б в видео |
| ROS/топики/TF/launch, 2 балла | Реальные endpoints, 11 TF без дубликатов, clock, demo.launch.py |
| Навигация, 3 балла | Два SUCCEEDED, ошибка меньше 0.25 м, контактов и пересечений нет |
| GPS/waypoints, 1 балл | Живой GPS anchor, преобразованные географические цели |
| Код/документация, 1 балл | Python-исполнитель/watchdog, 28 тестов, README и electronics.md |
| Воспроизводимость, 1 балл | Новый контейнер, чистая сборка, lock, фактический повтор scripts/run.sh |

## Ограничения и известные проблемы

Проверена Docker ARM64-симуляция; физический робот и нативный x86_64
не проверялись. GPS идеальный, начальная поза и ENU yaw известны.
Рельеф, динамические препятствия и все сочетания отказов не покрыты.

При прежних потоках 500 Гц наблюдалось устаревание внутренних поз Nav2,
в том числе контакт до добавления независимой защиты. Замена DDS на UDP
сама по себе сбой не устранила. Причина задержки конкретного внутреннего
TF-buffer не установлена; утверждать устранение библиотечного дефекта нельзя.
Окончательные потоки снижены до 100 Гц и добавлен независимый контроль
позы costmap. Два финальных прогона успешны, но не доказывают надёжность
при произвольной нагрузке. Параллельная тяжёлая C++-сборка во время записи
дала временную задержку TF; сборку выполняют до запуска демонстрации.

Mesa/RViz выдавал GLSL-сообщение об active samplers; карта и облако реально
отображались. При Ctrl+C некоторые процессы Nav2 требовали эскалации
SIGTERM/SIGKILL средствами launch; остановка контейнера могла занимать
около 15 секунд. MP4 и bag закрыты и проверены отдельно.

История неудачных проб и исправлений: [ledger](ledger.md).
Независимое ревью: [review](review.md). Старые run-01/run-02 и диагностические
записи сохранены отдельно и не подменяют финальные результаты.

Публикация и отправка работодателю не выполнялись. Перед сдачей осталось
опубликовать репозиторий и разместить MP4 по отдельной команде пользователя.
Видео/bag находятся вне Git; короткие доказательства включены в репозиторий.
