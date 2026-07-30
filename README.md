# sverk_gz_overrides

Gazebo (Harmonic) модели дронов Obrik, сенсоров и SITL-мир с ArUco-картой,
используемые для симуляции стека [sverk-ros2](https://github.com/sverk-tech/sverk-ros2)
поверх стандартного PX4 SITL.

Репозиторий не самостоятелен — это набор override/дополнительных моделей PX4
`Tools/simulation/gz`, подключаемых через bind-mount или git submodule в
проект, который собирает SITL-образ (`sverk-ros2/scripts/sitl/`).

## Структура

```
sverk_gz_overrides/
├── scripts/
│   └── update_overrides.sh          # установка моделей/миров/ROS-нод по местам (см. ниже)
├── px4/
│   ├── uxrce_dds_topics.patch        # DDS outputs PX4 для симуляционных сенсоров
│   ├── distance_sensor_input.patch   # DDS input PX4 для дальномера
│   └── optical_flow_input.patch      # DDS input PX4 для MTF-01 optical flow
├── worlds/
│   ├── obrik_aruco.sdf              # SITL-мир с картой ArUco-маркеров
│   ├── obrik_aruco_graffiti_wall.sdf # ArUco-карта со стеной для граффити
│   ├── orange_pi_camera_aruco_test.sdf # временный smoke test камеры над ArUco-картой
│   ├── gigaobrik_vision_test.sdf # smoke test полной сборки с двумя камерами
│   └── mtf01_aruco_test.sdf         # временный smoke test MTF-01 над ArUco-картой
├── ros_nodes/                       # colcon-пакеты, нужные ТОЛЬКО симуляции;
│   ├── graffiti_servo_gz_plugin/    #   gz-plugin кинематики серво опрыскивателя
│   ├── graffiti_servo_sim/          #   ROS 2 нода-симулятор серво
│   ├── led_strip_gz_plugin/        #   gz-plugin рендера светодиодных орбов
│   ├── led_strip_sim/              #   ROS 2 симулятор эффектов светодиодной ленты
│   ├── rangefinder_px4_bridge/     #   LaserScan -> PX4 DistanceSensor
│   └── mtf01_px4_bridge/           #   MTF image + ToF -> PX4 SensorOpticalFlow
└── models/
    ├── x500/                        # PX4 x500 wrapper (доп. визуальные joint'ы винтов)
    ├── x500_base/                   # физика NXP HoverGames x500 (апстрим PX4/Rudis Labs)
    │
    ├── x500_obrik_base/             # шасси Obrik без сенсоров (на физике x500_base)
    ├── x500_obrik_gigaobrik_base/   # остов и ВМГ Gigaobrik с физикой X500
    ├── x500_obrik_gigaobrik/        # Gigaobrik с MTF-01 OF и дальномером
    ├── x500_obrik_graffiti_base/    # legacy graffiti body and mechanism
    ├── x500_obrik_graffiti_old/     # legacy gear-and-lever graffiti configuration
    ├── x500_obrik_graffiti_cam_base/ # current graffiti body with standard Obrik rotors
    ├── x500_obrik_graffiti/         # current direct-cam graffiti configuration
    ├── x500_obrik_lidar/            # Obrik с лидаром LD19
    ├── x500_obrik_one_rangefinder/  # Obrik с одним дальномером VL53L0X (вниз)
    ├── x500_obrik_three_rangefinders_30/  # Obrik с 3 дальномерами, раскрытие 30°
    ├── x500_obrik_three_rangefinders_45/  # Obrik с 3 дальномерами, раскрытие 45°
    │
    ├── vl53l0x/                     # переиспользуемая модель дальномера VL53L0X
    ├── ld19/                        # переиспользуемая модель лидара LD19
    ├── rpi_camera/                  # переиспользуемая модель камеры Raspberry Pi
    ├── orange_pi_camera/            # Orange Pi 13 MP MIPI camera
    ├── mtf01/                       # переиспользуемая модель MicoAir MTF-01
    └── obrik_aruco_map_4x4/         # плоскость с ArUco-маркерами (словарь 4x4)
```

### Зависимости между моделями

Некоторые модели подключают другие через `<include><uri>model://...</uri></include>`
в `model.sdf` — при монтировании в PX4 обязательно нужны обе директории:

| Модель | Подключает |
|---|---|
| `x500` | — (физика зашита напрямую) |
| `x500_obrik_base` | `x500`, `rpi_camera` |
| `x500_obrik_gigaobrik` | `x500_obrik_gigaobrik_base`, `mtf01`, `orange_pi_camera` (two instances) |
| `x500_obrik_gigaobrik_base` | — (локальная копия физики и motor-model параметров X500) |
| `x500_obrik_graffiti` | `x500_obrik_graffiti_cam_base`, `rpi_camera` |
| `x500_obrik_graffiti_old` | `x500_obrik_graffiti_base`, `rpi_camera` |
| `x500_obrik_lidar` | `x500`, `ld19` |
| `x500_obrik_one_rangefinder`, `x500_obrik_three_rangefinders_*` | `x500`, `vl53l0x` |

## Graffiti Configurations

`x500_obrik_graffiti` is the current direct cam-to-cap model. It uses
`x500_obrik_graffiti_cam_base`, the standard Obrik X500 drivetrain, a
forward-facing `obrik_slam_rangefinder`, a `+Y` spray nozzle, and 104 LED
preview positions at `0.017200 m` Z. Its moving meshes are `cam.obj` and
`cap.obj` in `models/x500_obrik_graffiti/meshes/`.

`x500_obrik_graffiti_old` preserves the original gear-and-lever model.

## MicoAir MTF-01

`mtf01` — отдельная переиспользуемая модель с предоставленным mesh. Её
механическая ось измерения — локальная `+Z`, а общий оптический/ToF-центр
смещён на `(0, -0.0035, 0.010)` м. Модель использует характеристики реального
MTF-01: поле зрения optical flow 42°, частоту 100 Гц, рабочую высоту optical
flow от 0,08 м и ToF-диапазон 0,02–8 м.

В `x500_obrik_gigaobrik` MTF-01 подключён как отдельный sensor model. Его
механический origin находится в `(0, 0.0845, -0.005801)` м от `base_link` и
развёрнут roll=π, как в измеренной установке. С учётом локального смещения
сенсора `(0, -0.0035, 0.010)` м его оптический/ToF-центр находится в
`(0, 0.088, -0.015801)` м от `base_link`. Имена
`flow_link`/`lidar_sensor_link` сохранены, а PX4 `EKF2_OF_POS_*` и
`EKF2_RNG_POS_*` обновлены под фактический центр.

В ветке PX4 1.15.4, на которой собран SITL, ещё нет Gazebo
`OpticalFlowSystem`. Поэтому внутренняя 100×100 камера и ToF scan мостятся в
ROS: `mtf01_px4_bridge` вычисляет sparse optical flow и публикует
`px4_msgs/SensorOpticalFlow` в `/fmu/in/sensor_optical_flow`. Патч
`px4/optical_flow_input.patch` включает этот DDS input. Внутреннее изображение
доступно в симуляции как `/obrik/mtf01/flow/image_raw` для диагностики, но не
считается видеовыходом реального датчика. Инструкция и временный мир:
[`MTF01_TEST.md`](MTF01_TEST.md).

## Orange Pi camera

`orange_pi_camera` моделирует 13 MP MIPI-модуль OV13850/OV13855 с углом
обзора 77,6°. Рабочий поток использует реальный режим сенсора 1408×792 RGB,
30 Гц; нативный режим 4224×3136 остаётся отдельным opt-in сенсором
`imager_fullres` и не рендерится без подписчика. Это не передаёт два сырых
13 MP потока через ROS во время обычного SITL-полёта. Физическая оптическая
ось модели — локальная `+Z`, оптический центр расположен в
`(0, 0, 0.006)` м. Gazebo автоматически создаёт scoped image/camera-info
topics, поэтому модель можно безопасно включать несколько раз. В
`x500_obrik_gigaobrik` установлены две камеры: передняя смотрит вдоль `+Y`,
нижняя — вдоль `-Z`; нижняя сохраняет совместимый ROS-интерфейс
`/camera_1/*` и используется ArUco, передняя доступна как `/camera_2/*`.
Инструкция и временный проверочный мир:
[`ORANGE_PI_CAMERA_TEST.md`](ORANGE_PI_CAMERA_TEST.md).

Полную сборку Gigaobrik с обеими камерами можно проверить без PX4:

```bash
bash scripts/test_gigaobrik_vision.sh
```

## Использование в sverk-ros2

Единая точка входа — `scripts/update_overrides.sh`. Он раскладывает всё по
местам, где артефакты ждёт SITL-стек:

- `models/` → `$PX4_DIR/Tools/simulation/gz/models` (резолвятся через
  `GZ_SIM_RESOURCE_PATH`);
- `worlds/*.sdf` → `$PX4_DIR/Tools/simulation/gz/worlds`;
- `px4/*.patch` → добавляет DDS inputs дальномера и MTF-01 optical flow, а также sensor outputs в
  `$PX4_DIR/src/modules/uxrce_dds_client/dds_topics.yaml`;
- `ros_nodes/<pkg>/` → `$ROS_NODES_DIR` (по умолчанию
  `~/sverk_ws/src/sverk_drone/simulation`) — после этого пакеты собираются
  обычным `colcon build`.

Если скрипт применил DDS-патч, пересоберите PX4, чтобы сгенерировать новый
uXRCE-DDS client:

```bash
cd "$PX4_DIR" && make px4_sitl
```

Три сценария запуска:

```bash
# 1. При сборке SITL-образа (так делает sverk-ros2/scripts/sitl/Dockerfile.sitl):
#    репозиторий клонируется по тегу GZ_OVERRIDES_REF и скрипт запускается с --src.

# 2. Из checkout'а (скрипт сам поймёт, что лежит внутри репозитория):
git clone https://github.com/petayyyy/sverk_gz_overrides.git && cd sverk_gz_overrides
bash scripts/update_overrides.sh

# 3. «Ниоткуда» — например, внутри работающего контейнера sverk_sitl,
#    чтобы подтянуть свежие модели без пересборки образа
#    (скрипт сам сделает git clone, по умолчанию ветку main):
bash <(curl -fsSL https://raw.githubusercontent.com/petayyyy/sverk_gz_overrides/main/scripts/update_overrides.sh)
cd ~/sverk_ws && colcon build --packages-select \
  graffiti_servo_gz_plugin graffiti_servo_sim \
  led_strip_gz_plugin led_strip_sim mtf01_px4_bridge
```

Пути и источник переопределяются флагами `--src/--repo/--ref/--px4-dir/--ros-nodes-dir`
или переменными `GZ_OVERRIDES_REPO/GZ_OVERRIDES_REF/PX4_DIR/ROS_NODES_DIR`
(`--help` покажет всё).

В образ SITL модели и ноды запекаются на этапе сборки: `Dockerfile.sitl`
клонирует репозиторий по **тегу** (`ARG GZ_OVERRIDES_REF`) и вызывает этот
скрипт. Чтобы новая версия моделей попала в образ — создайте тег здесь и
поднимите `GZ_OVERRIDES_REF` в `Dockerfile.sitl` (значение ARG входит в ключ
docker-кэша, поэтому bump тега гарантированно перекачивает клон).

Выбор модели и мира при запуске SITL — через launch-аргументы
`full_system_sitl.launch.py` (`obrik_config`, `gz_world`), подробнее в
[docs/dev/doc_docker_sitl.md](https://github.com/petayyyy/sverk-ros2/blob/main/docs/dev/doc_docker_sitl.md)
основного репозитория.

## Формат моделей

Стандартный формат моделей Gazebo Sim (`gz-sim`, SDF 1.9): `model.config` +
`model.sdf`, меши в `meshes/*.obj` + `*.mtl`, текстуры карты ArUco в
`obrik_aruco_map_4x4/materials/textures/`.

## Лицензия и авторство

- `x500/` и `x500_base/` — производные от апстримной модели PX4
  `x500`/NXP HoverGames (© Benjamin Perseghetti, Rudis Labs), распространяются
  под BSD-3-Clause — см. [`models/x500_base/LICENSE`](models/x500_base/LICENSE).
- Остальные модели (`x500_obrik_*`, `vl53l0x`, `ld19`, `rpi_camera`,
  `orange_pi_camera`, `mtf01`,
  `obrik_aruco_map_4x4`) и мир `obrik_aruco.sdf` — оригинальные ассеты SVErk.
- Общая лицензия репозитория — см. [LICENSE](LICENSE).
