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
    ├── x500_obrik_firefighter_base/ # decimated Firefighter body on Gigaobrik collision
    ├── x500_obrik_firefighter/      # Firefighter with lidar, MTF-01, and optional cameras
    ├── porter/                       # P-2.2 / Porter X8 with source mass and inertia
    ├── porter_D2/                    # Porter D2 X8 with five payload sensors
    ├── firefighter_P3/               # Firefighter P3 X8 with six payload sensors
    ├── vkuss_vill_v4/                # П-4 X8: base_link = Cube FC, Avia, MV-SUA133GC, H-Flow
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
    ├── foxeer_t_rex_mini_camera/    # Foxeer T-Rex Mini analog FPV camera
    ├── bfs_pge_50s5c_camera/         # Blackfly S camera with lens
    ├── mtf01/                       # переиспользуемая модель MicoAir MTF-01
    ├── livox_avia/                  # Livox Avia 3D lidar, two realistic scan modes
    └── obrik_aruco_map_4x4/         # плоскость с ArUco-маркерами (словарь 4x4)
```

### Зависимости между моделями

Некоторые модели подключают другие через `<include><uri>model://...</uri></include>`
в `model.sdf` — при монтировании в PX4 обязательно нужны обе директории:

| Модель | Подключает |
|---|---|
| `x500` | — (физика зашита напрямую) |
| `x500_obrik_base` | `x500`, `rpi_camera` |
| `unitree_l2` | — (Unitree L2 3D lidar: normal and NAGE GPU-lidar modes; visible rotor at 333 RPM) |
| `brio_95_camera` | — (Logitech BRIO 95 1080p / 30 fps / 58° diagonal FoV) |
| `foxeer_t_rex_mini_camera` | — (Foxeer T-Rex Mini 4:3 FPV feed, 640x480 / 30 fps / 120° horizontal FoV) |
| `x500_obrik_gigaobrik` | `x500_obrik_gigaobrik_base`, `mtf01`, `orange_pi_camera` (two instances) |
| `x500_obrik_gigaobrik_base` | — (локальная копия физики и motor-model параметров X500) |
| `x500_obrik_firefighter` | `x500_obrik_firefighter_base`, `unitree_l2`, `mtf01`, `brio_95_camera` (five instances), `foxeer_t_rex_mini_camera` |
| `x500_obrik_firefighter_base` | decimated Firefighter body and VMGs; Gigaobrik collision/inertia; invisible X500 physics rotors |
| `porter` | `livox_avia` (full coaxial-X8 vehicle generated from the P-2.2 description) |
| `porter_D2` | `livox_avia`, `porter_D2_hflow`, `porter_D2_rangefinder_200m`, `porter_D2_n6_camera`, `porter_D2_mv_ca050_camera` |
| `firefighter_P3` | `livox_avia`, `bfs_pge_50s5c_camera`, `firefighter_P3_hflow`, `firefighter_P3_rangefinder_200m`, `firefighter_P3_n6_camera`, `firefighter_P3_foxeer_camera`; VMG mesh from `x500_obrik_firefighter_base` |
| `firefighter_P3_hflow`, `firefighter_P3_rangefinder_200m`, `firefighter_P3_n6_camera` | Meshes from the corresponding `porter_D2_*` models |
| `firefighter_P3_foxeer_camera` | `foxeer_t_rex_mini_camera` mesh |
| `vkuss_vill_v4` | `livox_avia`, `vkuss_vill_v4_mv133_camera`, `vkuss_vill_v4_hflow`; rotor meshes from `firefighter_P3` |
| `vkuss_vill_v4_hflow` | Mesh from `porter_D2_hflow` |
| `x500_obrik_gigaobrik_lidar` | `x500_obrik_gigaobrik_lidar_base`, `mtf01`, `brio_95_camera` (downward), `unitree_l2` |
| `x500_obrik_gigaobrik_lidar_base` | original Gigaobrik VMG meshes; the supplied clean body mesh; the same X500 collision and motor model |
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

## Firefighter

`x500_obrik_firefighter` combines the decimated Firefighter fuselage and
coaxial VMG visuals with the Gigaobrik collision/inertia envelope and the
four physical X500 rotors. The upper and lower VMGs are visually driven from
the same four actuator channels; the extra four are visual only and add no
lift. The configuration includes a Unitree L2, MTF-01, Foxeer FPV camera, and
five on-demand BRIO cameras (`bottom`, `front`, `rear`, `left`, `right`).
Camera selection and the bottom-camera ArUco pipeline are configured by the
`sverk-ros2` SITL launch file, not by this model.

## Livox Avia

`livox_avia` is a reusable, sensor-native module: local `+X` is the optical
axis, `+Y` points left, and `+Z` points up. It includes the 498 g housing mesh,
a conservative 190 m daylight range, 1 m blind zone, 2 cm Gaussian range
noise, and both real scan modes:

- `nonrepetitive`: 70.4° × 77.2°, shaped by Livox's published four-second
  Avia trajectory;
- `repetitive`: 70.4° × 4.5° line scan.

Both publish 24,000 points at 10 Hz, corresponding to the Avia
strongest/single-return rate of 240,000 points/s. Only the selected Gazebo
topic is rendered, since each GPU lidar is demand activated. The simulation
does not emulate reflectivity-dependent range, the 1–3 m distortion region,
beam divergence, or dual/triple echoes.

The cloud adapter enables `texel_fix=true` by default. It preserves each measured
range while placing the point on the centre ray of the Ogre2 cubemap texel that
supplied that range. Both Avia scan modes are corrected before publication;
timestamps and non-XYZ fields are preserved. The correction supports the Avia
field of view on the `+X` cube face; it must not be applied to direct-raycast data
or assumed valid for wider-field lidars. Disable it only to reproduce uncorrected
historical data. The optional `cube_face_texels` parameter overrides automatic
cubemap-size detection (`0` selects the size from the incoming GPU scan grid).

This repository is the source for the adapter installed by
`scripts/update_overrides.sh`. SITL image builders must pin a revision containing
the correction; changing only a vendored copy in `sverk-ros2` is overwritten by
that installation step.

## Porter

`porter` is the full P-2.2 coaxial-X8 vehicle, not an X500 visual shell. It is
generated reproducibly by `scripts/generate_porter_model.py` from the compiled
`p22_description` URDF and keeps the source component placement, primitive
collisions, centre of mass, inertia, 5.201 kg simulated takeoff mass, and all
eight physical motor plugins. Its reusable Livox Avia publishes the selected
scan through `/livox_avia/points`.

The source package marks its CAD meshes as proprietary. The copied Porter
meshes must not be redistributed until their distribution terms are confirmed;
see `models/porter/SOURCE.md`.

## Porter D2 and Firefighter P3

`porter_D2` and `firefighter_P3` are separate X8 configurations with eight
physical motor channels, decimated CAD meshes and fitted box collisions.
Both use +X forward, +Y left and +Z up. Mass, inertia, motor performance,
unidentified lenses and new sensor noise remain provisional.

The `porter_sensor_msgs` and `porter_sensor_adapters` packages provide payload
bridges, mounting TF, image-based H-Flow observations and range outputs. Livox
uses `livox_avia_cloud_adapter`. Tested runtime: Gazebo Harmonic 8.15, ROS 2
Humble and PX4 v1.15. Payload range/flow streams do not automatically aid PX4.

Model documentation includes launch commands, topics, sensor assumptions and
regeneration commands:

- [Porter D2](models/porter_D2/README.txt)
- [Firefighter P3](models/firefighter_P3/README.txt)
- [vkuss_vill_v4 / П-4](models/vkuss_vill_v4/README.txt)
- [Blackfly S camera](models/bfs_pge_50s5c_camera/README.txt)

`vkuss_vill_v4` (П-4, `scripts/generate_vkuss_vill_v4.py`) puts `base_link` at the
Cube flight controller, the datum of the real drone's `base_link`, so the PX4
IMU sits at its origin and the Avia, MindVision MV-SUA133GC (calibrated
intrinsics, 20 Hz) and H-Flow poses come from the 21.10 STEP and the
`rig_mv133.yaml` Rcl/Pcl; `tests/test_vkuss_vill_v4.py` pins them.

The P3 Foxeer FPV image bridge is off by default. Use an existing world such
as `obrik_aruco`; the shared sverk-ros2 launch spawns the selected vehicle.
The shared Avia topics currently allow one Avia instance per world.

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
  led_strip_gz_plugin led_strip_sim mtf01_px4_bridge \
  unitree_l2_rotor_gz_plugin porter_sensor_msgs porter_sensor_adapters \
  livox_avia_cloud_adapter
```

Пути и источник переопределяются флагами `--src/--repo/--ref/--px4-dir/--ros-nodes-dir`
или переменными `GZ_OVERRIDES_REPO/GZ_OVERRIDES_REF/PX4_DIR/ROS_NODES_DIR`
(`--help` покажет всё).

В образ SITL модели и ноды запекаются на этапе сборки: `Dockerfile.sitl`
получает репозиторий по ревизии `ARG GZ_OVERRIDES_REF` и вызывает этот
скрипт. После публикации изменений укажите полный SHA коммита overrides
в `Dockerfile.sitl` и пересоберите образ.

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

### Gazebo sensor corrections (2026-10-19)

Avia's ideal IMU runs at 250 Hz on the 4 ms physics grid. Its orientation is
relative to its initial sensor orientation; `sensor_realism` handles this
convention. Keep ideal streams noise-free and enable realism separately.
The existing Avia +X-face texel correction remains enabled; its scan pattern
uses simulation timestamps, including after dropped frames and adapter restarts.

World gravity is 9.80665 m/s². Gazebo magnetometer noise is expressed in tesla
(1e-7 T); PX4 v1.15 normally uses its own WMM `sensor_mag_sim` instead.
`gazebo_camera_tools` converts bridged CameraInfo from Gazebo to OpenCV pixel
centres on separate raw/public topics. The author-calibrated FLIR camera is
`bfs_pge_50s5c_calibrated`; it does not replace the user's D2/P3 camera choices.

The installer now applies two PX4 source patches, idempotently:

- `gazebo-sensor-truth.patch`: 15 °C barometer temperature, nonzero global truth
  publication timestamps, and opt-in `SIM_GZ_FLAT_ALT=1` for flat Cartesian maps.
  Flat altitude preserves the initial datum and follows world Z. Use zero for
  intentionally spherical height simulations. GPS delay is set to 20 ms in
  the matching SITL launch and D2/P3 profiles.
- `hrt-periodic-reschedule.patch`: prevent duplicate insertion of a periodically
  rescheduled POSIX timer into the timer queue. This avoids stalled callbacks.

After installing from this checkout with `scripts/update_overrides.sh --src
/path/to/sverk_gz_overrides`, rebuild PX4 (`make px4_sitl_default`) and the affected
ROS packages. Copying models does not update the PX4 binary. The patch installer
fails clearly on an incompatible PX4 source revision. The patches were built and
flight-tested on the local PX4 v1.15 fork at `78bd1cc`.
