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
├── worlds/
│   └── obrik_aruco.sdf              # SITL-мир с картой ArUco-маркеров
├── ros_nodes/                       # colcon-пакеты, нужные ТОЛЬКО симуляции;
│   ├── graffiti_servo_gz_plugin/    #   gz-plugin кинематики серво опрыскивателя
│   └── graffiti_servo_sim/          #   ROS 2 нода-симулятор серво
└── models/
    ├── x500/                        # PX4 x500 wrapper (доп. визуальные joint'ы винтов)
    ├── x500_base/                   # физика NXP HoverGames x500 (апстрим PX4/Rudis Labs)
    │
    ├── x500_obrik_base/             # шасси Obrik без сенсоров (на физике x500_base)
    ├── x500_obrik_graffiti_base/    # корпус + VMG-опрыскиватель Obrik
    ├── x500_obrik_graffiti/         # Obrik-опрыскиватель (include: x500_obrik_graffiti_base)
    ├── x500_obrik_lidar/            # Obrik с лидаром LD19
    ├── x500_obrik_one_rangefinder/  # Obrik с одним дальномером VL53L0X (вниз)
    ├── x500_obrik_three_rangefinders_30/  # Obrik с 3 дальномерами, раскрытие 30°
    ├── x500_obrik_three_rangefinders_45/  # Obrik с 3 дальномерами, раскрытие 45°
    │
    ├── vl53l0x/                     # переиспользуемая модель дальномера VL53L0X
    ├── ld19/                        # переиспользуемая модель лидара LD19
    ├── rpi_camera/                  # переиспользуемая модель камеры Raspberry Pi
    └── obrik_aruco_map_4x4/         # плоскость с ArUco-маркерами (словарь 4x4)
```

### Зависимости между моделями

Некоторые модели подключают другие через `<include><uri>model://...</uri></include>`
в `model.sdf` — при монтировании в PX4 обязательно нужны обе директории:

| Модель | Подключает |
|---|---|
| `x500` | — (физика зашита напрямую) |
| `x500_obrik_*` | `x500` (общая физика мотора/рамы) |
| `x500_obrik_graffiti` | `x500_obrik_graffiti_base`, `rpi_camera` |
| `x500_obrik_lidar` | `x500`, `ld19` |
| `x500_obrik_one_rangefinder`, `x500_obrik_three_rangefinders_*` | `x500`, `vl53l0x` |

## Использование в sverk-ros2

Единая точка входа — `scripts/update_overrides.sh`. Он раскладывает всё по
местам, где артефакты ждёт SITL-стек:

- `models/` → `$PX4_DIR/Tools/simulation/gz/models` (резолвятся через
  `GZ_SIM_RESOURCE_PATH`);
- `worlds/*.sdf` → `$PX4_DIR/Tools/simulation/gz/worlds`;
- `ros_nodes/<pkg>/` → `$ROS_NODES_DIR` (по умолчанию
  `~/sverk_ws/src/sverk_drone/simulation`) — после этого пакеты собираются
  обычным `colcon build`.

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
cd ~/sverk_ws && colcon build --packages-select graffiti_servo_gz_plugin graffiti_servo_sim
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
  `obrik_aruco_map_4x4`) и мир `obrik_aruco.sdf` — оригинальные ассеты SVErk.
- Общая лицензия репозитория — см. [LICENSE](LICENSE).
