# sverk_gz_overrides

Gazebo (Harmonic) модели дронов Obrik, сенсоров и SITL-мир с ArUco-картой,
используемые для симуляции стека [sverk-ros2](https://github.com/petayyyy/sverk-ros2)
поверх стандартного PX4 SITL.

Репозиторий не самостоятелен — это набор override/дополнительных моделей PX4
`Tools/simulation/gz`, подключаемых через bind-mount или git submodule в
проект, который собирает SITL-образ (`sverk-ros2/scripts/sitl/`).

## Структура

```
sverk_gz_overrides/
├── worlds/
│   └── obrik_aruco.sdf              # SITL-мир с картой ArUco-маркеров
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

Этот репозиторий подключается как git submodule в корень `sverk-ros2`
(`px4_gz_overrides/`) и монтируется bind-mount'ом прямо в дерево PX4 внутри
SITL-контейнера — см. `scripts/sitl/docker-compose.sitl.yml`:

```yaml
volumes:
  - ../../px4_gz_overrides/models/x500:/home/sverk/PX4-Autopilot/Tools/simulation/gz/models/x500:ro
  - ../../px4_gz_overrides/worlds/obrik_aruco.sdf:/home/sverk/PX4-Autopilot/Tools/simulation/gz/worlds/obrik_aruco.sdf:ro
  # ...остальные модели аналогично
```

Подключение как submodule:

```bash
git submodule add https://github.com/petayyyy/sverk_gz_overrides.git px4_gz_overrides
git submodule update --init --recursive
```

Обновление на актуальную версию моделей:

```bash
cd px4_gz_overrides
git pull origin main
cd ..
git add px4_gz_overrides
git commit -m "chore: bump sverk_gz_overrides"
```

Выбор модели и мира при запуске SITL — через launch-аргументы
`full_system_sitl_cam.launch.py` (`obrik_config`, `gz_world`), подробнее в
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
