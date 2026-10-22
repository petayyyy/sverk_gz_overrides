# Старые стриминговые миры (Бурный, Хельсинки 4×4 км), установленные 09–18.09

Снимок файлов из `PX4-Autopilot/Tools/simulation/gz` SITL-хоста popka (23.10). Там они лежали
неотслеживаемыми в клоне PX4-gazebo-models и больше нигде не хранились. Раскладка путей та же,
что в `Tools/simulation/gz`. Ветка отдельная, в `main` не вливается, `update_overrides` её не ставит.

Что есть:
- `burniy_airfield_streaming/`, `helsinki_streaming_4x4km/` — `tile_streamer.py` и GUI-плагин
  позы камеры (исходники и скрипт сборки). Собранные `plugins/*.so` и сгенерированные `qrc_*.cc`
  не включены.
- `worlds/` — `burniy_airfield*.sdf`, `helsinki_4x4km*.sdf`.
- `models/` — `aruco_landing_pad`, `burniy_landing_pad`, `burniy_airfield_collision` (collision.obj,
  10 МБ). От тайловых моделей `burniy_airfield_chunked` и `helsinki3d_2017_l21_chunked_4x4km` взяты
  только индексы (`model.sdf`, `model.config`, `tiles.json`).

Чего нет: мешей тайлов `meshes_glb*` (Бурный 2.5 ГБ, Хельсинки 4×4 км 5.1 ГБ). Без них миры не запускаются.

Чем заменено: стример с ограниченным радиусом `scripts/sitl/tile_streamer_radius.py` в sverk-ros2
(после краша на перезагрузке тайлов, `docs/dev/bugreport_tile_streamer_2026-09-23.md`). Наборы
Хельсинки 2×11 км и фабрики ставятся из своих архивов аддонов, поэтому здесь их нет.
