# sverk-ros2 integration

`physical_obrik_px4_params.patch` preserves the companion SITL launch changes
required by the physical parameters in `models/x500/model.sdf` and
`models/x500_base/model.sdf`.

It sets the simulated ESC range to 720-4800 rad/s, hover thrust to 0.53, and
the tested pitch-rate gains for the standard обрик configurations. Graffiti
and Gigaobrik are excluded because their payload mass and inertia differ.

Apply from the root of a compatible `sverk-ros2` checkout:

```bash
git apply /path/to/sverk_gz_overrides/integration/sverk-ros2/physical_obrik_px4_params.patch
```

## Porter D2 and Firefighter P3

The companion selector must support `porter_D2` and `firefighter_P3`, apply
their eight-motor PX4 parameters and start the matching payload launch from
`porter_sensor_adapters`. The required model, ROS package and parameter files
are installed by `scripts/update_overrides.sh`.

After merging the companion launch changes and building an image pinned to
the corresponding overrides revision:

```bash
bash scripts/sitl/sim.sh launch --display headless --config porter_D2 --world obrik_aruco
bash scripts/sitl/sim.sh launch --display headless --config firefighter_P3 --world obrik_aruco
```

Use `firefighter_p3_cameras:=all`, `none`, or a comma-separated subset of
`n6,blackfly,foxeer` to choose P3 video bridges. N6 and Blackfly are enabled
by default. Supply this launch argument after `--` in the sim.sh command.

See the model READMEs for output topics, frames and provisional parameters.

## Firefighter P4

The companion selector also needs `firefighter_P4`, its generated eight-motor
PX4 profile, body TF alias and `firefighter_P4.launch.py` payload launch.

```bash
bash scripts/sitl/sim.sh launch --display headless --config firefighter_P4 --world obrik_aruco
```

Use `firefighter_p4_cameras:=all`, `none`, or a comma-separated subset of
`n6,blackfly,foxeer`. N6 and Blackfly bridge by default; Foxeer is optional.
