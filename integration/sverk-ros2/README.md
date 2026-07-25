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
