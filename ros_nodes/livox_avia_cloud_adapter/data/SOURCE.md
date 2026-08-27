# Avia scan sequence

`avia_pattern_i16.bin.gz` contains 960,000 `(azimuth, elevation)` pairs in
centidegrees, derived from Livox's published Avia scan-mode CSV. It represents
four seconds at the strongest/single-return rate of 240,000 points/s.

- Source: https://github.com/Livox-SDK/livox_laser_simulation/blob/master/scan_mode/avia.csv
- Raw CSV SHA-256: `9363f7ec8da1541b49214ae18eea840151eda4eed176ebd573bd443eb7970d78`
- Conversion: azimuth is retained; elevation is `90 - zenith`; both are
  rounded to the nearest 0.01 degree and stored as little-endian signed int16.
- Upstream repository license: BSD-3-Clause.
