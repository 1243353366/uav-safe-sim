# Credits

This project studies the following open-source projects as architecture
references. **No code, model weights, or data from them is included in
this repository** - every module here is an original implementation.

## PX4 / PX4-Autopilot
- Authors: PX4 Development Team / Dronecode Foundation
- Repository: https://github.com/PX4/PX4-Autopilot
- License: BSD 3-Clause
- Credited for: the autopilot architecture studied for the flight-control
  interface and SITL staging (docs/05-simulation-setup.md).

## ros-navigation / navigation2 (Nav2)
- Authors: the Nav2 / ROS 2 navigation community
- Repository: https://github.com/ros-navigation/navigation2
- License: Apache-2.0 / BSD-3-Clause / LGPL-2.1-or-later (per package)
- Credited for: navigation framework concepts studied for the planning and
  behavior layers.

## euaziel / WiFi-CSI-Human-Pose-Detection
- Author: euaziel
- Repository: https://github.com/euaziel/WiFi-CSI-Human-Pose-Detection
- License: GPL-3.0
- Credited for: the concept of Wi-Fi-CSI-based human presence sensing,
  which inspired (conceptually) the simulated CSI pipeline. No code was
  used; GPL-3.0 made inclusion incompatible with this project's
  Apache-2.0 license.

## Ultralytics / ultralytics
- Authors: Ultralytics team
- Repository: https://github.com/ultralytics/ultralytics
- License: AGPL-3.0
- Credited for: the YOLO-family detector interface conventions studied for
  the simulated camera-detection statistics. No code or weights were used;
  AGPL-3.0 made inclusion incompatible with this project's Apache-2.0
  license.

## numpy
- Authors: NumPy developers
- License: BSD 3-Clause
- Used as a runtime dependency (listed in requirements.txt).

One item on the original research list was redacted (it contained a leaked
credential) and could not be audited or credited; see
docs/02-license-audit.md.
