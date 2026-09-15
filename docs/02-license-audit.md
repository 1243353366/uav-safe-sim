# 02 - Repository / dependency / license audit

Audited 2026-09-15 via the GitHub REST API (unauthenticated; the saved
`$GITHUB_TOKEN` was rejected as invalid). Facts below marked *(verified)*
come from the API response; the rest comes from project documentation and is
flagged for human review.

**Policy actually followed in this repo: no code, model weights, or data
from any of these repositories has been copied into this project.** All
modules are original implementations. The projects below are studied as
architecture references and (in the future) runtime dependencies kept
behind interfaces. Copying code because it is publicly visible is not
licensed reuse; that is why nothing was copied.

> **Missing item:** the 4th repository on the research list was redacted
> from the request (it contained a leaked credential, now stored as
> `$GITHUB_TOKEN`). It is **not** covered by this audit. Send the repo
> name and it will be audited before any use.

## 1. PX4 / PX4-Autopilot

- URL: https://github.com/PX4/PX4-Autopilot *(verified)*
- Maintainer: PX4 (org; project governed by the Dronecode Foundation) *(verified)*
- Purpose: production open-source autopilot/flight-control stack
- Relevant modules: SITL (`sitl/`), gazebo simulator integration, uXRCE-DDS
  bridge (`src/modules/uxrce_dds_client`), navigator/flight modes, EKF2
- Languages: C++, C, CMake, Python *(verified via API language stats)*
- Key dependencies: MAVLink, micro-XRCE-DDS, Gazebo (SITL), NuttX (hardware)
- License: **BSD 3-Clause** *(verified: root `LICENSE`, "Copyright (c) 2012 - 2025, PX4 Development Team")*
- Modification: permitted. Redistribution: must retain the license text and
  attribution. Attribution: required (notice retention).
- Model/data licenses: PX4 ships no ML models; datasets in-tree are not
  used by this project.
- Compatibility with Apache-2.0 (our license): **compatible**.
- ⚠ PX4 is a *submodule collection*: individual submodules may carry their
  own licenses. If code is ever vendored, audit the specific subdirectory.
  **Human review required** before any vendoring.
- Files: `LICENSE` (root) *(verified)*.

## 2. ros-navigation / navigation2 (Nav2)

- URL: https://github.com/ros-navigation/navigation2 *(verified)*
- Maintainer: ros-navigation (org) *(verified)*
- Purpose: ROS 2 navigation framework (planner, controller, behavior,
  recovery servers, costmaps)
- Relevant modules: `nav2_planner`, `nav2_controller`, `nav2_behaviors`,
  `nav2_costmap_2d`, `nav2_bt_navigator`
- Languages: C++, Python, CMake *(verified)*
- Key dependencies: ROS 2 stack (rclcpp, nav2 message packages, pluginlib)
- License: **multi-licensed** *(verified: root `LICENSE` is a meta-file -)*
  "Portions of this repository are available under one of the following
  licenses: LGPL-2.1-or-later, Apache-2.0, BSD-3-Clause"; per-package
  license lives in each package's `package.xml`; unmarked files default to
  Apache-2.0.
- Modification: permitted under the per-package terms. Redistribution:
  per-license obligations. Attribution: required.
- Compatibility with Apache-2.0: **compatible** (Apache-2.0 components);
  LGPL components would impose copyleft if linked - we do not link them in
  this prototype. **Human review required** before any Nav2 component is
  vendored into a distributed build.
- Files: `LICENSE` (root meta-license), per-package `package.xml`
  *(verified paths; package contents not individually fetched)*.

## 3. euaziel / WiFi-CSI-Human-Pose-Detection

- URL: https://github.com/euaziel/WiFi-CSI-Human-Pose-Detection *(verified -
  selected from 4 API search hits as the only name match with meaningful
  activity; 128 stars)*
- Maintainer: euaziel (individual) *(verified)*
- Purpose: human pose estimation from Wi-Fi CSI via deep learning
  ("camera-free sensing through walls")
- Languages: Rust, Python, JavaScript *(verified)*
- License: **GPL-3.0** *(verified: root `LICENSE`, full GPLv3 text)*
- Modification/redistribution: permitted only under GPL-3.0 copyleft terms.
- Attribution: required; derivative works must also be GPL-3.0.
- Compatibility with Apache-2.0: **INCOMPATIBLE** for code inclusion. A
  combined work would force GPL-3.0 over the whole derivative.
- **Decision: no code from this repository is or will be included.** It is
  studied conceptually (pipeline structure: CSI frames → preprocessing →
  pose inference). Our CSI model (`src/uav/sensors.py`) is an original
  simulation of presence-sensing statistics, not their code. If their
  approach is ever used as a runtime subprocess, treat it as a separate
  GPL-licensed process and obtain human legal review on aggregation.
- Files: `LICENSE` (root) *(verified)*.
- ⚠ Also flag: a repo created very recently claiming "through-wall"
  detection warrants a technical-credibility review before being cited as
  an approach reference.

## 4. (redacted - not audited)

See the note at the top. This slot stays open on purpose.

## 5. ultralytics / ultralytics

- URL: https://github.com/ultralytics/ultralytics *(verified)*
- Maintainer: Ultralytics (company) *(verified)*
- Purpose: YOLO-family object detection/segmentation/classification
- Relevant modules: YOLO model APIs, tracking, export
- Languages: Python, Shell *(verified)*
- Key dependencies: torch, torchvision, opencv-python, numpy, matplotlib,
  pillow, pyyaml, requests, polars, psutil *(verified from `pyproject.toml`
  on `main`)*
- License: **AGPL-3.0** *(verified: root `LICENSE`, full AGPLv3 text; the
  pyproject header states "Ultralytics 🚀 AGPL-3.0")*
- Modification/redistribution: permitted under strong copyleft; **network
  use counts as distribution** (§13) - users of a network service must be
  offered the source.
- Model/data licenses: YOLO pretrained weights are distributed by
  Ultralytics under AGPL-3.0 as well (per their FAQ at ultralytics.com -
  *not verified from the repo; flagged*). Dataset licenses (COCO etc.) are
  separate and impose their own terms.
- Compatibility with Apache-2.0: **INCOMPATIBLE for code/weights inclusion**
  in an Apache-2.0 project.
- **Decision: no ultralytics code or weights are included.** The camera
  human detector is an original simulated model of detector *statistics*
  (`p_detect`, `p_false`, confidence distribution). A real detector would
  run as a separate process behind the same contract. **Human legal review
  required** before any deployment that ships ultralytics weights, and
  before deciding whether a separate-process integration triggers §13.
- Files: `LICENSE` (root), `pyproject.toml` (dependency + license
  declarations) *(verified)*.

## License compatibility matrix

| Dependency | Its license | Use mode in this project | Apache-2.0 compatible? | Action |
|---|---|---|---|---|
| PX4-Autopilot | BSD-3-Clause | SITL runtime (docker), never vendored | Yes | OK; audit submodules before vendoring |
| navigation2 | Apache-2.0 / BSD-3 / LGPL-2.1+ (per package) | ROS 2 reference, never vendored | Apache parts yes; LGPL parts no | OK for reference; per-package review before vendoring |
| WiFi-CSI-Human-Pose-Detection | GPL-3.0 | concept study only, zero code | **No** | Excluded; legal review before any use |
| ultralytics | AGPL-3.0 (+AGPL weights) | interface study only, zero code/weights | **No** | Excluded from repo; legal review before any deployment |
| numpy (our dep) | BSD-3-Clause | runtime dependency | Yes | standard attribution |

## Requirements before incorporating ANY external code

1. Identify the license of the exact file/submodule (not the repo root).
2. Record the license text file and commit it under `THIRD_PARTY_NOTICES.md`.
3. Identify the dependency tree and each dependency's license.
4. Identify model/dataset licenses separately from code licenses.
5. Confirm compatibility with Apache-2.0 for this project.
6. Flag anything requiring human legal review - and do not merge until
   that review is recorded.
