# 05 - Simulation setup

## Simulation first - no physical hardware

This project deliberately does not touch physical flight hardware. The
working prototype (`src/uav/`) is a self-contained simulation that
exercises the *architecture*; the PX4 SITL + Gazebo + ROS 2 stack is the
staged path from this prototype toward a real system-under-test.

## Stage 1 (implemented here): pure-Python reference simulation

```bash
pip install -r requirements.txt
python -m pytest tests -q        # 59 tests, all scenarios
```

Scenario runs are constructed directly:

```python
from uav.config import SensorSetup
from uav.sim import Simulation
from uav.chaos import ChaosHarness

sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(gps_mode="degraded"))
log = sim.run(80.0)
```

Failure injection is a constructor argument (`SensorSetup`) or a live
harness (`ChaosHarness`), never a code edit.

## Stage 2 (reference configuration): PX4 SITL + Gazebo + ROS 2

`docker/simulation/docker-compose.yml` is the reference stack:

- **px4-sitl**: PX4 SITL in docker, uXRCE-DDS on UDP 8888
- **ros2**: ROS 2 bridge node (subscribe `fmu/out/*`, publish setpoints `fmu/in/*`)
- **gazebo**: simulator host

Port plan - each `sim.py` block becomes one ROS 2 node:

| sim.py block | ROS 2 node | PX4 / ROS interfaces |
|---|---|---|
| sensors.py reads | Gazebo sensor plugins | ground truth → noise model node |
| fusion + localization | `uav_localizer` | `PoseWithCovarianceStamped` |
| mapping | `uav_mapper` | `nav_msgs/OccupancyGrid` |
| perception | `uav_perception` | custom `PresenceAssessment` msg |
| planning + avoidance | `nav2_planner` + `uav_avoidance` | Nav2 action server |
| mission FSM | `uav_mission` | behavior tree / FSM node |
| safety layer | `uav_safety` (separate exec, watchdog-shaped) | final gate before `fmu/in/*` |
| provenance | `uav_provenance` | SQLite (see components/sql) |

The safety node stays **outside** the mission process: it consumes the
same telemetry topics and vetoes by *replacing* the setpoint stream,
mirroring the authority hierarchy proven in the prototype.

`ros2-interface/` contains the package skeleton (`package.xml`, launch
file) marked REFERENCE ONLY - it is not built in CI yet.

## Why not start with SITL in CI?

The prototype answers architecture questions (gating, epistemics, failsafe
ordering, chaos behavior) with 100x faster iteration and deterministic
seeds. The SITL stack answers platform questions (real DDS timing, actual
PX4 modes). Running the chaos matrix against SITL is planned *after* the
Python-level suite is green - which it now is.
