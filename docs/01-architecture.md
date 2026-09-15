# 01 - Architecture

## Research platform framing

The UAV is the **test environment**; the system under study is the
architecture itself: an autonomous software stack that degrades safely
under sensor disagreement, and a polyglot component ecosystem whose
boundaries are governed by explicit contracts. See `08-research-protocol.md`.

## Dataflow

```
                          ┌───────────────────────────────────────────────┐
                          │                SIMULATED SENSORS               │
                          │  GPS   IMU   Depth/LiDAR   RGB camera   Wi-Fi CSI │
                          └───────┬────────────────────────────────────────┘
                                  │ observations (typed, timestamped, noisy)
                          ┌───────▼───────────┐
                          │   CHAOS LAYER (4)  │  fault injection between
                          └───────┬───────────┘  sensors and fusion
                          ┌───────▼───────────┐
                          │ CONTRACTS (1)      │  units/range/freshness/epistemics
                          └───────┬───────────┘  violations DROPPED, logged
          ┌───────────────────────┼─────────────────────────────┐
          │               ┌───────▼──────────┐                  │
          │               │ SENSOR FUSION    │  gated EKF        │
          │               │ (chi^2 gate)     │  rejects contradictions
          │               └───────┬──────────┘                  │
          │               ┌───────▼──────────┐   ┌────────────┐ │
          │               │ LOCALIZATION     │──▶│ HEALTH (2) │ │
          │               │ pose + sigma     │   │ HEALTHY/…  │ │
          │               └───────┬──────────┘   └─────┬──────┘ │
          │    ┌──────────┐      │        ┌────────────▼──────┐ │
          │    │ MAPPING   │◀─────┘        │ PROVENANCE (3)    │ │
          │    │ occupancy │               │ every decision    │ │
          │    └────┬──────┘               │ reconstructable   │ │
          │         │                      └────────▲─────────┘ │
          │  ┌──────▼───────┐   ┌──────────────┐     │           │
          │  │ WORLD MODEL  │   │ PERCEPTION   │─────┘           │
          │  │ static grid  │   │ camera+CSI   │                 │
          │  │ (geometry)   │   │ presence     │                 │
          │  └──────┬──────┘   │ (inferred,   │                 │
          │         │          │  never fact) │                 │
          │  ┌──────▼──────┐    └──────┬──────┘                 │
          │  │ PLANNING    │           │                        │
          │  │ A* + inflate│           │                        │
          │  └──────┬──────┘           │                        │
          │  ┌──────▼──────┐    ┌──────▼──────┐                  │
          │  │ AVOIDANCE   │    │ MISSION FSM │                  │
          │  │ stopping-   │◀───│ NAV/INSPECT/ │                 │
          │  │ distance    │    │ RTL/LAND     │                 │
          │  │ gating      │    └──────┬──────┘                  │
          │  └──────┬──────┘           │ proposes                │
          │         └─────────┐        │                         │
          │             ┌─────▼────────▼─────┐                   │
          │             │  SAFETY LAYER      │ FINAL AUTHORITY   │
          │             │  (independent veto)│ vetoes anything   │
          │             └─────┬──────────────┘                   │
          │             ┌─────▼──────────────┐                   │
          │             │ FLIGHT-CONTROL      │ accel-limited     │
          │             │ INTERFACE          │ velocity tracking  │
          │             └────────────────────┘                   │
          └──────────────────────────────────────────────────────┘
```

## Modules (src/uav/)

| Module | Responsibility | Key invariant |
|---|---|---|
| `config.py` | all thresholds in one auditable place | safety changes require test updates |
| `types.py` | typed observations/commands | presence types carry `inferred`/`confirmed` flags |
| `world.py` | 2.5-D grid world, raycast, humans, drone body | world truth never leaks into autonomy decisions |
| `sensors.py` | GPS/IMU/depth/camera/CSI simulators | every mode = a research protocol scenario |
| `contract.py` | subsystem contracts (Layer 1) | violations are dropped, never "mostly used" |
| `health.py` | graded sensor health (Layer 2) | HEALTHY/DEGRADED/STALE/FAILED/UNKNOWN |
| `fusion.py` | gated EKF | contradictions are rejected, not averaged |
| `localization.py` | fusion front-end, honest uncertainty | sigma grows when information is absent |
| `mapping.py` | occupancy grid from depth raycasts | static geometry only |
| `perception.py` | camera+CSI presence fusion | no confirmation path exists, structurally |
| `planning.py` | A* over inflated occupancy | optimistic about the unknown (avoidance guards) |
| `avoidance.py` | stopping-distance speed gating | stale/missing depth ⇒ full stop |
| `safety.py` | independent veto authority | mission cannot bypass; landing never interrupted |
| `mission.py` | FSM NAV/INSPECT/RTL/LAND/DONE | INSPECT needs cross-modal agreement |
| `comms.py` | ground-link model | loss ⇒ timed failsafe, not a freeze |
| `provenance.py` | decision records (Layer 3) | what/time/model/uncertainty/consumer/decision |
| `chaos.py` | fault injection (Layer 4) | mandatory before declaring a subsystem complete |
| `sim.py` | harness wiring all of the above | single integration point; maps 1:1 onto ROS 2 nodes |

## Authority hierarchy (explicit, tested)

1. **Safety layer** - final authority over every command. Written as a pure
   function of observable state; unit-tested independently
   (`tests/test_safety_layer.py`); mirrored as a reference implementation in
   Rust (`components/rust/safety_veto.rs`).
2. **Avoidance reflex** - may only scale motion *down*; cannot create motion
   or override failsafes.
3. **Mission FSM** - proposes; never disposes. Any veto forces a state
   transition, and there is no code path from the mission to flight control
   that skips the safety filter (`sim.step` order is fixed and covered by
   tests).

Degradation rule: stale, missing, contradictory, or low-confidence input
moves the system toward a predefined safe state (LOITER → RTL → LAND). No
failure mode produces *more* aggressive behavior. Priority order inside the
safety layer: battery-land > battery-RTH > localization-lost > sensor
conflict > comms loss > perception lost > geofence.

## ROS 2 / PX4 SITL mapping

Each `sim.py` block maps to one ROS 2 node; see `05-simulation-setup.md`
for the concrete port plan and `docker/simulation/docker-compose.yml` for
the PX4 SITL + Gazebo reference stack. The Python core is the executable
reference implementation *first*; porting starts only after the scenario
test suite passes here.
