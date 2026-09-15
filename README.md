# Experimental Polyglot Autonomous Systems Platform

**A research platform, not a drone project.** The UAV is the test
environment: this repository studies whether an AI-assisted development
agent can construct and maintain a complex distributed autonomous-system
architecture - and whether deliberate language specialization justifies
its costs.

> **Research Question:** Can an AI-assisted development agent coordinate a
> heterogeneous autonomous-system architecture spanning approximately 20
> programming languages while maintaining correctness, observability,
> reproducibility, and fault tolerance?

If the answer is "no, and here is where it breaks", that is a result.

## What this is

A research-grade, **non-weaponized** autonomous-UAV **simulation**: a
complete autonomy pipeline (sensors → perception → sensor fusion →
localization → world model → planning → avoidance → mission state machine
→ flight-control interface) with an **independent safety layer holding
final veto authority**, and a four-layer robustness architecture
(contracts, graded health states, provenance, chaos testing).

```bash
pip install -r requirements.txt
python -m pytest tests -q    # 59 tests: scenarios, failures, chaos, unit
```

## What it does

- simulated flight in a 2.5-D world with deterministic seeds
- autonomous waypoint navigation (A* over a mapped occupancy grid)
- obstacle avoidance with stopping-distance guarantees
- mapping + localization (gated EKF with honest, growing uncertainty)
- sensor fusion of simulated GPS / IMU / depth / RGB camera / **Wi-Fi CSI**
- camera-based human detection (simulated detector statistics)
- Wi-Fi CSI presence sensing as an **experimental** sensor with a
  three-stage pipeline (signal → measurement → inference)
- uncertainty estimation exposed everywhere (σ in every pose record)
- safe behavior when sensors disagree, degrade, lie, stale, freeze, or die
- full provenance: every significant decision is reconstructable

## What it does NOT do

- **No weapons, weapon payloads, autonomous target selection for attack,
  firing logic, or explosive payloads.** Civilian/research scope only:
  navigation, mapping, inspection, environmental sensing, search-and-rescue.
- No physical flight hardware. Simulation only (PX4 SITL port is planned -
  see `docs/05-simulation-setup.md`).
- No confirmation of humans. Presence is **inferred, never confirmed** -
  structurally: no code path can turn an inference into a confirmed fact.
- No copied code from the studied repositories (see below).

## Experimental Design Rationale ("why the hell is this so complicated?")

The 20-language matrix is **intentionally excessive**. The architecture is
designed to test specialization versus complexity. Some languages may
ultimately be removed. Failures are expected and are part of the
experiment. The AI agent is being evaluated as much as the software
itself.

### Known Experimental Excess

> The architecture intentionally contains more languages and subsystem
> boundaries than would normally be recommended for a production UAV. This
> is deliberate. The additional complexity is an experimental variable
> rather than an accidental design requirement.

Outcome so far (evidence, not vibes): asked to justify each language, the
agent assigned real code to 9 of 20 - 5 verified (Python, C, C++, JS, SQL),
4 reference (Rust, Go, TypeScript, Haskell) - and documented deferrals and
rejections with rationale for the other 11
(`docs/07-language-assignments.md`).

### Success = learning, not completion

"Success" is measurable evidence about which approaches succeed or fail:
Rust's independent veto mirror improving auditability (success), a
language boundary creating unacceptable drift (failure), Python remaining
the only fully-verified implementation (unexpected), or the agent's own
conceptual mistakes caught by adversarial tests (AI failure). Recorded
examples of all four live in `docs/08-research-protocol.md`.

## The four layers

| Layer | What | Where |
|---|---|---|
| 1. Contracts | units/ranges/deadlines/error states/version at every boundary; violations dropped | `src/uav/contract.py`, `components/c/sensor_abi.h` |
| 2. Health state | HEALTHY/DEGRADED/STALE/FAILED/UNKNOWN, never binary | `src/uav/health.py` |
| 3. Observability | every decision reconstructable: observation, time, model version, uncertainty, consumer, result | `src/uav/provenance.py`, `components/sql/` |
| 4. Chaos testing | drop/corrupt/delay/duplicate/contradict/restart mid-flight - then: detect? contain? recover? log? | `src/uav/chaos.py`, `tests/test_chaos.py` |

**Failure injection is mandatory before any subsystem may be declared
complete.**

## Safety architecture

Mission-level logic **proposes**; the independent safety layer (a pure,
unit-tested function of observable state) **disposes**. No code path
reaches flight control without passing through it. Every failsafe degrades
toward a predefined safe state - loiter, RTL, land - and no failure mode
produces *more* aggressive behavior. Landing is never interrupted by a
failsafe that landing already satisfies. The full priority chain and
failsafe ladder: `docs/04-failure-modes.md`.

## Test suite (59 tests)

- waypoint navigation (nominal + walled worlds with gaps to route through)
- obstacle avoidance (braking, gap-finding, zero collisions)
- sensor failure: GPS drop/stale, depth drop/stuck (frozen values), IMU drop
- GPS degradation (silent, 10x true noise) and GPS/IMU contradiction (bias jump)
- camera false positives, false negatives; CSI-alone never confirms
- communications loss, low battery, critical battery, emergency landing
- safety-layer authority (8 unit tests pinning the priority table)
- contracts, health-state transitions, provenance completeness
- chaos: corrupted NaN data rejected at contracts; contradictions declare
  conflict; multiple seeds, obstacle world + chaos; always: no collisions

## Polyglot components (`components/`)

Verified: Python core, C sensor ABI, C++ flight-control, JS telemetry
consumer, SQL provenance schema. Reference (unverified, conformance tests
pending toolchains): Rust safety mirror, Go relay, TS dashboard types,
Haskell FSM ADT experiment. **Do not assume components are compatible
merely because they have similar names** - compatibility is established
only by executed conformance tests. Status table:
`components/README.md`.

## Studied projects (license-audited, zero code copied)

PX4/PX4-Autopilot (BSD-3-Clause), ros-navigation/navigation2 (multi:
Apache-2.0/BSD-3/LGPL-2.1+), euaziel/WiFi-CSI-Human-Pose-Detection
(GPL-3.0 - **excluded**), ultralytics/ultralytics (AGPL-3.0 -
**excluded**), and one redacted item pending audit. Full audit with
compatibility matrix and human-review flags: `docs/02-license-audit.md`.
The fourth repository from the research list was redacted (it contained a
leaked credential) and is **not** covered by the audit.

## Documentation

- `docs/01-architecture.md` - modules, dataflow, authority hierarchy
- `docs/02-license-audit.md` - repositories, dependencies, licenses, matrix
- `docs/03-sensor-fusion.md` - EKF, gating, CSI pipeline, epistemics
- `docs/04-failure-modes.md` - 20 failure modes: detect→contain→recover→log
- `docs/05-simulation-setup.md` - prototype + PX4 SITL/Gazebo/ROS 2 plan
- `docs/06-open-questions.md` - uncertainties, stated rather than hidden
- `docs/07-language-assignments.md` - the 20-language hypothesis and decision table
- `docs/08-research-protocol.md` - research question, four layers, success criteria, scope

## License and credit

This project is licensed under **Apache-2.0** (`LICENSE`). All code here is
original. No code, model weights, or data from the studied repositories is
included; their authors are credited in `CREDITS.md` and
`THIRD_PARTY_NOTICES.md`. No attribution requirement of any dependency is
currently triggered because no dependency code is redistributed.
