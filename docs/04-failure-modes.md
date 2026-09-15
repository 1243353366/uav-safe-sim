# 04 - Failure-mode analysis (detect → contain → recover → log)

The objective is not to prevent every failure - that is impossible. Each
row defines how the failure is **detected**, **contained** (degrade toward
a safe state, never escalate), **recovered**, and **logged**, plus the
mechanism and the test that proves it.

| # | Failure | Detect | Contain | Recover | Log | Test |
|---|---|---|---|---|---|---|
| 1 | Unit mismatch | contracts: units in field names + validation (`contract.py`, C ABI) | violating obs dropped | n/a (prevented) | contract_rejection record | `test_contracts.py`, C conformance |
| 2 | Coordinate-frame mismatch | one canonical ENU world frame; absolute angles in depth rays | invalid transforms impossible by construction | n/a | n/a (architectural) | planner/navigation tests |
| 3 | Timestamp mismatch | freshness deadlines on every message (`gps_max_age` etc.) | stale → discard | next fresh obs resumes | health STALE/FAILED transitions | `test_health.py`, `test_sensor_failures.py` |
| 4 | Race condition | single-threaded prototype; concurrency deferred to ROS 2 port where message passing replaces shared state | n/a (documented) | n/a | n/a | open question 06 |
| 5 | Serialization mismatch | typed observations at boundaries; JSON for provenance | unknown shapes dropped (chaos `_corrupt` → None) | n/a | contract_rejection | `test_chaos.py::corrupt` |
| 6 | Float divergence | no float equality on measurements; tolerance-based tests | NaN/inf rejected at contracts | n/a | contract_rejection | C ABI asserts NaN rejection |
| 7 | Stale sensor data | age vs deadline; stuck-scan plausibility check (identical rays after >1 m motion) | stale depth → full stop + LOITER; stale GPS → discard | fresh data restores trust | health transitions | `test_sensor_failures.py::stale/stuck` |
| 8 | False confidence | correlation tracked via EKF covariance; agreement ≠ independent evidence (camera+CSI weighted fusion still keeps uncertainty) | σ > 6 m → LAND | re-fix restores σ | σ in every pose record | `test_sensor_failures.py::gps_drop` |
| 9 | Model/version mismatch | model registry pinned in `provenance.py`; every inference record carries `model_version` | incompatible version = investigate, not trust | n/a | provenance records | `test_chaos.py` provenance asserts |
| 10 | Dependency drift | pinned deps (`requirements.txt`); single-file core minimizes surface | n/a | n/a | n/a | CI = fresh install |
| 11 | Runtime disagreement (NaN, overflow across languages) | contract validators mirror in C; Rust rejects NaN input explicitly | reject + drop | next valid message | contract_rejection | C conformance, Rust `nan_inputs_rejected` |
| 12 | Partial failure (NO DATA vs NO DETECTION) | health states distinguish missing vs negative; camera p_detect=0 vs drop tested separately | bounded fallback: LOITER/RTL/LAND | n/a | graded health | `test_perception_scenarios.py::false_negative` vs `cam_mode=drop` |
| 13 | Network partition | comms model with timeout (`comms_loss_rtl_after`) | local state maintained; RTL failsafe | link restore stops further vetoes | veto provenance | `test_failsafes.py::comms_loss` |
| 14 | Duplicate messages | fusion keys off timestamps: replayed obs is stale or idempotent | no double-count of identical fixes | n/a | health transitions on staleness | `test_chaos.py::duplicate` |
| 15 | Message reordering | monotonic sim clock; stale timestamps rejected | old data never overwrites newer | n/a | n/a | contract staleness tests |
| 16 | Config precedence | single `SimConfig` dataclass = one documented precedence; all thresholds in one file | n/a | n/a | n/a | config used by every test |
| 17 | Resource exhaustion | prototype: bounded log lists, O(cells) mapper; documented as ROS-2-port item (bounded queues) | n/a yet | n/a | n/a | open question 06 |
| 18 | Deadlock | no locks in prototype (single-threaded); message-passing design for the port | n/a | n/a | n/a | open question 06 |
| 19 | Safety-layer disagreement | explicit authority hierarchy (safety > avoidance > mission); safety is pure + independently unit-tested; landing never interrupted by lower-priority failsafes | any disagreement resolves to the *safer* command | n/a | every veto logged with provenance | `test_safety_layer.py` (8 unit tests) |
| 20 | AI-generated conceptual error | the whole protocol: adversarial scenario tests, chaos runs, provenance review, structural epistemics (`confirmed` cannot exist), human review of critical assumptions | tests encode the intended semantics; failures expected and recorded | iterate | experiment log | entire suite + `08-research-protocol.md` |

## Failsafe ladder (containment summary)

```
HEALTHY ──stale/missing──▶ LOITER (hover, safe)
   │                          │  conflict persists >30 s ──▶ LAND
   │                          │  depth blind while airborne ─▶ LOITER
   │                          │  link lost >10 s ──────────▶ RTL
   ├──battery ≤25%───────────▶ RTL
   ├──battery ≤12%───────────▶ LAND (immediately, wherever it is)
   └──σ >6 m ────────────────▶ LAND (cannot trust localization)
```

Every arrow is a tested transition; none increases aggressiveness.

## Emergency behavior

- **Emergency land**: critical battery or unbounded localization uncertainty
  → descend in place; on the ground, motors off and all failsafes stand down
  (`test_failsafes.py::critical_battery`, `::battery_wins`).
- **Return behavior**: RTH reserves and comms-loss both force RTL toward the
  recorded home position; reaching home transitions to landing, and landing
  is never interrupted by a failsafe that RTL would satisfy.
