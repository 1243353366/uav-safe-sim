# 03 - Sensor fusion design

## State and prediction

EKF over the 2-D state `x = [x, y, vx, vy]` (SI units, ENU world frame;
altitude is a fixed abstraction at this stage - see open questions).

- **Predict** each tick from IMU accelerations:
  `x' = F x + G a`, with `F` the constant-velocity transition and `G` the
  control matrix mapping `(ax, ay)` into position/velocity over `dt`.
  Process noise `Q = G Σa Gᵀ` with `σa = 0.6 m/s²`.
- **No usable IMU** → coast with zero acceleration and 6x process noise:
  coasting is honest about knowing less.
- **Update** from GPS fixes with measurement noise `R = σ²I`.

## Innovation gating (contradiction handling)

Every GPS fix must pass a Mahalanobis gate before fusion:

```
ν = z - H x̂            (innovation)
S = H P Hᵀ + R          (innovation covariance)
d² = νᵀ S⁻¹ ν          (gated at χ²(2 dof, 99.9%) = 13.82)
```

- `d² ≤ gate`: normal Kalman update.
- `d² > gate`: the fix is **rejected, not averaged in**. Rejections are
  counted; 5 consecutive rejections declare a **sensor conflict**, the
  source is excluded, and the safety layer loiters (then lands if the
  conflict persists).
- A hard bias (e.g. +40 m) produces `d²` in the thousands - it can never
  pass. Silent degradation (true noise 10x the reported sigma) also trips
  the gate: this is the designed detection path for lying sensors.

## Uncertainty semantics

- Position uncertainty = `σ = sqrt(λ_max(P[0:2,0:2]))` - the 1-sigma radius
  the rest of the stack sees. It appears in every pose record.
- **With no usable GPS fix**, position variance is grown by a conservative
  `0.1 m²/tick` plus the natural EKF growth from uncorrected velocity
  uncertainty. Dead reckoning after a conflict reaches `σ > 6 m` in
  seconds - and the safety layer lands the aircraft rather than navigate
  on belief (`localization_lost`).
- Stale fixes (`age > gps_max_age`) are discarded - never silently reused.

## Wi-Fi CSI pipeline (experimental sensor, never ground truth)

Three stages, kept explicitly distinct in `CsiObs`:

| Stage | Field | Meaning |
|---|---|---|
| 1. observed signal | `raw` | noisy per-subcarrier amplitude frame (dB-like) |
| 2. processed measurement | `feature` | Doppler-energy proxy; moving humans dominate, static ones barely register |
| 3. inference | `inference` | presence belief `p_present ∈ [0,1]`, confidence, uncertainty |

Structural epistemics (enforced by `types.py`, validated by `contract.py`):
`inferred = True` and `confirmed = False` are not settable opinions - there
is **no code path** that can turn an inference into a confirmed fact.
CSI provides presence belief only; it cannot localize a person.

## Presence fusion (camera + CSI)

`assess_presence` fuses both modalities with equal weight:

```
fused = 0.5 · max(camera detection confidence) + 0.5 · csi.p_present
```

Design property: **neither modality alone can cross the INSPECT threshold**
(0.55). A perfect camera reading alone tops out at ~0.48; camera false
positives alone cap at ~0.39. Cross-modal agreement is required to act -
and even then the recorded event is a *probable presence* belief with
position, confidence, sources, and provenance, never a confirmed human.

## Health states (Layer 2)

Each sensor carries `HEALTHY / DEGRADED / STALE / FAILED / UNKNOWN`:

- fresh contract-valid data → HEALTHY
- valid but flagged by a consumer (fusion conflict, frozen-value
  plausibility check) → DEGRADED
- past freshness deadline → STALE
- contract violation or 3x-deadline silence → FAILED
- startup → UNKNOWN

Transitions are logged exactly once (never silently overwritten) and drive
both the safety layer and the observability stream.

## Provenance (Layer 3)

Every significant decision appends a `DecisionRecord`:
what was observed (`inputs`), at what time (`t`), which model processed it
(`model_version` - e.g. `ekf-gated-v0.1`), what uncertainty existed, which
component consumed it, and the decision with rationale. Veto records keep
both the proposed and effective command. This is what chaos runs audit.
