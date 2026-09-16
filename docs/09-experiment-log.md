# 09 - Experiment log: what ran as expected, what surprised us

Recorded 2026-09-15, per the four success categories defined in
`08-research-protocol.md` (success / failure / unexpected / AI failure).
This is the evidence the platform exists to produce: measurable findings
about which architectural approaches succeeded or failed - including the
AI agent's own mistakes, which are data, not embarrassment.

## What ran as expected: the degradation architecture

Every injected failure mode ended in a *safer* state, never a more
aggressive one. Across the full test matrix (59 tests, multiple chaos
seeds) there were **zero collisions**, including chaos runs against an
obstacle world with walls and gaps:

- stale GPS → discarded, uncertainty grows, failsafe ladder engages
- frozen depth sensor → plausibility check detects, sensor distrusted
- lying GPS (10x true noise) → chi-square gate rejects, conflict declared
- contradictory GPS (+35 m bias) → gate rejects, source excluded, LOITER
- dead comms → RTL failsafe after 10 s
- dying battery → RTL at 25%, LAND at 12%
- chaos-corrupted NaN data → 100% caught at the contracts boundary,
  never fused (proven per-run in `test_chaos.py::corrupt_nan`)
- the epistemic rule held *structurally*: no code path can turn a CSI
  inference into a confirmed fact

The failsafe ladder (LOITER → RTL → LAND) never produced a transition
that increases aggressiveness. That is the core claim of the safety
architecture, and it survived chaos testing.

## What surprised us (including the agent's own bugs)

### 1. The failsafes barely fired in nominal runs (test design flaw)

The nominal mission completes in ~39 s, so early comms-loss and
low-battery tests passed *vacuously*: the failsafes never intervened
before DONE. Passing for the wrong reason is not evidence. The tests were
tightened (shorter `comms_loss_rtl_after`, lower battery reserves) until
the failsafes actually fired mid-flight and their full behavior could be
observed. Lesson: a test that cannot fail cannot detect anything.

### 2. The infinite-hover bug: failsafes vetoing the landing (AI failure)

The comms-loss RTL override kept replacing the mission's final `land`
command - indefinitely. The drone hovered over its own landing pad
forever because two safety mechanisms were each "protecting" the
aircraft from touching down. The battery-RTH override had the
*identical* bug. Fix: a failsafe whose goal is already being achieved
must stand down (never interrupt `land`), plus a defense-in-depth guard
in the drone body (no lateral motion while landed). Both fixes are now
pinned by unit tests (`test_safety_layer.py`,
`test_failsafes.py::comms_loss`). Lesson: independent safety mechanisms
compose badly unless their *interactions* are tested, not just each one
alone.

### 3. A detection check that could not detect (AI failure)

The first frozen-depth plausibility check required 0.5 m of motion
between ticks, but cruise speed is ~4 m/s at 0.1 s ticks = 0.4 m/tick.
The check was mathematically incapable of firing at cruise. Caught only
because failure injection is mandatory before completion; fixed by
accumulating motion since the last *different* scan
(`world.py` stuck-scan logic). Lesson: thresholds need arithmetic checks
against the actual operating envelope, not intuition.

### 4. The system was more conservative than designed (test wrong, code right)

The GPS-degradation scenario expected the 30-second conflict-loiter
escalation to run its course. Instead `localization_lost` fired at ~7 s:
once GPS is excluded, dead-reckoning covariance grows fast, and honest
uncertainty crosses the 6 m trust bound almost immediately. The original
test asserted the wrong expectation; the implementation was correct.
Lesson (a pleasant one): honest uncertainty estimation produced *earlier*
failsafes than the designed policy - conservatism emerging from the
math rather than from a rule.

### 5. The 20-language hypothesis argued against itself (finding)

Forced to justify each of the 20 languages individually, the agent
assigned real code to 9 and documented deferrals/rejections with
rationale for the other 11 (`07-language-assignments.md`). The rejections
(Kotlin, C#, Lua, Ruby, PHP, MATLAB...) were better-argued than most of
the assignments. This is preliminary evidence against the "20 languages
by default" premise: specialization is easy to *claim* and hard to
*justify*. A follow-up finding: the Haskell reference component - never
compiled - surfaced a real invariant (landing must never be interruptible
by a failsafe) purely from the ADT's exhaustive pattern matching, before
any toolchain existed. Language choice changed the *reasoning*, not just
the runtime.

## Summary scorecard

| Category | Count | Representative items |
|---|---|---|
| Success (architecture) | degradation ladder held under all chaos; contracts caught 100% NaN; structural epistemics | every failure-mode test |
| Failure (complexity) | failsafe *interactions* produced an infinite hover; vacuous tests passed while proving nothing | fixes #1, #2 |
| Unexpected | honest uncertainty beat the designed policy; an uncompiled language found a real invariant | #4, #5 |
| AI failure | self-interrupting failsafes; impossible detection threshold; wrong test expectations | #1-#4 |

The meta-finding, which is the point of the protocol: **the agent's own
conceptual errors were caught by the agent's adversarial tests, because
the protocol made failure injection mandatory before declaring anything
complete.** Without chaos testing and the four-layer requirements, all
three AI-failure bugs would have shipped as "working" code.
