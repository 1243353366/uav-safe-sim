# 06 - Open questions and uncertainties

Per the research protocol: when a design decision is uncertain, stop and
explain the uncertainty rather than silently inventing an assumption.

## Unresolved

1. **The 4th audited repository is unknown.** The research list included an
   item that was redacted (it contained a leaked credential, stored as
   `$GITHUB_TOKEN`). It is NOT covered by the license audit. It must be
   audited before any use.
2. **Ultralytics AGPL §13 in a robotics stack.** If a real detector ever
   ships behind our camera contract, does serving detections to a drone
   count as network distribution? This needs human legal review - it is
   flagged in `02-license-audit.md`, and the current design avoids the
   question by simulating detector statistics only.
3. **CSI physics is a proxy, not physics.** The simulated CSI feature is a
   motion-energy proxy. Real Wi-Fi sensing involves multipath, phase
   calibration, and hardware-specific artifacts. The *pipeline structure*
   (signal → measurement → inference) and epistemic rules transfer; the
   numbers do not.
4. **Conflict recovery is absent.** Once GPS is excluded due to conflict,
   there is no re-acceptance path (e.g. fix consistency checks after a
   cooldown). Current behavior degrades to LOITER/LAND - safe but
   pessimistic. A recovery policy is an open design question.
5. **`no_path` LOITER does not auto-recover** if replanning later reveals a
   route (mapping-only recoverability is deliberately minimal in the
   prototype; recovery is documented in the FSM ADT mirror).
6. **Altitude is a 2.5-D abstraction.** No vertical planning, no terrain
   following, no geofence ceiling. Fine for architecture research;
   irrelevant for a real stack.
7. **Humans are not obstacles.** The presence model is probabilistic and
   geometric avoidance ignores humans (they can be flown "through" in sim).
   A deployment would need a minimum-keepout behavior - and that interacts
   with the never-confirmed epistemics (you cannot keep out from a
   position you never confirm). This is a genuine research tension.
8. **Concurrency is deferred.** The prototype is single-threaded; the
   ROS 2 port introduces message passing and removes shared state, but
   race conditions/deadlock analysis is untested until then.
9. **Toolchains missing in CI:** rustc, Go, tsc, GHC. The four REFERENCE
   components cannot be promoted to VERIFIED without them.
10. **Battery/energy model is linear and uncalibrated.** Failsafe ordering
    works; the absolute percentages (25/12) are illustrative, not derived
    from real discharge curves or reserve margins.
11. **20-language outcome unknown.** The polyglot hypothesis is being
    evaluated, not asserted - see `07-language-assignments.md`.

## Deliberate simplifications (accepted for the prototype)

- 2-D world with axis-aligned walls; raycast-based depth.
- A* without kinodynamic constraints (avoidance layer handles dynamics).
- No wind, no motor delay, no ground effect.
- Deterministic seeds everywhere; chaos schedules are seeded too.
