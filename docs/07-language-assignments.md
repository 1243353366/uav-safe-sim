# 07 - Language assignments: hypothesis, not dogma

## The hypothesis (explicit)

This project does **not** claim that "20 languages makes it more robust."
It evaluates:

> Can an AI-assisted development agent coordinate a heterogeneous
> autonomous-system architecture spanning ~20 programming languages while
> maintaining correctness, observability, reproducibility, and fault
> tolerance - and does language specialization justify its interoperability
> and maintenance cost at all?

If it turns out horribly, that is a *result*. See `08-research-protocol.md`.

## Known experimental excess

The architecture intentionally contains more languages and subsystem
boundaries than would normally be recommended for a production UAV. This
is deliberate. The additional complexity is an experimental variable, not
an accidental design requirement. Some languages may ultimately be
removed. Failures are expected and are part of the experiment.

## The 20-language decision table

**Do not assume components are compatible because they have similar
names.** Compatibility is established only by executed conformance tests.

| # | Language | Assignment | Status | Rationale / tradeoff |
|---|---|---|---|---|
| 1 | Python | core stack: fusion, mapping, perception, planning, mission, safety, sim harness, tests | **VERIFIED (load-bearing)** | best research iteration speed; huge numerical/scientific stack; the executable reference implementation |
| 2 | C | sensor-interface ABI at the hardware boundary | **VERIFIED** | real sensor drivers ship as C ABIs; unit-named fields force the unit contract |
| 3 | C++ | flight-control reference (velocity tracking) | **VERIFIED** | PX4/ROS 2 control stacks are C++; same-family porting risk is lowest |
| 4 | Rust | safety veto mirror | REFERENCE | memory safety in the veto path + an independent second implementation of the authority chain is itself a safety feature |
| 5 | Go | telemetry network relay | REFERENCE | static binary fan-out service; goroutines fit a small stateless network edge; NOT in control loop |
| 6 | TypeScript | dashboard types | REFERENCE | compile-time schema drift detection for telemetry views |
| 7 | JavaScript | simulation glue / telemetry consumer | **VERIFIED** | zero-build tooling over the JSON provenance stream |
| 8 | Haskell | mission FSM experiment | REFERENCE (experiment) | ADT + exhaustive matching makes "forgot a transition" a compile error; already surfaced an invariant |
| 9 | SQL | telemetry/provenance store | **VERIFIED** | post-mortem reconstruction of decisions is a query problem |
| 10 | Java | mission orchestration | **NOT USED - deferred** | the FSM lives in Python; JVM orchestration adds a runtime boundary with no benefit at prototype scale. Revisit only if a real orchestration ecosystem (ROS 2 JVM) is adopted |
| 11 | Kotlin | telemetry | **NOT USED - rejected** | telemetry already has Go (relay) + SQL (store) + JS (consume); a third telemetry language duplicates cost with zero new capability |
| 12 | Swift | monitoring client | **NOT USED - deferred** | no Apple-platform client exists; adds a platform boundary nobody consumes. Documented as a future slot if a phone client is ever built |
| 13 | C# | simulator tooling | **NOT USED - rejected** | simulator tooling is Python (pytest, scenario runners); a parallel C# toolchain would double tooling maintenance |
| 14 | Lua | configuration scripting | **NOT USED - rejected** | config is a single typed dataclass (`config.py`) - one precedence hierarchy; embedding a scripting language reintroduces arbitrary config execution |
| 15 | Ruby | test harness | **NOT USED - rejected** | pytest already covers unit+scenario+chaos; a second harness fragments evidence |
| 16 | PHP | API layer | **NOT USED - rejected** | no web API exists; provenance is read via SQL/JSON directly |
| 17 | R | sensor-data analysis | **NOT USED - deferred** | analysis happens in tests (numpy); if offline statistics tooling grows beyond numpy, R or Python-only is an explicit decision point |
| 18 | MATLAB/Octave | signal-processing prototype | **NOT USED - rejected** | proprietary toolchain for an open CI loop; signal processing is numpy; MATLAB `.m` export from PX4 logs is noted as a future convenience |
| 19 | Julia | numerical modeling | **NOT USED - deferred** | the EKF is 4x4 - numpy is sufficient; Julia earns its place only if numerical modeling becomes heavy (optimal control, batch estimation) |
| 20 | Zig | systems component | **NOT USED - deferred** | C already owns the ABI boundary; adopting Zig would be an experiment *within* the experiment - kept as an explicitly documented slot |

### Score so far

- **VERIFIED**: 5 (Python, C, C++, JavaScript, SQL)
- **REFERENCE**: 4 (Rust, Go, TypeScript, Haskell)
- **Deferred**: 6 (Java, Swift, R, Julia, Zig - the slot exists, the need does not yet)
- **Rejected with rationale**: 5 (Kotlin, C#, Lua, Ruby, PHP, MATLAB)

Nine languages carry real code; eleven are documented decisions. That
*is* the experimental finding so far: an agent left to justify each
language assigns less than half of them, which is evidence against the
"20 languages by default" premise.

## Cross-language compatibility policy

1. The Python core is the **authoritative semantics** for every module.
2. Every boundary carries a contract (units, ranges, deadlines, versions,
   error states) - `src/uav/contract.py` is normative; the C ABI mirrors it
   and is conformance-tested.
3. A REFERENCE component is **not** compatible by inspection. Promotion to
   VERIFIED requires: toolchain in CI, conformance test against the
   Python implementation, and a chaos run over its failure handling.
4. Schema drift between language mirrors (Go struct vs Python dataclass)
   is an experimental observable, not something to hide.
