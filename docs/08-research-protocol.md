# 08 - Research protocol

## Research question

> Can an AI-assisted development agent construct and maintain a complex
> distributed autonomous-system architecture across ~20 programming
> languages while maintaining correctness, observability, reproducibility,
> and fault tolerance?

The UAV is the test environment. The architecture - and the agent that
built it - are the study objects.

## Experimental design rationale

- The 20-language matrix is **intentionally excessive** (see "Known
  experimental excess" in `07-language-assignments.md`).
- The architecture is designed to test specialization versus complexity.
- Some languages may ultimately be removed.
- Failures are expected and are part of the experiment.
- The AI agent is being evaluated as much as the software itself.

## Four layers above the failure-mode table

### Layer 1: Contracts

Every subsystem has an explicit contract: input/output types, units,
timestamps, validity ranges, error states, timeouts, versions
(`src/uav/contract.py`, mirrored in `components/c/sensor_abi.h`). A
violating observation is dropped and recorded - never "mostly used".

### Layer 2: Health state (not binary)

`HEALTHY / DEGRADED / STALE / FAILED / UNKNOWN` per sensor
(`src/uav/health.py`). "NO DATA" and "NO DETECTION" are different facts
and are logged differently. Transitions are reported exactly once.

### Layer 3: Observability / provenance

Every significant decision appends a reconstructable record: what was
observed, at what time, which model version processed it, what
uncertainty existed, which component consumed it, what was decided, and
why (`src/uav/provenance.py`, stored per `components/sql/schema`).

### Layer 4: Chaos testing

Once the basic simulation works, deliberately break it: drop messages,
corrupt measurements (NaN), delay timestamps, replay duplicates, inject
contradictions, kill the ground link, restart the mission mid-flight.
Then ask:

> Did the system DETECT the problem, CONTAIN it, RECOVER or terminate
> safely, and LOG enough evidence to understand what happened?

That is the project's definition of robustness - not "it flew around the
simulator 100 times". A clean nominal run proves nothing.

**Failure injection is mandatory before any subsystem may be declared
complete.** Every subsystem in this repo has at least one adversarial
scenario test in `tests/` before it is called done.

## Success criteria: learning, not completion

"Success" is measurable evidence about which architectural approaches
succeed or fail, in four categories - all equally useful:

- **Success**: e.g. contracts eliminated a whole class of cross-language
  unit/range bugs at the boundary (C conformance passes with zero
  divergence from the Python validators).
- **Failure**: e.g. a language boundary that created unacceptable
  complexity (candidates: the Go struct drift risk vs Python provenance
  schema; measure it).
- **Unexpected**: e.g. the "slow" Python prototype turns out to be the
  only fully-verified implementation - the "fast" languages shipped
  unverified code that cannot be trusted yet.
- **AI failure**: agent mistakes are data. Recorded examples so far:
  (1) the agent's first safety filter let the comms-loss failsafe
  interrupt an active landing, creating an infinite RTL/LAND flap -
  caught by scenario tests, fixed, and pinned by unit tests; (2) the
  agent's stuck-sensor plausibility check compared per-tick motion that
  could never exceed its own threshold at cruise speed - caught by the
  failure-mode test, fixed by accumulating motion since the last
  *different* scan. Both were caught because the protocol demands
  adversarial tests before completion - which is the point.

## Scope (civilian, non-weaponized)

The prototype is restricted to **simulation and civilian/research
applications** including navigation, mapping, inspection, environmental
sensing, and search-and-rescue scenarios. Weaponization and autonomous
engagement are outside the project's implementation scope: no weapons,
weapon payloads, autonomous target selection for attack, firing logic,
or explosive payloads exist in this codebase, and the architecture is not
designed to accommodate them.
