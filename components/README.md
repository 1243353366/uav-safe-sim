# Polyglot components - status and compatibility policy

**Do not assume these components are compatible merely because they have
similar names.** Similar names are the *hypothesis*; conformance tests are
the only proof. Compatibility between two components is established by an
executed conformance check (same inputs -> same outputs, same contracts,
same units, same semantics), never by inspection.

## Status

| Language | Component | Status | Verified how |
|---|---|---|---|
| C | `c/sensor_abi.h` + conformance check | **VERIFIED** | `gcc -std=c11 -Wall -Wextra -pedantic` compile + run, asserts pass in CI |
| C++ | `cpp/flight_controller.cpp` | **VERIFIED** | `g++ -std=c++17 -Wall -Wextra -pedantic` compile + run, asserts pass in CI |
| Python | core stack (`src/uav/`) | **VERIFIED** | 59 pytest scenario/unit tests |
| JavaScript | `javascript/telemetry_printer.js` | **VERIFIED** | `node --check` + executed against sample provenance JSONL |
| SQL | `sql/telemetry_schema.sql` | **VERIFIED** | executed via sqlite3 (Python driver), schema + round-trip |
| Rust | `rust/safety_veto.rs` | REFERENCE | no rustc in CI yet; conformance tests written, awaiting toolchain |
| Go | `go/telemetry_relay.go` | REFERENCE | no Go toolchain in CI yet |
| TypeScript | `typescript/dashboard-types.ts` | REFERENCE | no tsc in CI yet |
| Haskell | `haskell/MissionFSM.hs` | REFERENCE (experiment) | no GHC in CI yet; the ADT encoding already surfaced one invariant (see file) |

REFERENCE means: syntactically complete, deliberately written, **not
executed in CI**, and explicitly *not* assumed compatible with the Python
core. Promoting a component from REFERENCE to VERIFIED requires:

1. A toolchain in CI.
2. A conformance test against the authoritative Python implementation.
3. A chaos run exercising the component's failure handling.

See `docs/07-language-assignments.md` for the full 20-language decision
table, including the languages deliberately **not** used and why.
