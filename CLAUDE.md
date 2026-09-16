# UAV Safe Sim — Claude Code Project Context

## What this is
An **experimental polyglot autonomous systems platform** — a research project,
NOT a drone/weapon project. The UAV is the test environment for studying
whether an AI-assisted development agent can build and maintain a complex
distributed autonomous-systems architecture (~20 languages) while keeping
correctness, observability, reproducibility, and fault tolerance.

Research question lives in README.md; if the answer is "no, and here is
where it breaks", that is a valid result.

## Repo layout
- `src/uav/` — Python simulation core (20 modules: perception, fusion,
  planning, localization, mapping, safety, chaos, contracts, provenance…)
- `components/` — polyglot satellite components, one dir per language:
  `c` (sensor ABI + conformance), `cpp` (flight controller), `go` (telemetry
  relay), `haskell` (mission FSM), `javascript` (telemetry printer),
  `rust` (safety veto — the independent safety layer), `sql` (telemetry
  schema), `typescript` (dashboard types)
- `tests/` — pytest suite (59 tests, all must pass)
- `docs/` — numbered docs; 01 is architecture, 04 is failure modes,
  09 is the experiment log
- `docker/simulation/` — sim bring-up stack
- `ros2-interface/` — ROS2 launch/package plumbing
- `public/` + `wrangler.jsonc` — static dashboard, deployed as an
  assets-only Cloudflare Worker
- `.github/workflows/ci.yml` — CI: pip install + pytest on every push

## Commands
```bash
pip install -r requirements.txt       # numpy>=1.23, pytest>=7.0
python -m pytest tests -q             # full test suite (59 tests)
./scripts/run_tests.sh                # same, via wrapper
npx wrangler deploy                    # publish public/ to
                                      # uav-safe-sim.<account>.workers.dev
npx wrangler deploy --dry-run          # validate deploy without publishing
```

## Rules of engagement
1. **Never weaken a test to make it pass.** If a test fails, the finding is
   the result — fix the code or document the breakage in docs/09.
2. The Rust `safety_veto` component is the independent safety layer. It may
   veto the planner. Do not route around it.
3. Every experiment goes in `docs/09-experiment-log.md` — what ran, what
   surprised you.
4. Apache-2.0. Keep THIRD_PARTY_NOTICES.md accurate when adding dependencies.
5. Pushes to `main` trigger GitHub Actions CI (pytest) and the Cloudflare
   Workers build (wrangler deploy). Both must stay green.

## Current state (2026-09-16)
- CI green on main (run #1), 59/59 tests passing on Python 3.13
- Dashboard deployed via assets-only Worker; deploy command `npx wrangler deploy`
- License audit (docs/02) and language assignments (docs/07) are current
