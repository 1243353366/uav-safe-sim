-- uav-safe-sim - SQL telemetry/event database (verified component)
--
-- Language rationale: SQL is assigned to the telemetry store because the
-- provenance requirement (reconstruct every decision) is a query problem.
-- Post-mortem analysis ("what did the system believe at t=...?") is
-- expressible in SQL without writing bespoke analysis code.
--
-- Status: VERIFIED - schema executes and round-trips records via sqlite3
-- (the CI check uses Python's sqlite3 module to execute this file).

-- One row per mission run.
CREATE TABLE IF NOT EXISTS missions (
    id            INTEGER PRIMARY KEY,
    started_at    REAL    NOT NULL,          -- simulation clock, s
    finished_at   REAL,
    final_state   TEXT    NOT NULL,          -- FSM terminal state
    goal_x_m      REAL,                      -- SI units, ENU world frame
    goal_y_m      REAL,
    seed          INTEGER NOT NULL,          -- reproducibility requirement
    chaos_seed    INTEGER                    -- NULL when nominal run
);

-- Raw pose/battery stream (one row per tick; the sim logs this already).
CREATE TABLE IF NOT EXISTS telemetry (
    id            INTEGER PRIMARY KEY,
    mission_id    INTEGER NOT NULL REFERENCES missions(id),
    t_s           REAL    NOT NULL,
    x_m           REAL, y_m REAL, z_m REAL,
    battery_pct   REAL,
    pose_sigma_m  REAL,                      -- localization uncertainty
    mission_state TEXT
);
CREATE INDEX IF NOT EXISTS idx_telemetry_t ON telemetry(mission_id, t_s);

-- Provenance: every significant decision, reconstructable.
CREATE TABLE IF NOT EXISTS decisions (
    id            INTEGER PRIMARY KEY,
    mission_id    INTEGER NOT NULL REFERENCES missions(id),
    t_s           REAL    NOT NULL,          -- what time was it?
    component     TEXT    NOT NULL,          -- who decided
    decision      TEXT    NOT NULL,          -- what was decided
    inputs        TEXT,                      -- what it observed (JSON)
    uncertainty   REAL,                      -- what uncertainty existed
    model_version TEXT,                      -- which model processed it
    consumer      TEXT,                      -- which component consumed it
    rationale     TEXT                       -- why
);
CREATE INDEX IF NOT EXISTS idx_decisions_t ON decisions(mission_id, t_s);

-- Graded health-state transitions (HEALTHY/DEGRADED/STALE/FAILED/UNKNOWN).
CREATE TABLE IF NOT EXISTS health_transitions (
    id            INTEGER PRIMARY KEY,
    mission_id    INTEGER NOT NULL REFERENCES missions(id),
    t_s           REAL    NOT NULL,
    sensor        TEXT    NOT NULL,
    from_state    TEXT    NOT NULL,
    to_state      TEXT    NOT NULL,
    reason        TEXT
);

-- Contract rejections at subsystem boundaries.
CREATE TABLE IF NOT EXISTS contract_rejections (
    id            INTEGER PRIMARY KEY,
    mission_id    INTEGER NOT NULL REFERENCES missions(id),
    t_s           REAL    NOT NULL,
    sensor        TEXT    NOT NULL,
    reason        TEXT    NOT NULL,          -- e.g. nan_or_out_of_range
    dropped       INTEGER NOT NULL DEFAULT 1 -- dropped, never partially used
);

-- Example post-mortem query (see docs/08-research-protocol.md):
--   What did the system believe, and with what uncertainty, in the 5 s
--   before the safety layer forced a return to home?
-- SELECT d.t_s, d.component, d.decision, d.uncertainty, d.model_version
-- FROM decisions d
-- WHERE d.mission_id = :id
--   AND d.t_s BETWEEN (:rtl_t - 5.0) AND :rtl_t
-- ORDER BY d.t_s;
