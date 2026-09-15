//! uav-safe-sim - Rust safety veto mirror (REFERENCE component - not yet compiled)
//!
//! Language rationale: Rust is assigned to the safety layer because the veto
//! path must never fail due to a memory-safety bug, and because an
//! independent implementation of the authority chain is itself a safety
//! feature: two implementations of the same priority table can be
//! cross-checked (see docs/08-research-protocol.md).
//!
//! This mirrors the Python SafetyLayer (src/uav/safety.py) priority order:
//! battery_land > battery_rth > localization_lost > conflict > comms_loss >
//! perception_lost > geofence. A behavioral conformance test must be run
//! against the Python implementation before this is promoted to verified.
//!
//! Status: REFERENCE - syntactically complete but NOT compiled here (no
//! rustc in CI yet). Do not assume compatibility with the Python core;
//! it is only established once the conformance tests pass.

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Health {
    Healthy,
    Degraded,
    Stale,
    Failed,
    Unknown,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Velocity { vx_mps: f64, vy_mps: f64 },
    Rtl { vx_mps: f64, vy_mps: f64 },
    Loiter,
    Land,
    Takeoff,
    None,
}

impl Command {
    pub fn kind(&self) -> &'static str {
        match self {
            Command::Velocity { .. } => "velocity",
            Command::Rtl { .. } => "rtl",
            Command::Loiter => "loiter",
            Command::Land => "land",
            Command::Takeoff => "takeoff",
            Command::None => "none",
        }
    }
    pub fn is_motion(&self) -> bool {
        matches!(self, Command::Velocity { .. } | Command::Rtl { .. })
    }
}

/// Safety thresholds - MUST match SimConfig (src/uav/config.py).
#[derive(Debug, Clone, Copy)]
pub struct SafetyConfig {
    pub battery_land_pct: f64,      // land immediately at/below
    pub battery_rth_pct: f64,       // return to home at/below
    pub max_uncertainty_m: f64,     // localization trust bound
    pub conflict_loiter_max_s: f64, // loiter window before escalating
    pub comms_loss_rtl_after_s: f64,
    pub depth_max_age_s: f64,
    pub geofence_margin_m: f64,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        SafetyConfig {
            battery_land_pct: 12.0,
            battery_rth_pct: 25.0,
            max_uncertainty_m: 6.0,
            conflict_loiter_max_s: 30.0,
            comms_loss_rtl_after_s: 10.0,
            depth_max_age_s: 1.0,
            geofence_margin_m: 1.0,
        }
    }
}

pub struct Veto {
    pub command: Command,
    pub reason: &'static str,
}

/// Independent veto authority. Pure function of observable state: same
/// inputs must always produce the same decision (determinism requirement).
pub fn filter(
    cfg: &SafetyConfig,
    proposed: &Command,
    t_s: f64,
    battery_pct: f64,
    landed: bool,
    pose_sigma_m: Option<f64>,
    conflict_active: bool,
    conflict_since_s: Option<f64>,
    comms_lost_for_s: f64,
    depth_age_s: f64,
    in_geofence: bool,
) -> Result<Option<Veto>, &'static str> {
    if !battery_pct.is_finite() {
        return Err("battery_pct must be finite"); // contracts: no NaN crosses
    }
    if landed {
        return Ok(None); // on the ground, all failsafes are moot
    }
    // 1. critical battery -> LAND
    if battery_pct <= cfg.battery_land_pct {
        return Ok(Some(Veto { command: Command::Land, reason: "battery_land" }));
    }
    // 2. reserve battery -> RTL (never interrupt an active landing)
    if battery_pct <= cfg.battery_rth_pct && !matches!(proposed, Command::Land | Command::None) {
        return Ok(Some(Veto { command: Command::Rtl { vx_mps: 0.0, vy_mps: 0.0 }, reason: "battery_rth" }));
    }
    // 3. localization untrustworthy -> LAND
    if let Some(sigma) = pose_sigma_m {
        if !sigma.is_finite() || sigma > cfg.max_uncertainty_m {
            return Ok(Some(Veto { command: Command::Land, reason: "localization_lost" }));
        }
    } else {
        return Ok(Some(Veto { command: Command::Land, reason: "localization_lost" }));
    }
    // 4. sensors contradicting -> LOITER, escalate to LAND if persistent
    if conflict_active {
        if let Some(t0) = conflict_since_s {
            if t_s - t0 > cfg.conflict_loiter_max_s {
                return Ok(Some(Veto { command: Command::Land, reason: "conflict_escalate" }));
            }
        }
        return Ok(Some(Veto { command: Command::Loiter, reason: "sensor_conflict" }));
    }
    // 5. lost link -> RTL (landing satisfies the failsafe intent)
    if comms_lost_for_s > cfg.comms_loss_rtl_after_s
        && !matches!(proposed, Command::Land | Command::None)
    {
        return Ok(Some(Veto { command: Command::Rtl { vx_mps: 0.0, vy_mps: 0.0 }, reason: "comms_loss" }));
    }
    // 6. blind while airborne -> LOITER
    if !depth_age_s.is_finite() || depth_age_s > cfg.depth_max_age_s {
        return Ok(Some(Veto { command: Command::Loiter, reason: "perception_lost" }));
    }
    // 7. geofence breach -> RTL
    if !in_geofence && !matches!(proposed, Command::Land | Command::None) {
        return Ok(Some(Veto { command: Command::Rtl { vx_mps: 0.0, vy_mps: 0.0 }, reason: "geofence_breach" }));
    }
    Ok(None) // compliant: pass through untouched
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> (SafetyConfig, f64, f64, bool, Option<f64>, bool, Option<f64>, f64, f64, bool) {
        (SafetyConfig::default(), 10.0, 100.0, false,
         Some(0.3), false, None, 0.0, 0.0, true)
    }

    #[test]
    fn battery_land_outranks_everything() {
        let (cfg, t, _, landed, sigma, conflict, cts, comms, depth, fence) = base();
        let v = filter(&cfg, &Command::Velocity { vx_mps: 4.0, vy_mps: 0.0 }, t,
                       5.0, landed, sigma, true, cts, comms, depth, fence)
            .unwrap().unwrap();
        assert_eq!(v.reason, "battery_land");
    }

    #[test]
    fn conflict_escalates_after_window() {
        let (cfg, _, battery, landed, sigma, _, _, comms, depth, fence) = base();
        let v1 = filter(&cfg, &Command::Velocity { vx_mps: 4.0, vy_mps: 0.0 },
                        6.0, battery, landed, sigma, true, Some(5.0), comms, depth, fence)
            .unwrap().unwrap();
        assert_eq!(v1.reason, "sensor_conflict");
        let v2 = filter(&cfg, &Command::Velocity { vx_mps: 4.0, vy_mps: 0.0 },
                        36.0, battery, landed, sigma, true, Some(5.0), comms, depth, fence)
            .unwrap().unwrap();
        assert_eq!(v2.reason, "conflict_escalate");
    }

    #[test]
    fn landing_is_never_interrupted() {
        let (cfg, t, battery, landed, sigma, conflict, cts, _, depth, fence) = base();
        let r = filter(&cfg, &Command::Land, t, battery, landed, sigma, conflict, cts,
                       999.0, depth, fence).unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn nan_inputs_are_rejected_not_fused() {
        let (cfg, t, _, landed, _, conflict, cts, comms, depth, fence) = base();
        assert!(filter(&cfg, &Command::Loiter, t, f64::NAN, landed,
                       Some(0.3), conflict, cts, comms, depth, fence).is_err());
    }
}
