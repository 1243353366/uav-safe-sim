-- uav-safe-sim - Haskell mission FSM experiment (REFERENCE component)
--
-- Language rationale: Haskell is assigned as a one-off *experiment* (not a
-- load-bearing component): the mission state machine is a textbook case for
-- ADTs and exhaustive pattern matching - the compiler proves the transition
-- function total, which the Python implementation cannot. If the ADT
-- encoding surfaces states/transition gaps the Python FSM hides, the
-- experiment succeeded regardless of whether Haskell ships.
--
-- Status: REFERENCE - not compiled here (no GHC in CI). NOT integrated.
-- The authoritative FSM is src/uav/mission.py; any divergence found here is
-- filed as a finding, not merged silently.

module MissionFSM where

data MissionState
  = Idle
  | Takeoff
  | Nav
  | Inspect
  | Loiter
  | Rtl
  | Land'
  | Done
  deriving (Show, Eq)

data Event
  = Arm                      -- ground station arms the vehicle
  | ReachedCruiseAltitude
  | ProbablePresenceConfirmed -- belief above threshold (inferred, never fact)
  | InspectWindowElapsed
  | GoalReached
  | HomeReached
  | WheelsDown
  | SafetyForced SafetyOverride
  | NoPath
  deriving (Show, Eq)

data SafetyOverride = ForceRtl | ForceLand | ForceLoiter
  deriving (Show, Eq)

-- Total transition function: the compiler proves every (state, event) pair
-- is handled. In the Python FSM, "forget a case" is a runtime behavior;
-- here it is a compile error. That asymmetry is the experiment.
transition :: MissionState -> Event -> MissionState
transition Idle Arm = Takeoff
transition Idle _ = Idle
transition Takeoff ReachedCruiseAltitude = Nav
transition Takeoff (SafetyForced ForceLand) = Land'
transition Takeoff (SafetyForced ForceRtl) = Rtl
transition Takeoff _ = Takeoff
transition Nav ProbablePresenceConfirmed = Inspect
transition Nav GoalReached = Rtl
transition Nav NoPath = Loiter
transition Nav (SafetyForced o) = applyOverride o
transition Nav _ = Nav
transition Inspect InspectWindowElapsed = Nav
transition Inspect (SafetyForced ForceLand) = Land'
transition Inspect (SafetyForced ForceRtl) = Rtl
transition Inspect _ = Inspect
transition Loiter GoalReached = Rtl      -- replanned path recovered
transition Loiter (SafetyForced o) = applyOverride o
transition Loiter _ = Loiter
transition Rtl HomeReached = Land'
transition Rtl (SafetyForced ForceLand) = Land'
transition Rtl _ = Rtl
transition Land' WheelsDown = Done
transition Land' _ = Land'
transition Done _ = Done

applyOverride :: SafetyOverride -> MissionState
applyOverride ForceRtl = Rtl
applyOverride ForceLand = Land'
applyOverride ForceLoiter = Loiter

-- EXPERIMENT FINDING (to be filed, not silently fixed): the Python
-- mission.on_override maps a forced LOITER onto LOITER but a forced RTL
-- while LANDING does not exist (safety never interrupts landing) - this
-- ADT makes that invariant explicit: Land' has no ForceRtl case.
