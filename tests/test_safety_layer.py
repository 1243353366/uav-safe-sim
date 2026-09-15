"""Unit tests for the independent safety layer (the authority hierarchy).

These test the veto logic directly, with stubbed state, so the priority
order and the 'never interrupt a landing' rules are pinned independently of
the mission implementation.
"""
from uav.config import SimConfig
from uav.fusion import SensorHealth
from uav.safety import SafetyLayer
from uav.types import Command, PoseEstimate


class FakeLog:
    def __init__(self):
        self.vetoes = []

    def veto(self, t, reason):
        self.vetoes.append((t, reason))


class FakeDrone:
    def __init__(self, battery=100.0, x=5.0, y=5.0):
        self.battery = battery
        self.x, self.y, self.z = x, y, 3.0
        self.motors_on = True
        self.landed = False


def make_layer(size=60.0):
    cfg = SimConfig()
    return cfg, SafetyLayer(cfg, (5.0, 5.0), size)


def test_battery_land_outranks_everything():
    cfg, layer = make_layer()
    drone = FakeDrone(battery=5.0)
    pose = PoseEstimate(5, 5, 0, 0, 100.0, True)  # also uncertain + conflicting
    health = SensorHealth()
    health.conflict_active = True
    cmd, reason = layer.filter(Command("velocity", 4, 0), 10.0, drone, pose,
                               health, 0.0, 999.0, FakeLog())
    assert reason == "battery_land" and cmd.kind == "land"


def test_conflict_loiters_then_escalates_to_land():
    cfg, layer = make_layer()
    drone = FakeDrone()
    pose = PoseEstimate(5, 5, 0, 0, 0.3, False)
    health = SensorHealth()
    health.conflict_active = True
    health.conflict_t0 = 5.0
    _, r1 = layer.filter(Command("velocity", 4, 0), 6.0, drone, pose, health,
                         0.0, 0.0, FakeLog())
    assert r1 == "sensor_conflict"
    _, r2 = layer.filter(Command("velocity", 4, 0), 36.0, drone, pose, health,
                         0.0, 0.0, FakeLog())
    assert r2 == "conflict_escalate"  # loitering on conflict is time-limited


def test_comms_loss_forces_rtl_but_never_interrupts_landing():
    cfg, layer = make_layer()
    drone = FakeDrone()
    pose = PoseEstimate(30, 30, 0, 0, 0.3, False)
    health = SensorHealth()
    cmd, reason = layer.filter(Command("velocity", 4, 0), 30.0, drone, pose,
                               health, 0.0, 100.0, FakeLog())
    assert reason == "comms_loss" and cmd.kind == "rtl"
    # mission already landing: the failsafe must stand down
    cmd, reason = layer.filter(Command("land"), 30.0, drone, pose,
                               health, 0.0, 100.0, FakeLog())
    assert reason is None and cmd.kind == "land"


def test_localization_uncertainty_lands():
    cfg, layer = make_layer()
    drone = FakeDrone()
    pose = PoseEstimate(5, 5, 0, 0, 7.0, False)  # sigma > 6 m
    _, reason = layer.filter(Command("velocity", 4, 0), 10.0, drone, pose,
                             SensorHealth(), 0.0, 0.0, FakeLog())
    assert reason == "localization_lost"


def test_stale_depth_blinds_to_loiter():
    cfg, layer = make_layer()
    drone = FakeDrone()
    pose = PoseEstimate(5, 5, 0, 0, 0.3, False)
    _, reason = layer.filter(Command("velocity", 4, 0), 10.0, drone, pose,
                             SensorHealth(), 5.0, 0.0, FakeLog())
    assert reason == "perception_lost"


def test_geofence_breach_returns_home():
    cfg, layer = make_layer()
    drone = FakeDrone(x=0.2, y=30.0)  # inside margin
    pose = PoseEstimate(0.2, 30.0, 0, 0, 0.3, False)
    cmd, reason = layer.filter(Command("velocity", 4, 0), 10.0, drone, pose,
                               SensorHealth(), 0.0, 0.0, FakeLog())
    assert reason == "geofence_breach" and cmd.kind == "rtl"


def test_landed_aircraft_is_never_commanded():
    cfg, layer = make_layer()
    drone = FakeDrone(battery=2.0)  # even with a dead battery
    drone.landed = True
    drone.z = 0.0
    cmd, reason = layer.filter(Command("none"), 10.0, drone,
                               PoseEstimate(5, 5, 0, 0, 0.3, False),
                               SensorHealth(), float("inf"), 999.0, FakeLog())
    assert reason is None and cmd.kind == "none"


def test_compliant_commands_pass_untouched():
    cfg, layer = make_layer()
    drone = FakeDrone()
    pose = PoseEstimate(20, 20, 0, 0, 0.3, False)
    proposed = Command("velocity", 4.0, 0.0)
    cmd, reason = layer.filter(proposed, 10.0, drone, pose,
                               SensorHealth(), 0.0, 0.0, FakeLog())
    assert reason is None and cmd is proposed
