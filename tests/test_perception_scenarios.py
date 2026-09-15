"""Scenario tests: human detection, false positives, and false negatives."""
from uav.config import SensorSetup
from uav.sim import Simulation
from uav.world import World


def test_human_present_triggers_probable_presence_inspect():
    world = World()
    world.add_human(16.0, 16.0, moving=True, seed=3)  # on the route, near launch
    setup = SensorSetup(cam_p_detect=0.95, cam_p_false=0.0, seed=11)
    sim = Simulation(world=world, goal=(52.0, 52.0), setup=setup)
    log = sim.run(110.0)

    events = sim.events_of("probable_presence")
    assert len(events) >= 1, "expected at least one probable-presence assessment"
    data = events[0][2]
    assert data["inferred"] is True and data["confirmed"] is False
    assert data["confidence"] > 0.55
    assert "camera" in data["sources"] and "wifi_csi" in data["sources"]
    assert sim.mission.state == "DONE"  # inspection completes, mission resumes


def test_camera_false_positives_do_not_confirm_or_derail():
    # no humans exist at all; the camera fires false positives constantly
    setup = SensorSetup(cam_p_detect=0.0, cam_p_false=0.5, seed=5)
    sim = Simulation(goal=(52.0, 52.0), setup=setup)
    log = sim.run(80.0)

    assert sim.log.cam_detections > 0  # false positives actually occurred
    assert len(sim.events_of("probable_presence")) == 0  # never confirmed
    for c in log.csi_inferences:
        assert c["confirmed"] is False
    assert sim.mission.state == "DONE"  # FPs never derail the mission


def test_camera_false_negative_keeps_presence_uncertain():
    # a real (static) human exists; the camera is blind to it (p_detect = 0)
    world = World()
    world.add_human(20.0, 20.0, moving=False)
    setup = SensorSetup(cam_p_detect=0.0, cam_p_false=0.0, seed=5)
    sim = Simulation(world=world, goal=(52.0, 52.0), setup=setup)
    log = sim.run(80.0)

    assert sim.log.cam_detections == 0  # camera detected nothing
    assert len(sim.events_of("probable_presence")) == 0  # no fabricated facts
    # CSI may weakly sense the static human - but only ever as an inference
    for c in log.csi_inferences:
        assert c["inferred"] is True and c["confirmed"] is False
    assert sim.mission.state == "DONE"


def test_csi_alone_cannot_trigger_inspect():
    # camera disabled entirely; CSI sees a moving human strongly
    world = World()
    world.add_human(15.0, 15.0, moving=True, seed=3)  # on the route, near launch
    setup = SensorSetup(cam_mode="drop", seed=5)
    sim = Simulation(world=world, goal=(52.0, 52.0), setup=setup)
    log = sim.run(80.0)

    assert len(sim.events_of("probable_presence")) == 0  # needs both modalities
    strong = [c for c in log.csi_inferences if c["p_present"] > 0.8]
    assert strong, "CSI should at least produce strong presence beliefs"
    for c in strong:
        assert c["confirmed"] is False  # strong belief, still never confirmed
