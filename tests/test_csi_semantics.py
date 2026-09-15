"""Tests that the Wi-Fi CSI pipeline and human world model can never emit
a confirmed fact. These are the epistemic core of the research protocol:
inference must remain structurally distinguishable from confirmation.
"""
import random

from uav.perception import HumanWorldModel, assess_presence
from uav.sensors import CameraHumanDetector, WifiCsiSensor
from uav.types import CameraObs, CsiObs, Detection, PresenceAssessment
from uav.world import Drone, World


def _drone_at(x, y):
    d = Drone(x, y)
    d.yaw = 0.0
    return d


def test_csi_pipeline_stages_are_distinct():
    world = World()
    world.add_human(10.0, 10.0, moving=True)
    csi = WifiCsiSensor(rng=random.Random(1))
    obs = csi.read(0.0, _drone_at(10.0, 10.0), world)
    assert isinstance(obs, CsiObs)
    assert len(obs.raw) == 30          # stage 1: observed signal
    assert isinstance(obs.feature, float)  # stage 2: processed measurement
    assert obs.inference is not None      # stage 3: inference
    assert obs.inference.inferred is True
    assert obs.inference.confirmed is False


def test_csi_never_confirms_even_at_high_confidence():
    world = World()
    world.add_human(5.0, 5.0, moving=True)  # right on top of the drone
    csi = WifiCsiSensor(rng=random.Random(1))
    drone = _drone_at(5.0, 5.0)
    for i in range(50):
        obs = csi.read(i * 0.1, drone, world)
        assert obs.inference.confirmed is False
        assert obs.inference.inferred is True
        assert obs.inference.present is True or obs.inference.present is False


def test_csi_confidence_and_uncertainty_are_consistent():
    world = World()
    world.add_human(5.0, 5.0, moving=True)
    csi = WifiCsiSensor(rng=random.Random(1))
    obs = csi.read(0.0, _drone_at(5.0, 5.0), world)
    inf = obs.inference
    assert 0.0 <= inf.confidence <= 1.0
    assert abs(inf.confidence + inf.uncertainty - 1.0) < 1e-9


def test_human_world_model_has_no_confirmation_path():
    m = HumanWorldModel()
    m.record(0.0, PresenceAssessment(True, 0.99, 0.01, ["camera", "wifi_csi"]))
    assert m.confirmed_humans == []
    assert m.probable_humans[0]["confirmed"] is False
    assert m.probable_humans[0]["inferred"] is True


def test_fused_presence_never_confirms():
    cam = CameraObs(t=0.0, detections=[Detection(
        t=0.0, x=5.0, y=5.0, confidence=0.95, source="camera")])
    csi = WifiCsiSensor(rng=random.Random(1)).read(
        0.0, _drone_at(5.0, 5.0), World())  # empty world: absent
    a = assess_presence(cam, csi)
    assert a.confirmed is False and a.inferred is True
    assert a.assessed_present is (a.confidence > 0.5)


def test_camera_detections_marked_as_inferences():
    world = World()
    world.add_human(10.0, 10.0, moving=True)
    cam = CameraHumanDetector(p_detect=1.0, p_false=0.0, rng=random.Random(1))
    obs = cam.read(0.0, _drone_at(5.0, 10.0), world)
    assert len(obs.detections) >= 1
    for det in obs.detections:
        assert det.inferred is True and det.confirmed is False
