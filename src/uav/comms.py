"""Communications link model (uplink/downlink to a ground station).

The drone is autonomous: losing the link does not stop onboard processing,
but a link lost longer than `comms_loss_rtl_after` triggers a failsafe RTL
in the safety layer. This models the standard RC-loss failsafe behavior of
real flight stacks.
"""
from typing import Optional


class CommsLink:
    def __init__(self, mode="ok", loss_at: Optional[float] = None):
        self.mode = mode
        self.loss_at = loss_at

    def lost_for(self, t: float) -> float:
        if self.mode != "lost":
            return 0.0
        start = self.loss_at if self.loss_at is not None else 0.0
        return max(0.0, t - start)
