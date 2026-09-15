"""uav-safe-sim: research-grade, non-weaponized autonomous UAV simulation.

Architecture: sensors -> perception -> sensor fusion -> localization ->
world model -> path planning -> obstacle avoidance -> mission state machine
-> flight-control interface, with an independent safety layer holding final
veto authority. See README.md and docs/.
"""
__version__ = "0.1.0"
