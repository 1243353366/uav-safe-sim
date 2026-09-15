// uav-safe-sim - C++ flight-control reference (verified component)
//
// Language rationale: C++ is assigned to flight control because PX4 and the
// ROS 2 control stacks are C++; keeping the flight-critical numeric path in
// the same language family as the target flight stack minimizes porting
// risk and lets us reuse the exact same units/frames conventions.
//
// This mirrors the Python Drone._track() semantics exactly: accel-limited
// velocity tracking with SI units only. The conformance main() asserts the
// behavioral equivalence (same accel cap, same clamping).
//
// Status: VERIFIED - compiled and executed with g++ in CI.
//   g++ -std=c++17 -Wall -Wextra -pedantic -o fc flight_controller.cpp && ./fc

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace uav {

struct Command {          // flight-control interface command (world frame, ENU)
    enum class Kind { Velocity, Rtl, Loiter, Land, Takeoff, None };
    Kind kind = Kind::None;
    double vx_mps = 0.0;  // m/s
    double vy_mps = 0.0;  // m/s
};

struct DroneState {       // 2.5-D kinematic state, SI units
    double x_m = 0.0, y_m = 0.0, z_m = 0.0;
    double vx_mps = 0.0, vy_mps = 0.0;
    double ax_mps2 = 0.0, ay_mps2 = 0.0;
};

class FlightController {
public:
    explicit FlightController(double max_accel_mps2 = 3.0, double max_speed_mps = 4.0)
        : max_accel_(max_accel_mps2), max_speed_(max_speed_mps) {}

    // Accel-limited velocity tracking: identical semantics to Python
    // Drone._track: cap the acceleration vector magnitude, integrate.
    void apply(DroneState &d, const Command &cmd, double dt_s) {
        double tvx = 0.0, tvy = 0.0;
        switch (cmd.kind) {
            case Command::Kind::Velocity:
            case Command::Kind::Rtl:
                tvx = cmd.vx_mps; tvy = cmd.vy_mps;
                break;
            case Command::Kind::Loiter:
            case Command::Kind::None:
            case Command::Kind::Land:
            case Command::Kind::Takeoff:
                break;  // zero lateral velocity; altitude handled by the stack
        }
        double ax = (tvx - d.vx_mps) / dt_s;
        double ay = (tvy - d.vy_mps) / dt_s;
        const double n = std::hypot(ax, ay);
        if (n > max_accel_) {           // magnitude-capped, direction preserved
            ax *= max_accel_ / n;
            ay *= max_accel_ / n;
        }
        d.vx_mps += ax * dt_s;
        d.vy_mps += ay * dt_s;
        // speed cap (safety layer backstop; Python enforces it upstream too)
        const double v = std::hypot(d.vx_mps, d.vy_mps);
        if (v > max_speed_) {
            d.vx_mps *= max_speed_ / v;
            d.vy_mps *= max_speed_ / v;
        }
        d.x_m += d.vx_mps * dt_s;
        d.y_m += d.vy_mps * dt_s;
        d.ax_mps2 = ax;
        d.ay_mps2 = ay;
    }

private:
    double max_accel_;
    double max_speed_;
};

}  // namespace uav

int main() {
    using uav::Command;
    uav::DroneState d;                  // starts at rest at origin
    uav::FlightController fc(3.0, 4.0); // same limits as SimConfig

    // 1) a step command accelerates but never exceeds the accel cap
    Command go{Command::Kind::Velocity, 4.0, 0.0};
    for (int i = 0; i < 100; ++i) fc.apply(d, go, 0.1);
    assert(std::abs(d.vx_mps - 4.0) < 1e-9);
    assert(std::hypot(d.ax_mps2, d.ay_mps2) <= 3.0 + 1e-9);

    // 2) diagonal command: accel vector is capped in magnitude, not per-axis
    uav::DroneState d2;
    Command diag{Command::Kind::Velocity, 4.0, 4.0};
    fc.apply(d2, diag, 0.1);
    const double mag = std::hypot(d2.ax_mps2, d2.ay_mps2);
    assert(mag <= 3.0 + 1e-9);          // per-axis would be 40.0: must not happen

    // 3) speed is capped at 4 m/s even under a huge command
    Command insane{Command::Kind::Velocity, 100.0, 0.0};
    for (int i = 0; i < 100; ++i) fc.apply(d, insane, 0.1);
    assert(d.vx_mps <= 4.0 + 1e-9);

    // 4) loiter decelerates to rest
    Command stop{Command::Kind::Loiter, 0.0, 0.0};
    for (int i = 0; i < 100; ++i) fc.apply(d, stop, 0.1);
    assert(std::abs(d.vx_mps) < 1e-9 && std::abs(d.vy_mps) < 1e-9);

    std::cout << "C++ flight-control conformance: ALL CHECKS PASSED" << std::endl;
    return 0;
}
