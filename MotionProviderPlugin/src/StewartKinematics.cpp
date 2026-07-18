#include "StewartKinematics.h"
#include "StewartConfig.h"
#include <cmath>

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

struct Vec3 { double x, y, z; };

// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians.
Vec3 rotate(double roll, double pitch, double yaw, Vec3 p) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);
    Vec3 a{ p.x,               cr * p.y - sr * p.z,  sr * p.y + cr * p.z };
    Vec3 b{ cp * a.x + sp * a.z, a.y,               -sp * a.x + cp * a.z };
    Vec3 c{ cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z };
    return c;
}
}  // namespace

double StewartKinematics::homeHeight() {
    using namespace stewart;
    const double phi = kPhi[0] * kDeg2Rad;
    const double psi = kPsi[0] * kDeg2Rad;
    const double beta = kBeta[0] * kDeg2Rad;
    const double bx = kRb * std::cos(phi);
    const double by = kRb * std::sin(phi);
    const double hx = bx + kHorn * std::cos(beta);  // horn tip at theta=0
    const double hy = by + kHorn * std::sin(beta);
    const double ax = kRp * std::cos(psi);
    const double ay = kRp * std::sin(psi);
    const double dx = ax - hx, dy = ay - hy;
    const double h2 = kRod * kRod - (dx * dx + dy * dy);
    return h2 > 0.0 ? std::sqrt(h2) : 0.0;
}

SolveResult StewartKinematics::solve(const Pose& pose) {
    using namespace stewart;
    SolveResult out{};
    out.allReachable = true;

    const double z0 = homeHeight();
    const Vec3 origin{ pose.surge, pose.sway, z0 + pose.heave };
    const double roll = pose.roll * kDeg2Rad;
    const double pitch = pose.pitch * kDeg2Rad;
    const double yaw = pose.yaw * kDeg2Rad;

    for (int i = 0; i < kLegs; ++i) {
        const double phi = kPhi[i] * kDeg2Rad;
        const double psi = kPsi[i] * kDeg2Rad;
        const double beta = kBeta[i] * kDeg2Rad;

        const Vec3 pl{ kRp * std::cos(psi), kRp * std::sin(psi), 0.0 };
        const Vec3 r = rotate(roll, pitch, yaw, pl);
        const Vec3 q{ origin.x + r.x, origin.y + r.y, origin.z + r.z };
        const Vec3 B{ kRb * std::cos(phi), kRb * std::sin(phi), 0.0 };
        const Vec3 l{ q.x - B.x, q.y - B.y, q.z - B.z };

        const double L2 = (l.x * l.x + l.y * l.y + l.z * l.z)
                          - (kRod * kRod - kHorn * kHorn);
        const double M = 2.0 * kHorn * l.z;
        const double N = 2.0 * kHorn * (l.x * std::cos(beta) + l.y * std::sin(beta));
        const double denom = std::sqrt(M * M + N * N);

        LegResult lr;
        if (denom < 1e-9) {
            lr.reachable = false;
            lr.angleDeg = 0.0;
        } else {
            double ratio = L2 / denom;
            if (ratio < -1.0 || ratio > 1.0) {
                lr.reachable = false;
                ratio = ratio < 0.0 ? -1.0 : 1.0;   // clamp to envelope edge
            }
            lr.angleDeg = (std::asin(ratio) - std::atan2(N, M)) / kDeg2Rad;
        }
        if (!lr.reachable) out.allReachable = false;
        out.legs[i] = lr;

        double demand = kDemandHome + (lr.angleDeg / kAngleAtFullScale) * kDemandHome;
        if (demand < 0.0) demand = 0.0;
        if (demand > kDemandMax) demand = kDemandMax;
        out.setpoints[kBff[i] - 1] = static_cast<uint16_t>(std::lround(demand));
    }
    return out;
}
