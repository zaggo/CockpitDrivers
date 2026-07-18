#include "StewartKinematics.h"
#include <cmath>

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
struct Vec3 { double x, y, z; };

Vec3 rotate(double roll, double pitch, double yaw, Vec3 p) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);
    Vec3 a{ p.x,                cr * p.y - sr * p.z,  sr * p.y + cr * p.z };
    Vec3 b{ cp * a.x + sp * a.z, a.y,                -sp * a.x + cp * a.z };
    Vec3 c{ cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z };
    return c;
}
}  // namespace

StewartKinematics::StewartKinematics(const StewartGeometry& geo) : geo_(geo) {
    // Home height from leg-0 horizontal closure (all legs congruent).
    const double phi = geo_.phiDeg[0] * kDeg2Rad;
    const double psi = geo_.psiDeg[0] * kDeg2Rad;
    const double beta = geo_.betaDeg[0] * kDeg2Rad;
    const double hx = geo_.baseRadius * std::cos(phi) + geo_.hornLength * std::cos(beta);
    const double hy = geo_.baseRadius * std::sin(phi) + geo_.hornLength * std::sin(beta);
    const double ax = geo_.platformRadius * std::cos(psi);
    const double ay = geo_.platformRadius * std::sin(psi);
    const double dx = ax - hx, dy = ay - hy;
    const double h2 = geo_.rodLength * geo_.rodLength - (dx * dx + dy * dy);
    z0_ = h2 > 0.0 ? std::sqrt(h2) : 0.0;
}

SolveResult StewartKinematics::solve(const Pose& pose) const {
    SolveResult out{};
    out.allReachable = true;

    const Vec3 origin{ pose.surge, pose.sway, z0_ + pose.heave };
    const double roll = pose.roll * kDeg2Rad;
    const double pitch = pose.pitch * kDeg2Rad;
    const double yaw = pose.yaw * kDeg2Rad;
    const double s2ma2 = geo_.rodLength * geo_.rodLength - geo_.hornLength * geo_.hornLength;

    for (int i = 0; i < 6; ++i) {
        const double phi = geo_.phiDeg[i] * kDeg2Rad;
        const double psi = geo_.psiDeg[i] * kDeg2Rad;
        const double beta = geo_.betaDeg[i] * kDeg2Rad;

        const Vec3 pl{ geo_.platformRadius * std::cos(psi),
                       geo_.platformRadius * std::sin(psi), 0.0 };
        const Vec3 r = rotate(roll, pitch, yaw, pl);
        const Vec3 q{ origin.x + r.x, origin.y + r.y, origin.z + r.z };
        const Vec3 B{ geo_.baseRadius * std::cos(phi),
                      geo_.baseRadius * std::sin(phi), 0.0 };
        const Vec3 l{ q.x - B.x, q.y - B.y, q.z - B.z };

        const double L2 = (l.x * l.x + l.y * l.y + l.z * l.z) - s2ma2;
        const double M = 2.0 * geo_.hornLength * l.z;
        const double N = 2.0 * geo_.hornLength * (l.x * std::cos(beta) + l.y * std::sin(beta));
        const double denom = std::sqrt(M * M + N * N);

        LegResult lr;
        if (denom < 1e-9) {
            lr.reachable = false;
            lr.angleDeg = 0.0;
        } else {
            double ratio = L2 / denom;
            if (ratio < -1.0 || ratio > 1.0) {
                lr.reachable = false;
                ratio = ratio < 0.0 ? -1.0 : 1.0;
            }
            lr.angleDeg = (std::asin(ratio) - std::atan2(N, M)) / kDeg2Rad;
        }
        if (!lr.reachable) out.allReachable = false;
        out.legs[i] = lr;

        double demand = geo_.demandHome
                        + (lr.angleDeg / geo_.angleAtFullScale) * geo_.demandHome;
        if (demand < 0.0) demand = 0.0;
        if (demand > geo_.demandMax) demand = geo_.demandMax;
        out.setpoints[geo_.bff[i] - 1] = static_cast<uint16_t>(std::lround(demand));
    }
    return out;
}
