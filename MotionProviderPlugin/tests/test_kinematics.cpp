#include "StewartKinematics.h"
#include "StewartGeometry.h"
#include <cmath>
#include <cstdio>
#include <cstdint>

static int g_failures = 0;
static int g_checks = 0;
static const StewartGeometry G = StewartGeometry::defaults();
static const StewartKinematics K(G);

static void check(bool cond, const char* what) {
    ++g_checks;
    if (!cond) { ++g_failures; std::printf("  FAIL: %s\n", what); }
}
static void checkNear(double got, double want, double tol, const char* what) {
    ++g_checks;
    if (std::fabs(got - want) > tol) {
        ++g_failures;
        std::printf("  FAIL: %s (got %.6f want %.6f tol %.6f)\n", what, got, want, tol);
    }
}

static double rodClosureError(const Pose& pose, int leg) {
    const double d2r = 3.14159265358979323846 / 180.0;
    SolveResult r = K.solve(pose);
    double th = r.legs[leg].angleDeg * d2r;
    double phi = G.phiDeg[leg]*d2r, psi = G.psiDeg[leg]*d2r, beta = G.betaDeg[leg]*d2r;
    double hx = G.baseRadius*std::cos(phi) + G.hornLength*std::cos(th)*std::cos(beta);
    double hy = G.baseRadius*std::sin(phi) + G.hornLength*std::cos(th)*std::sin(beta);
    double hz = G.hornLength*std::sin(th);
    double cr=std::cos(pose.roll*d2r), sr=std::sin(pose.roll*d2r);
    double cp=std::cos(pose.pitch*d2r), sp=std::sin(pose.pitch*d2r);
    double cy=std::cos(pose.yaw*d2r), sy=std::sin(pose.yaw*d2r);
    double px=G.platformRadius*std::cos(psi), py=G.platformRadius*std::sin(psi), pz=0.0;
    double ax=px, ay=cr*py - sr*pz, az=sr*py + cr*pz;
    double bx=cp*ax + sp*az, by=ay, bz=-sp*ax + cp*az;
    double qx=cy*bx - sy*by + pose.surge;
    double qy=sy*bx + cy*by + pose.sway;
    double qz=bz + K.homeHeight() + pose.heave;
    double dx=qx-hx, dy=qy-hy, dz=qz-hz;
    return std::sqrt(dx*dx+dy*dy+dz*dz) - G.rodLength;
}

int main() {
    std::printf("home height...\n");
    checkNear(K.homeHeight(), 456.3, 0.5, "z0 ~= 456.3mm");

    std::printf("home pose -> all angles 0, all setpoints midscale...\n");
    {
        SolveResult r = K.solve(Pose{});
        check(r.allReachable, "home reachable");
        for (int i=0;i<6;i++) checkNear(r.legs[i].angleDeg, 0.0, 0.05, "home angle ~0");
        for (int i=0;i<6;i++) check(r.setpoints[i]==32640, "home setpoint 32640");
    }

    std::printf("setpoint mapping endpoints...\n");
    {
        auto demand = [](double deg){
            double d = 32640 + (deg/45.0)*32640;
            if (d<0) d=0; if (d>65280) d=65280; return (uint16_t)std::lround(d);
        };
        check(demand(0.0)==32640, "map 0 -> 32640");
        check(demand(45.0)==65280, "map +45 -> 65280");
        check(demand(-45.0)==0,    "map -45 -> 0");
        check(demand(90.0)==65280, "map +90 clamps to 65280");
    }

    std::printf("rod closure round-trip across poses...\n");
    {
        Pose poses[] = {
            Pose{}, Pose{0,0,30,0,0,0}, Pose{0,0,-30,0,0,0},
            Pose{0,0,0,5,0,0}, Pose{0,0,0,0,5,0}, Pose{0,0,0,0,0,5},
            Pose{10,-8,15,3,-2,4},
        };
        for (const Pose& p : poses)
            for (int i=0;i<6;i++)
                if (K.solve(p).legs[i].reachable)
                    checkNear(rodClosureError(p, i), 0.0, 1e-3, "rod closes to s");
    }

    std::printf("pure heave symmetry + sign...\n");
    {
        SolveResult up = K.solve(Pose{0,0,20,0,0,0});
        SolveResult dn = K.solve(Pose{0,0,-20,0,0,0});
        for (int i=0;i<6;i++) check(up.legs[i].angleDeg > 0.0, "heave up -> angle>0");
        for (int i=0;i<6;i++) check(dn.legs[i].angleDeg < 0.0, "heave down -> angle<0");
        for (int i=1;i<6;i++)
            checkNear(up.legs[i].angleDeg, up.legs[0].angleDeg, 0.05, "heave legs equal");
    }

    std::printf("extreme pose flagged unreachable, no NaN...\n");
    {
        SolveResult r = K.solve(Pose{0,0,10000,0,0,0});
        check(!r.allReachable, "huge heave unreachable");
        for (int i=0;i<6;i++) check(std::isfinite(r.legs[i].angleDeg), "angle finite");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
