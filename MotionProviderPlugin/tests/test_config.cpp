#include "MotionConfig.h"
#include "StewartGeometry.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }
static void near(double a,double b,const char*w){ ++g_checks; if(std::fabs(a-b)>1e-9){++g_failures; std::printf("  FAIL: %s (%.6f vs %.6f)\n",w,a,b);} }

int main() {
    // Missing file -> defaults.
    {
        StewartGeometry g = MotionConfig::loadGeometry("/no/such/file.toml");
        StewartGeometry d = StewartGeometry::defaults();
        near(g.baseRadius, d.baseRadius, "missing file -> default Rb");
        near(g.rodLength,  d.rodLength,  "missing file -> default rod");
    }

    // Partial file -> overrides present keys, defaults the rest.
    {
        std::string tmp = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                          + "/mp_test.toml";
        {
            std::ofstream f(tmp);
            f << "[geometry]\n"
                 "base_radius_mm = 500.0\n"
                 "base_angle_deg = [1,2,3,4,5,6]\n"
                 "[servo]\n"
                 "demand_home = 30000\n";
        }
        StewartGeometry g = MotionConfig::loadGeometry(tmp);
        StewartGeometry d = StewartGeometry::defaults();
        near(g.baseRadius, 500.0, "override Rb");
        near(g.platformRadius, d.platformRadius, "default Rp kept");
        near(g.phiDeg[0], 1.0, "override phi[0]");
        near(g.phiDeg[5], 6.0, "override phi[5]");
        check(g.demandHome == 30000, "override demand_home");
        check(g.demandMax == d.demandMax, "default demand_max kept");
        std::remove(tmp.c_str());
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
