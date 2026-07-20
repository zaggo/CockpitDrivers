#include "ArmGate.h"
#include <cstdio>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

// Arm intent as the caller computes it.
static bool intent(ArmGate& g, bool hwArmed) {
    g.update(hwArmed);
    return hwArmed && !g.latched();
}

int main() {
    // Plain follow: switch on -> armed, switch off -> disarmed.
    {
        ArmGate g;
        check(intent(g, false) == false, "off -> no intent");
        check(intent(g, true)  == true,  "on -> intent");
        check(intent(g, true)  == true,  "still on -> intent");
        check(intent(g, false) == false, "off -> no intent");
    }
    // Latch (fault/e-stop) while switch stays on: stays disarmed until cycled.
    {
        ArmGate g;
        (void)intent(g, true);           // armed
        g.latchDisarm();                 // fault or e-stop
        check((true && !g.latched()) == false, "latched -> no intent while switch on");
        check(intent(g, true) == false,  "still on + latched -> no intent");
        // Cycle the switch off: update returns true (reset point) and clears latch.
        bool cleared = g.update(false);
        check(cleared == true, "switch off -> reset point reported");
        check(g.latched() == false, "switch off clears the latch");
        // Switch back on: arms again.
        check(intent(g, true) == true, "re-arm after cycle");
    }
    // Reset point only fires on the on->off edge, not while steady off.
    {
        ArmGate g;
        (void)g.update(true);
        check(g.update(false) == true, "on->off is a reset point");
        check(g.update(false) == false, "steady off is not a reset point");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
