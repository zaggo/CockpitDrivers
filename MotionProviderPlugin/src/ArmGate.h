#pragma once

// Latch that gates the hardware arm switch. A fault or manual e-stop latches
// disarm; the latch clears only when the switch is cycled off (armed->disarmed),
// so a fault can't be instantly re-armed by a switch that is still held on.
// Pure, header-only, no deps.
class ArmGate {
public:
    // Call once per tick with the hardware-armed signal (switch closed AND
    // heartbeat fresh). Returns true on the armed->disarmed edge (the reset
    // point), and clears the latch on that edge.
    bool update(bool hwArmed) {
        const bool cleared = prev_ && !hwArmed;
        if (cleared) latched_ = false;
        prev_ = hwArmed;
        return cleared;
    }

    // Latch disarm until the switch is cycled off (fault or manual e-stop).
    void latchDisarm() { latched_ = true; }

    bool latched() const { return latched_; }

private:
    bool prev_    = false;
    bool latched_ = false;
};
