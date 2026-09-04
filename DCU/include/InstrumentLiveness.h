#ifndef INSTRUMENTLIVENESS_H
#define INSTRUMENTLIVENESS_H
#include <stdint.h>

// Liveness of the instrument nodes, held as two bitmasks rather than one entry
// per node in the CAN error table. Bit N is node N.
//
// `seen` is sticky: a node enters it the first time it reports and never leaves.
// `alive` tracks whether its heartbeat is currently fresh. A node the rig does
// not have is in neither mask, which is what keeps a half-populated bus from
// lighting the alarm LED.
//
// Deliberately Arduino-free so the native Unity tests can exercise it.

static const uint8_t kLivenessMaxNodes = 16;

inline uint16_t nodeBit(uint8_t nodeId)
{
    return (nodeId < kLivenessMaxNodes) ? (uint16_t)(1u << nodeId) : (uint16_t)0u;
}

inline void instrumentMarkSeen(uint16_t &seen, uint8_t nodeId)
{
    seen |= nodeBit(nodeId);
}

inline void instrumentSetAlive(uint16_t &alive, uint8_t nodeId, bool isAlive)
{
    const uint16_t bit = nodeBit(nodeId);
    if (isAlive)
    {
        alive |= bit;
    }
    else
    {
        alive &= (uint16_t)~bit;
    }
}

inline bool instrumentIsAlive(uint16_t alive, uint8_t nodeId)
{
    return (alive & nodeBit(nodeId)) != 0;
}

// Nodes that reported at least once and are not reporting now.
inline uint16_t silentInstruments(uint16_t seen, uint16_t alive)
{
    return (uint16_t)(seen & (uint16_t)~alive);
}

inline bool anyKnownInstrumentSilent(uint16_t seen, uint16_t alive)
{
    return silentInstruments(seen, alive) != 0;
}

#endif // INSTRUMENTLIVENESS_H
