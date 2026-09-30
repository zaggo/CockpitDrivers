# Ticket: two AltimeterCAN defects found while building HSICAN

**Status:** open. Written 2026-09-30, during the final review of `feature/hsi-can`. `AltimeterCAN` is
running on the rig today — neither defect is hypothetical.

**Type:** two small, independent bug fixes. Not bundled into the HSI branch because they touch a
different board and don't belong in that diff.

## 1. Silent 180° mis-home in the hall-window midpoint

`AltimeterCAN/src/Altimeter.cpp:299`:

```cpp
int32_t zeroAdjust = (zeroEndPosition[axis] - zeroStartPosition) % static_cast<int32_t>(axes[axis]->getTotalSteps()) / 2L;
```

`zeroEndPosition` and `zeroStartPosition` are two hall-edge captures taken in different homing
phases, each following its own `resetPosition()`. The subtraction assumes `zeroEnd` is always ahead of
`zeroStart` in the same lap. When the two arrive out of order the difference wraps to almost a full
turn, and half of that becomes the new zero offset — a **180° mis-home that still returns `success`**.
There is no range check on the result to catch it.

`WhiskeyCompassCAN` hit this exact defect twice before it was understood as a wrap, not a one-off bad
reading — see `HSICAN/include/HSIGeometry.h`'s `homingWindowCentre()` comment. That function is the
tested fix: it computes the span as an always-forward modular distance, rejects anything wider than an
eighth of a turn as a misread (returning `false` instead of a bad centre), and only then halves it. It
is Arduino-free and already covered by `HSICAN/test/test_geometry` — lifting it into `AltimeterCAN`
is a signature change at the call site plus handling the new failure return (fold it into the existing
`homingTimeout` path), not new logic.

## 2. Brightness slams to full on gateway reconnect

`AltimeterCAN/src/CAN.cpp:101`, `CAN::onGatewayHeartbeatDiscovered()`:

```cpp
altimeter->setBrightness(255);
```

Unconditional. If the panel dimmer was sitting at zero when the gateway heartbeat drops and comes
back, the instrument flashes to full brightness and stays there until the next `lights` CAN frame
resyncs it — which, per `DCU/CLAUDE.md`'s max-age resync, could be up to 5s away, and is jarring at
night.

`HSICAN` doesn't have this bug: it tracks a `panelPwmKnown` flag and only restores brightness on
reconnect if a real value has been received, because 0 is a legitimate dimmer setting and must not be
treated as "no value yet". Porting that flag (plus gating `setBrightness()` on it in the reconnect
handler) is a few lines and directly portable — same `InstrumentCAN` heartbeat callback, same
brightness API.

## Why these are a ticket, not fixed inline

Both live in `AltimeterCAN`, which this branch doesn't touch. Fixing them here would be a
scope-creeping diff into an unrelated board for a PR titled "HSI CAN". They're cheap, understood, and
should be picked up as a follow-up commit against `AltimeterCAN` directly.
