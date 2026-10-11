# Rudder axis response curve (DCUProviderPlugin)

Date: 2026-10-11
Scope: `DCUProviderPlugin` only. `RudderCAN`, `DCU` and the serial protocol stay untouched.

## Goal

Make the rudder pedal response tunable on the fly, MSFS-style: an S-curve with
Sensitivity +, Sensitivity −, Neutral, Dead Zone and Extremity Dead Zone, adjusted with
sliders in a dedicated plugin window, applied immediately to the next pedal frame and
persisted across X-Plane restarts. A plot of the curve with a live marker of the current
pedal position helps tuning while flying.

Toe brakes stay linear for now. The curve type and config layout are axis-agnostic so a
second axis (toe brakes, later other joystick axes) can be added without reshaping the
code.

## Non-goals

- Toe-brake or any second axis curve (only the extension path is kept open).
- Axis switcher in the window, tabs, registries.
- Hot-reload of the config file.
- Keyboard input in the window.
- Any change on the firmware side.

## Design

### 1. Curve math — `src/AxisCurve.h` / `src/AxisCurve.cpp`

XPLM-free, testable natively.

```cpp
struct AxisCurve {
    float sensPos  = 0.f;   // -1..1, applies to the positive half
    float sensNeg  = 0.f;   // -1..1, applies to the negative half
    float neutral  = 0.f;   // -1..1, shifts the zero point
    float deadZone = 0.f;   //  0..1 (slider caps at 0.5)
    float extremityDeadZone = 0.f; // 0..1 (slider caps at 0.5)

    float apply(float in) const;   // in -1..1 -> out -1..1
    void  clampToRange();          // used after loading from config
};
```

`apply(in)` runs these steps in order; every step is continuous and monotonic so the
composition is too:

1. Clamp `in` to −1..1.
2. **Neutral**: `x = in - neutral`, then renormalise each half to full length
   (`x > 0: x / (1 - neutral)`, `x < 0: x / (1 + neutral)`), clamp to −1..1. The pedal
   end stops still reach ±1.
3. **Dead zone**: `|x| <= dz` → 0, otherwise `(|x| - dz) / (1 - dz)`, sign kept.
4. **Extremity dead zone**: `|x| >= 1 - ext` → 1, otherwise `|x| / (1 - ext)`.
5. **Sensitivity**: power curve `|x|^p` with `p = 3^(-s)`, `s = sensPos` for `x > 0`,
   `sensNeg` for `x < 0`. `s = 0` → linear, `s = −1` → cubic (flat around centre, the
   screenshot's −0.35 look), `s = +1` → cube root (steep around centre).
6. Reapply sign, clamp to −1..1.

Denominators `(1 - dz)`, `(1 - ext)`, `(1 ∓ neutral)` are clamped to a small positive
epsilon inside `apply()` so out-of-range values from a hand-edited config can never divide
by zero.

The plot in the window samples `apply()` at 65 points; there is no second evaluation path.

### 2. Integration and persistence

**Data flow.** `DCUProvider` owns `AxisCurve rudderCurve_` and `float lastRudderRaw_`.
In `updateUplink()`, case `SerialMessageRudder`:

```cpp
const float raw = static_cast<float>(message->rudder) / 1000.0f;
lastRudderRaw_ = raw;
dataRefMgr_->setRudderRatio(rudderCurve_.apply(raw));
// left/right brake ratios unchanged (raw)
```

Everything runs on X-Plane's main thread (flight loop + draw callbacks). No locking.

**Config** (`ConfigUtils`, existing flat `key=value` file `~/.dcuprovider.cfg`):

```cpp
AxisCurve loadAxisCurve(const std::string& axis);                    // axis = "rudder"
void      saveAxisCurve(const std::string& axis, const AxisCurve& c);
bool      loadCurveWindowVisible();   // default false
void      saveCurveWindowVisible(bool);
```

Keys, prefixed per axis so a second axis is just another prefix:

```
curve.rudder.sens_pos
curve.rudder.sens_neg
curve.rudder.neutral
curve.rudder.dead_zone
curve.rudder.extremity
curve_window_visible
```

Missing keys → 0 (linear). Loaded values go through `clampToRange()`. Loaded once in
`DCUProvider::initialize()`. Saved on slider release (`MouseUp`) and on reset, not per
drag frame.

### 3. Window — `src/CurveWindow.h` / `src/CurveWindow.cpp`

**Menu.** `StatusWindow` keeps owning the "DCU Provider" submenu. It gains

```cpp
void appendMenuItem(const std::string& label, std::function<void()> onSelect);
```

Items are stored in a vector, the XPLM item refcon is the index, `menuCallback`
dispatches. "Show Status Window" uses the same mechanism. `CurveWindow` registers
"Rudder Curve..." through it.

**Class.**

```cpp
class CurveWindow {
public:
    CurveWindow(std::string axisLabel, AxisCurve* curve);
    void initialize();                     // XPLMCreateWindowEx, round-rect decoration
    void destroy();
    void setVisible(bool visible);
    bool isVisible() const;
    void update();                         // 1 Hz: detect native close button, persist visibility
    void setLiveInput(float raw);          // called per rudder frame
    void setChangedCallback(std::function<void()> cb);   // DCUProvider → saveAxisCurve
};
```

`curve` points straight at `DCUProvider::rudderCurve_`; a slider change is live on the
next rudder frame.

**Layout** (about 560×300):

- Left: 240×240 plot. Frame, axis cross, curve as `GL_LINE_STRIP` over 65 samples, live
  marker as a small quad at `(raw, apply(raw))`. Graphics state via
  `XPLMSetGraphicsState(0,0,0,0,0,0,0)` then immediate-mode GL (OpenGL framework is
  already linked).
- Right: five rows `Label | track 150 px with knob | value "%.2f" | [R]`.
  Ranges: Sensitivity ± −1..1, Neutral −1..1, Dead Zone 0..0.5, Extremity 0..0.5.
- Bottom: live line `in 0.31 -> out 0.12`.

**Mouse.** Hitboxes are registered in `draw()` for exactly what is on screen (same
pattern as `StatusWindow`). `MouseDown` on a track sets the value from x and grabs the
slider; `MouseDrag` follows; `MouseUp` fires the changed callback. `[R]` resets that
parameter to 0 and fires the callback.

**Visibility.** `curve_window_visible`, same close-button polling as `StatusWindow`,
driven from the existing 1 Hz status accumulator in `onFlightLoopTick`.

### 4. Files, tests, failure modes

New:
- `src/AxisCurve.h`, `src/AxisCurve.cpp`
- `src/CurveWindow.h`, `src/CurveWindow.cpp`
- `tests/CMakeLists.txt`, `tests/test_axiscurve.cpp`

Changed:
- `CMakeLists.txt` — add sources/headers
- `ConfigUtils.h/.cpp` — curve load/save, curve window visibility
- `StatusWindow.h/.cpp` — `appendMenuItem`
- `DCUProvider.h/.cpp` — `rudderCurve_`, `lastRudderRaw_`, `curveWin_`, apply in the
  rudder case, 1 Hz update, save callback
- `DCUProviderPlugin/CLAUDE.md` — tests exist now, config keys

**Tests** (`tests/test_axiscurve.cpp`, assert-style like `MotionProviderPlugin/tests`,
no framework):

```bash
cd DCUProviderPlugin
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
```

Cases: identity with defaults; endpoints map to ±1; dead zone yields 0 inside and is
continuous at its edge; extremity saturates; neutral shift moves the zero and still
reaches ±1 at the stops; `sens = −1` equals `x³`, `sens = +1` equals cube root; monotonic
sweep over −1..1 for a mixed parameter set; out-of-range config values clamp.

Window and config are verified manually:

- Menu "DCU Provider → Rudder Curve..." opens/closes the window, state survives restart.
- Pedal moves the live marker; dragging Sensitivity changes the sim response immediately.
- Each `[R]` resets its row; values are in `~/.dcuprovider.cfg` after release.
- Status window unchanged.

**Failure modes:**
- Config missing or malformed → defaults, linear, no crash.
- `XPLMCreateWindowEx` fails → log line, window inactive, rudder path unaffected.
- Values out of range in the file → clamped on load.
- No rudder frame yet → marker stays at last value (0 at start), curve still editable.

## Extension path

A second axis = another `AxisCurve` member in `DCUProvider`, another config prefix
(`curve.left_brake.*`), `apply()` at the matching `setXxxRatio` call, and either a second
`CurveWindow` instance with its own menu item or an axis switcher in the existing one.
Nothing in `AxisCurve` or `ConfigUtils` needs to change.
