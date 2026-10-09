# AttitudeCAN — Design

Datum: 2026-10-09
Status: abgenommen, bereit für den Implementierungsplan

## Zweck

Der künstliche Horizont (Attitude Indicator) wird bisher vom `StepperBoard` (Mega 2560)
über AirManager angesteuert, Treiber `GyroDrive.cpp/h`. Er wird ein eigenständiges
CAN-Instrument `AttitudeCAN` auf der üblichen Hardware: Arduino Nano, MCP2515, zwei
28BYJ-48 hinter ULN2003. Die Kette Plugin → DCU → Board existiert für dieses Instrument
noch nicht — Node-ID, CAN-Nachricht, Serial-Nachricht und Dataref-Anbindung kommen alle neu.

`StepperBoard` bleibt unangetastet: es treibt weiterhin die X25-Motoren über AirManager,
und der Gyro-Pfad dort ist der nominale Fallback, bis AttitudeCAN am Rig bestätigt ist
(gleiches Muster wie `AltimeterDriver` / `HSIDriver`).

## Topologie

```
X-Plane ── DCUProviderPlugin ──USB serial 0x0F──▶ DCU ──CAN 0x101──▶ AttitudeCAN
           pitch_vacuum_deg_pilot                 (Mega 2560)        (Nano, 2× 28BYJ-48, 1 LED)
           roll_vacuum_deg_pilot
```

Reiner Konsument. Das Board sendet außer dem Instrument-Heartbeat nichts zurück.

## Mechanik — was `GyroDrive` verrät

- Zwei Achsen, je 4096 Mini-Steps pro Umdrehung, ULN2003, direkt an Pins.
- **Verschachtelt:** der Pitch-Motor sitzt auf dem Roll-Rahmen. Dreht Roll um R Steps,
  dreht die Pitch-Anzeige um R Steps mit. `GyroDrive::moveSteps` zieht deshalb bei jeder
  Roll-Fahrt `−numStepsRoll` vom Pitch-Motor ab (`correctPitchForRoll`), und
  `moveToDegree` rechnet `correctedPositionPitch = targetPitch − rollPosition`.
  In Steps gilt: **Anzeige-Pitch = Pitch-Motor + Roll** (modular).
- **Roll** ist eine Vollrotation (Shortest-Path über 360°).
- **Pitch-Anzeige** hat mechanische Anschläge. `lookForZeroChange` meldet „Reisebudget
  aufgebraucht ohne Sensorwechsel" als `homingError` — das ist die Endanschlag-Erkennung.
  Das AirManager-LUA kappt Pitch auf ±30°.
- Skalierung: Firmware multipliziert Pitch mit `kAdjustmentFactor = 0.76923077`
  (mechanische Übersetzung Anzeige-Grad → Motor-Grad). LUA negiert Roll.
- Geschwindigkeitskopplung: die Achse mit mehr Steps fährt Max-RPM, die andere bekommt
  ihren Delay proportional gestreckt, so dass beide gleichzeitig ankommen.
- Homing blockierend, Hall-Sensoren an D2/D3 per CHANGE-Interrupt.

Vakuum-Ausfall: das LUA parkte das Instrument bei Vakuum-Fehler auf (−30, +30). Die
`*_vacuum_deg_pilot`-Datarefs modellieren das Taumeln des vakuumgetriebenen Kreisels in
X-Plane selbst. **Entscheidung:** nur die beiden Datarefs, kein zusätzliches Flag.

## Protokoll (`shared/CANBase`)

### `CanNodeId.h`

```cpp
attitudeNodeId = 0x0C
```

Folgt `hsiNodeId = 0x0B`, entspricht Attitude = 12 in der Gerätetabelle.

### `CanMessageId.h`

```cpp
// 0x101: Attitude (Gateway -> Instrument, 50Hz)
// [0..1] pitch int16, degrees * 100 (sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot)
// [2..3] roll  int16, degrees * 100 (sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot)
// [4..7] reserved
attitude = 0x101
```

Big-endian wie jedes Mehrbyte-Feld auf dem Bus. Beide Werte sind legitim negativ.

### `SerialMessageId.h`

```cpp
// Plugin -> DCU. Payload: AttitudeMessage (8 bytes, host order).
SerialMessageAttitude = 0x0F

struct __attribute__((packed)) AttitudeMessage {
    float pitchDeg;   // pitch_vacuum_deg_pilot
    float rollDeg;    // roll_vacuum_deg_pilot
};
static_assert(sizeof(AttitudeMessage) == 8, ...);
```

MSVC-Zweig mit `#pragma pack` wie bei `HsiHeadingMessage`. Reihenfolge Pitch, Roll —
dieselbe wie im CAN-Frame.

## Hardware

Pinvorschlag; die Verdrahtung entscheidet der Aufbau.

| Pin | Funktion | Begründung |
|---|---|---|
| D11 / D12 / D13 | SPI MOSI / MISO / SCK | Hardware-SPI |
| D10 | MCP2515 `/CS` | AVR `/SS` muss Ausgang sein, sonst fällt SPI aus dem Master-Modus |
| D2 | MCP2515 `/INT` | INT0; `BaseCAN` hängt den RX-Handler an `attachInterrupt()` |
| D4, D7, D8, D9 | Roll ULN2003 IN1..IN4 | rein digital |
| A0, A1, A2, A3 | Pitch ULN2003 IN1..IN4 | rein digital, volle GPIO |
| D3 | Hall Roll | `INPUT_PULLUP`, aktiv LOW, **gepollt** |
| D6 | Hall Pitch | `INPUT_PULLUP`, aktiv LOW, **gepollt** |
| D5 | Panel-LED | Timer0-PWM; berührt den Prescaler nicht, `millis()` bleibt sauber |

Die Hall-Sensoren wandern von Interrupts auf Polling: der Nano hat nur D2/D3 als
externe Interrupts, D2 braucht CAN. Die Homing-State-Machine liest die Sensoren ohnehin
zwischen den Steps, wie `WhiskeyCompassCAN`.

A4/A5 bleiben frei. Kein Servo, kein MCP23017 → `lib_deps` nur `coryjfowler/mcp_can`.

## Projektstruktur

```
AttitudeCAN/
  platformio.ini                      [avr] + env:nano + env:native
  lib/CheapStepper/                   Kopie von WhiskeyCompassCAN/lib/CheapStepper
    library.json, src/CheapStepper.cpp/.h   (inversed-Flag, kein MCP23017-Pfad)
  include/AttitudeGeometry.h          Arduino-frei, nativ getestet
  test/test_geometry/test_geometry.cpp
  src/main.cpp                        setup()/loop(), BENCHDEBUG-Schalter
  src/Configuration.h                 Pins, Node-ID, Budgets, Defaults
  src/DebugLog.h
  src/AttitudeIndicator.cpp/.h        zwei Stepper, Kopplung, Homing, EEPROM, LED
  src/CAN.cpp/.h                      InstrumentCAN-Subklasse
  src/BenchDebug.cpp/.h               Serial-Konsole
```

`platformio.ini` exakt wie `WhiskeyCompassCAN` (`[avr]`-Abschnitt, `env:nano` mit
`extends = avr`, `env:native` mit `build_src_filter = -<*>`).

## `Configuration.h`

```cpp
#define BENCHDEBUG 0
const CanNodeId kNodeId = CanNodeId::attitudeNodeId;

const uint8_t kCanIntPin = 2, kCanCSPin = 10, kLightPin = 5;
enum AttitudeAxis { roll = 0, pitch, attitudeAxisCount };
enum RpmKeys { maxRpm = 0, minRpm, rpmKeyCount };
const uint8_t kStepperPins[attitudeAxisCount][4] = { {4, 7, 8, 9}, {A0, A1, A2, A3} };
const uint8_t kHallPins[attitudeAxisCount] = { 3, 6 };
// Ein Wert für beide Achsen: die Kopplung (Anzeige = Motor + Roll in Steps)
// setzt gleiche Stepzahlen voraus. GyroDrives convertPosition() entfällt damit.
const uint32_t kTotalSteps = 4096L;
const double kRpmLimits[attitudeAxisCount][rpmKeyCount] = { {14.0, 4.0}, {14.0, 4.0} };

// Anzeige-Grenzen und Übersetzung (aus LUA-Cap und kAdjustmentFactor).
const double kPitchMaxDegree = 30.0;
const double kPitchScale = 0.76923077;
const bool kRollInversed = true;        // LUA sendete -roll

// Werkseinstellung der Nullpunkte; live in EEPROM.
const int16_t kDefaultZeroAdjustDegree[attitudeAxisCount] = { 1, -1 };

// Homing-Reisebudgets in Grad, aus GyroDrive übernommen (siehe Homing).
```

Die RPM-Werte sind die heutigen (`kRollMinRpm = 4`). Die 4 rpm liegen unter der
„<6 rpm überhitzt"-Marke von CheapStepper, werden aber nur für die kurzen Feinsuch-Phasen
benutzt — wie bisher. `kPitchScale` bleibt Compile-Konstante: ändert sich nur mit der
Mechanik, nicht mit dem Exemplar.

## `AttitudeGeometry.h`

Arduino-frei, reine Funktionen, alle mit Unity-Tests.

- `normalizeStepPosition`, `shortestPathSteps` — Kopie aus `CompassGeometry.h`.
- `signedDegreesToSteps(double deg, uint32_t total)` → `int32_t`, gerundet, **nicht**
  normalisiert (Pitch ist signed).
- `pitchDisplayDegree(double pitchDeg)`: clamp auf `±kPitchMaxDegree`, dann `× kPitchScale`.
- `rollDisplayDegree(double rollDeg)`: `kRollInversed ? −roll : roll`.
- `pitchMotorTarget(int32_t pitchDisplaySteps, uint32_t rollTargetSteps, uint32_t total)`
  = `normalize(pitchDisplaySteps − rollTargetSteps)`. Das ist GyroDrives Kopplung, nur
  als Zielposition statt als Delta-Korrektur. Beide Motoren fahren danach Shortest-Path
  auf ihr Ziel. Zulässig, weil der *relative* Pitch nie über ±30° hinauskommt: der
  Pitch-Motor-Delta ist immer `−rollDelta + pitchDelta`, und keiner der beiden kann den
  Anschlag erreichen.
- `coordinatedDelays(int32_t stepsRoll, int32_t stepsPitch, uint32_t delayRollMax,
  uint32_t delayPitchMax, uint32_t *delayRoll, uint32_t *delayPitch)`: Achse mit mehr
  Steps bekommt ihren Max-Delay, die andere `delay × |mehr| / |weniger|`. Null-Steps auf
  einer Seite → beide Max-Delay. Verhältnis in `uint32_t`, wie bisher.
- `homingWindowCentre` — Kopie aus `HSIGeometry.h` (Fensterbreite > total/8 → `false`).
- `normalizeZeroAdjust`, `accumulateZeroAdjust`, `jogDegreesFromPosition` — Kopie aus
  `CompassGeometry.h`.

## `AttitudeIndicator`

```cpp
class AttitudeIndicator {
public:
    bool isHomed = false;
    void loop();                         // Homing-Step, Fail-Blink, run() beider Stepper
    void beginHoming();                  // nicht-blockierend
    bool isHoming() const;
    bool hasHomingFailed() const;
    AttitudeResult moveToAttitude(double pitchDeg, double rollDeg);   // notHomed wenn nicht homed
    AttitudeResult moveAxisToDegree(AttitudeAxis axis, double deg);   // Bench, Anzeige-Frame
    void setBrightness(uint8_t);
    bool calibrateZero(AttitudeAxis axis);   // akkumulierend, EEPROM
    void wipeCalibration();                  // Defaults, isHomed = false
    void stop(); void off();
    int32_t position(AttitudeAxis) const;
    bool hallActive(AttitudeAxis) const;
};
```

### Setpoints

`moveToAttitude` rechnet per Geometrie: Roll-Ziel (normalisiert) → Shortest-Path-Delta;
Pitch-Anzeige-Steps (geklemmt, skaliert) → Pitch-Motor-Ziel über Roll-Ziel → Shortest-Path-Delta;
`coordinatedDelays`; `newMove` auf beide mit demselben `micros()`. 50-Hz-Frames ersetzen
den laufenden Move wie bisher (`newMove` setzt `stepsLeft` neu). Kein Deadband — HSICAN
brauchte eins wegen Backlash, hier ist keiner bekannt. Bei Bedarf Folge-Ticket.

`moveAxisToDegree` (Bench) hält die andere Achse auf ihrem aktuellen Anzeige-Wert, d. h.
eine Roll-Fahrt geht auch hier mit Pitch-Gegenbewegung.

### Homing

**Harte Regel: Reihenfolge wie in `GyroDrive::homeAllAxis` — erst Roll, dann Pitch.
Jede Roll-Fahrt während des Homings bekommt die Pitch-Gegenbewegung.** Wird davon
abgewichen, fährt die Pitch-Anzeige während der Roll-Suche in den mechanischen Anschlag.
Pitch wird erst gehomt, wenn Roll auf 0 steht; dann ist Pitch-Motor = Pitch-Anzeige und
die Pitch-Phasen brauchen keine Korrektur.

Nicht-blockierende State-Machine im Stil von `WhiskeyCompass::nextHomingState`, eine
Phasenfolge für beide Achsen hintereinander. Jede Suchphase hat ein Reisebudget;
Budget aufgebraucht ohne den erwarteten Sensorwechsel = Endanschlag oder toter Sensor
→ `homingTimeout`.

| Phase | Achse | Fahrt | Erwartung |
|---|---|---|---|
| rollLeaveZero | Roll | +370° @max | Hall Roll inaktiv |
| rollSearchZero | Roll | +370° @max | Hall Roll aktiv |
| rollSearchZeroEnd | Roll | +30° @min | Hall inaktiv → `zeroEnd` merken |
| rollReturnToZeroEnd | Roll | −35° @min | Hall aktiv |
| rollSearchZeroStart | Roll | −35° @min | Hall inaktiv → `zeroStart` merken |
| rollMoveToTrueZero | Roll | `homingWindowCentre` (Plausibilität!) | — |
| rollMoveToAdjustedZero | Roll | `rollZeroAdjustDegree` @max, dann `resetPosition` | — |
| pitchLeaveZero | Pitch | −30° @max | Hall Pitch inaktiv |
| pitchSearchZero | Pitch | +62° @max | Hall Pitch aktiv |
| pitchBackOff | Pitch | −10° @max | — |
| pitchSlowSearchZero | Pitch | +15° @min | Hall Pitch aktiv |
| pitchMoveToAdjustedZero | Pitch | `−30° + pitchZeroAdjustDegree` @max, dann `resetPosition` | — |
| homed | | | `isHomed = true` |

Die Roll-Phasen sind `homeRollAxis` 1:1, ergänzt um den Fenster-Plausibilitätscheck aus
WhiskeyCompass/HSI (zu breit oder zu schmal → Fehler statt 180°-Fehlhoming). Die
Pitch-Phasen sind `homePitchAxis` 1:1. Alle Budgets als Konstanten in `Configuration.h`.

Fehlerfall: `homingFailed = true`, Setpoints werden verweigert, LED blinkt 1 s/1 s mit
Mindesthelligkeit 64 (Compass-Muster), bis `beginHoming()` erneut gerufen wird.

### EEPROM

```cpp
struct Config { uint32_t magic; uint16_t version; int16_t zeroAdjustDegree[attitudeAxisCount]; };
```

Magic `'A','T','C','1'`, Adresse 0. `calibrateZero(axis)` nimmt die aktuelle Position der
Achse als Jog (`jogDegreesFromPosition`) und **addiert** ihn auf den gespeicherten Offset
(`accumulateZeroAdjust`) — nach dem Homing steht die Achse per Definition auf 0 mit altem
Offset eingerechnet. Für Pitch zählt dabei die *Anzeige*-Position (Motor + Roll), damit ein
Jog bei Roll ≠ 0 nicht den Roll-Anteil speichert; in der Praxis wird bei Roll = 0
kalibriert. `wipeCalibration` setzt Defaults und erzwingt Re-Homing (`isHomed = false`),
wie bei WhiskeyCompass begründet.

### LED

`setBrightness(uint8_t)` merkt den Wert immer und schreibt ihn nur durch, wenn kein
Fail-Blink läuft.

## `CAN`

Filter: RXB0 `attitude` exakt (beide Filter), RXB1 `lights` + `gatewayHeartbeat`
(`MASK_EXACT`).

`handleFrame`:
- `0x101`, `len >= 4`: zwei `int16` BE → `/100.` → `moveToAttitude` nur wenn `isHomed`.
  Kein Log pro Frame (Serial-Blockierung bremst `run()`, siehe Compass-Begründung).
- `0x203`, `len >= 8`: `panelDim1000` → PWM, `panelPwm` merken, `panelPwmKnown = true`.

`onStartupFail` / `onGatewayHeartbeatTimeout`: LED aus.
`onGatewayHeartbeatDiscovered`: `setBrightness(panelPwmKnown ? panelPwm : 255)`, dann
`beginHoming()` falls weder homed noch homing. Erstkontakt mit dem Gateway startet das
Homing — vorher läuft kein Sim.

Firmware-Info `{nodeId, 1, 0}`.

## DCU

`DCUReceiver`:
- Case `SerialMessageAttitude`: `len != 8` → return; `memcpy` beide Floats; `lroundf`
  (negativ legitim, kein `+0.5`-Trick); auf `int16` klemmen; bei Änderung eines der beiden
  Werte `sendAttitude()`.
- `sendAttitude()`: `[0..1]` Pitch, `[2..3]` Roll, BE, `[4..7]` 0.
- `attitudeMeta` mit 5000 ms MaxAge in `checkMaxAgeResync`.

`DCU/CLAUDE.md`: `SerialMessageAttitude` in die Receiver-Liste. CAN-Filter des DCU
unverändert — es sendet nur.

## Plugin

`DataRefManager`: `dr_pitch_vacuum`, `dr_roll_vacuum` auf
`sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot` bzw. `roll_vacuum_deg_pilot`;
`getPitchVacuumDeg()`, `getRollVacuumDeg()` über `readFloat`.

`DCUProvider`: `ATTITUDE_RATE = 50.0f`, `attitudeAccumulator_`, lokales Struct
`{ float pitchDeg; float rollDeg; }` wie `AltimeterVsiData`,
`enqueueTx(MessageType::SerialMessageAttitude, …)`.

## BenchDebug

| Befehl | Wirkung |
|---|---|
| `ho` | Homing starten (nicht-blockierend; Fortschritt im DebugLog) |
| `ro<deg>` | Roll auf Grad (Anzeige-Frame, Pitch wird mitgeführt) |
| `pi<deg>` | Pitch auf Grad (Anzeige-Frame, geklemmt ±30) |
| `at<pitch>,<roll>` | Setpoint wie vom Bus |
| `zr` / `zp` | aktuelle Position als Nullpunkt speichern (akkumulierend) |
| `cw` | Kalibrierung auf Defaults, Re-Homing nötig |
| `li<0..255>` | LED-Helligkeit |
| `mo` | beide Motoren stromlos |
| `st` | Status: homed/homing/failed, Motor- und Anzeigepositionen, Halls, Offsets |
| `hm` | Hall-Monitor: beide Sensoren live, Abbruch per Enter |
| `?` | Hilfe |

Ablauf am Tisch: `hm` (Sensoren prüfen), `ho`, dann `ro`/`pi` auf die optische Null und
`zr`/`zp` festschreiben.

## Tests

### Native (`pio test -e native`, `test/test_geometry`)

- `signedDegreesToSteps`: 0, ±90 → ±1024, Rundung, Vielfache von 360.
- `shortestPathSteps`: Kopie der Compass-Fälle (359→1, Gleichstand bricht CW).
- `pitchDisplayDegree`: Clamp ±30, Skalierung, negative Werte.
- `rollDisplayDegree`: Inversion.
- `pitchMotorTarget`: Roll 0 → Motor = Anzeige; Anzeige konstant über eine volle
  Roll-Umdrehung → Pitch-Motor-Delta exakt −4096; Roll 359→1 bei festem Pitch → kleiner
  Pitch-Motor-Delta (kein Wrap-Sprung).
- `coordinatedDelays`: gleich viele Steps → gleiche Delays; 2:1 → doppelter Delay auf der
  kurzen Achse; eine Achse 0 Steps → kein Division-by-zero, beide Max.
- `homingWindowCentre`: Kopie der HSI-Fälle.
- Zero-Adjust: Kopie der Compass-Fälle (Akkumulation, −180..179, Jog rückwärts).

### Bench-Checkliste

1. `hm`: beide Halls schalten beim Drehen von Hand.
2. `ho`: Roll findet Fenster, Pitch findet Sensor, keine Anschlagberührung, `st` zeigt homed.
3. `ro90`, `ro-90`, `ro180`: Pitch-Anzeige bleibt optisch konstant.
4. `pi30`, `pi-30`, `pi40` (geklemmt): kein Anschlag.
5. `at10,-20`: beide Achsen kommen gleichzeitig an.
6. `li0` / `li255`.
7. Am Rig: Gateway-Heartbeat startet Homing; X-Plane-Roll nach rechts → Horizont dreht
   in die richtige Richtung (sonst `kRollInversed` kippen); Pitch-Vorzeichen prüfen.

## Dokumentation

- Root `CLAUDE.md`: `AttitudeCAN` in Board-Liste und Native-Test-Liste,
  `AttitudeGeometry.h` zu den Arduino-freien Headern, Fallback-Hinweis auf den Gyro-Pfad
  in `StepperBoard`.
- `DCU/CLAUDE.md`: neue Serial-Nachricht.

## Nicht im Scope

- Vakuum-Flag im Protokoll (Datarefs reichen, s. o.).
- Setpoint-Deadband und Backlash-Kompensation.
- Stepper-Idle-Abschaltung (bestehendes Ticket `2026-09-30-stepper-idle-shutdown-ticket.md`).
- Änderungen an `StepperBoard`.

## Rollback

`StepperBoard` bleibt flashbar und verdrahtbar wie heute; der künstliche Horizont wird
dafür zurück an den Mega gehängt. AirManager-Seite unverändert.
