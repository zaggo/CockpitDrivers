# TurnCoordinatorCAN — Design

Datum: 2026-10-09
Status: abgenommen, bereit für den Implementierungsplan

## Zweck

Der Turn Coordinator (Wendezeiger mit Kugel) hat zwei Nadeln: Turn Rate (Flugzeugsymbol)
und Side Slip (Kugel). Beide sitzen auf je einem X27.168-Schrittmotor und werden bisher vom
`StepperBoard` (Mega 2560, SwitecX25 direkt an Pins) über AirManager angesteuert. Sie werden
ein eigenständiges CAN-Instrument `TurnCoordinatorCAN` auf der üblichen Hardware: Arduino
Nano, MCP2515, ein AX1201728SG-Treiber (zwei Kanäle, STEP/DIR je Kanal, gemeinsamer RST,
12 Steps/°). Die Kette Plugin → DCU → Board existiert für dieses Instrument noch nicht —
Node-ID, CAN-Nachricht, Serial-Nachricht und Dataref-Anbindung kommen alle neu.

`StepperBoard` bleibt unangetastet und ist der nominale Fallback, bis das CAN-Board am Rig
bestätigt ist (gleiches Muster wie `AltimeterDriver` / `HSIDriver` / Gyro-Pfad).

Vorlage ist `VerticalSpeedCAN`: gleicher Treiber-Fork, gleiche Kalibriertabelle, gleiche
Eigenart, dass die Nadel bei 0 mitten auf der Skala steht und der mechanische Home-Anschlag
ein angelernter Punkt wie jeder andere ist — nur zweimal.

## Topologie

```
X-Plane ── DCUProviderPlugin ──USB serial 0x10──▶ DCU ──CAN 0x103──▶ TurnCoordinatorCAN
           turn_rate_roll_deg_pilot               (Mega 2560)        (Nano, AX1201728SG,
           slip_deg                                                   2× X27.168, 1 LED)
```

Reiner Konsument. Das Board sendet außer dem Instrument-Heartbeat nichts zurück.

## Entscheidungen

- **Eine Node, nicht zwei.** Node = physischer MCP2515 + Heartbeat. Zwei IDs auf einem
  Board hieße zwei Heartbeats fürs selbe Gerät und ein doppelt geführtes Instrument in der
  DCU-Liveness-Tabelle; 0x103 trägt ohnehin beide Nadeln in einem Frame. Gerätetabelle:
  TurnCoordinator = 13.
- **Mapping Grad → Steps über angelernte Punkte, pro Nadel eine Tabelle.** Das alte LUA
  zeigt, warum: Turn Rate war linear (±90° ≙ ±0,29 Vollweg), Side Slip klein und
  asymmetrisch um die Mitte (0,084 Mitte, 0 … 0,156 Endwerte, links/rechts verschieden
  steil). Die VSI-Tabelle deckt beides ohne Sonderfall ab, klemmt jenseits der Endpunkte
  gratis und ist nativ getestet. Compile-Konstanten aus dem LUA scheiden aus: sie gelten
  für SwitecX25 mit 3 Steps/° auf 315°, und der Nullpunkt stimmt nach dem Umbau sowieso
  nicht mehr.
- **Kein Bus-Spannungs-Flag im Protokoll.** Das LUA parkte die Nadeln bei
  `bus_volts < 7`. Analog zur Attitude-Entscheidung (Vakuum) verlassen wir uns auf die
  `gauges/indicators`-Datarefs, die X-Plane bei Stromausfall selbst abfallen lässt.
- **Keine Richtungsumkehr im Code.** Die Tabelle lernt die Drehrichtung mit: läuft eine
  Nadel verkehrt herum, werden am Bench die Punkte entsprechend angelernt.

## Protokoll (`shared/CANBase`)

### `CanNodeId.h`

```cpp
turnCoordinatorNodeId = 0x0D
```

Folgt `attitudeNodeId = 0x0C`.

### `CanMessageId.h`

Einsortiert zwischen `altimeterVsi` (0x102) und `hsiHeading` (0x104):

```cpp
// 0x103: Turn coordinator (Gateway -> Instrument, 50Hz)
// [0..1] turn rate int16, degrees * 100 (sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot)
// [2..3] side slip int16, degrees * 100 (sim/cockpit2/gauges/indicators/slip_deg)
// [4..7] reserved
// Both signed; both needles rest mid-dial at 0. The gateway forwards whatever
// the sim says, the board clamps via its calibration table.
turnCoordinator = 0x103,
```

Big-endian wie jedes Mehrbyte-Feld auf dem Bus.

### `SerialMessageId.h`

```cpp
// Plugin -> DCU. Payload: TurnCoordinatorMessage (8 bytes, host order).
// Repacked by the DCU into CAN 0x103 as two big-endian int16 of degrees * 100.
SerialMessageTurnCoordinator = 0x10,

struct __attribute__((packed)) TurnCoordinatorMessage {
    float turnRateDeg;  // turn_rate_roll_deg_pilot
    float slipDeg;      // slip_deg
};
static_assert(sizeof(TurnCoordinatorMessage) == 8, "TurnCoordinatorMessage must be 8 bytes on the wire");
```

MSVC-Zweig mit `#pragma pack` wie bei `AttitudeMessage`. Reihenfolge Turn Rate, Slip —
dieselbe wie im CAN-Frame.

Grad → int16·100 macht `attitudeDeg100()` aus `AttitudeWire.h` (NaN → 0, Clamp auf int16,
Rundung weg von Null). Kein neuer Helper.

## Hardware

Pinvorschlag, VSI-Belegung um den zweiten Kanal erweitert; die Verdrahtung entscheidet der
Aufbau.

| Pin | Funktion | Begründung |
|---|---|---|
| D11 / D12 / D13 | SPI MOSI / MISO / SCK | Hardware-SPI |
| D10 | MCP2515 `/CS` | AVR `/SS` muss Ausgang sein, sonst fällt SPI aus dem Master-Modus |
| D2 | MCP2515 `/INT` | INT0; `BaseCAN` hängt den RX-Handler an `attachInterrupt()` |
| D7 / D8 | Turn Rate STEP / DIR | wie VSI |
| D4 / D6 | Side Slip STEP / DIR | rein digital |
| D9 | AX1201728SG RST | gemeinsam für beide Kanäle, wie `kRstPin` bei VSI |
| D3 | Panel-LED | Timer2-PWM, `millis()` unberührt |

`lib_deps` nur `coryjfowler/mcp_can`. Der nicht-blockierende vid6608-Fork wird aus
`VerticalSpeedCAN/lib/vid6608` nach `lib/vid6608` kopiert (Fork-Hinweis im Header bleibt,
das `platformio.ini`-Kommentar „nicht petrows/vid6608 eintragen“ ebenso). Beide Motoren
`kSteps = 320 * 12`. Die VSI-Accel-Tabelle (Cruise nach 32 statt 800 Steps, weil 50-Hz-Ziele
nur wenige Steps auseinanderliegen) wird übernommen.

## Projektstruktur

```
TurnCoordinatorCAN/
  platformio.ini                        [avr] + env:nano + env:native (Kopie VSI)
  lib/vid6608/                          Kopie VerticalSpeedCAN/lib/vid6608
    library.json, src/vid6608.cpp/.h
  include/TurnCoordinatorCalibration.h  VSI-Tabelle, Feld `fpm` → `value`
  test/test_calibration/test_calibration.cpp
  src/main.cpp                          setup()/loop(), BENCHDEBUG-Schalter
  src/Configuration.h                   Pins, Node-ID, kSteps, MASK_EXACT
  src/DebugLog.h
  src/TurnCoordinator.cpp/.h            beide Nadeln, EEPROM, LED
  src/CAN.cpp/.h                        InstrumentCAN-Subklasse
  src/BenchDebug.cpp/.h                 Serial-Konsole
```

### `TurnCoordinatorCalibration.h`

Kopie von `VerticalSpeedCalibration.h` mit `CalibrationPoint { int16_t value; uint16_t step; }`
und denselben Funktionen (`calibrationWipe`, `calibrationSet`, `calibrationMinValue`,
`calibrationMaxValue`, `calibrationStepFor`). Arduino-frei, `kMaxCalibrationPoints = 12`.
Werte in ganzen Grad: Turn Rate braucht Punkte um ±90°, Slip bei −8/0/8.

### `TurnCoordinator`

```cpp
enum Needle { turnRate = 0, sideSlip, needleCount };
```

Zwei `vid6608*` in einem Array, über `Needle` indiziert. Config im EEPROM:

```cpp
struct Config {
    uint32_t magic;      // 'T','C','N','1'
    uint16_t version;    // 1
    CalibrationTable tables[needleCount];
};
```

`loadConfig()`/`saveConfig()` wie VSI: falsches Magic oder falsche Version → Defaults
schreiben; richtige Kennung, aber `count > kMaxCalibrationPoints` in einer der Tabellen →
beide wipen (nie über das Array hinaus indizieren). `count == 0` ist legal.

Konstruktor: RST LOW, beide Motoren anlegen und Accel-Tabelle setzen, LED-Blink wie VSI,
RST HIGH, `zero()` **nacheinander** (blockierend; zwei Sweeps, einmalig beim Start),
`loadConfig()`.

API:

- `moveNeedle(Needle, float deg, bool calibration = false)` — Kalibriermodus fährt
  `deg * 12` Rohsteps ab Home-Anschlag; sonst `calibrationStepFor(tables[n], deg)`.
  Kein `isMoving()`-Bailout: der Fork retargetet im Flug, 50-Hz-Ziele glätten die Fahrt.
- `setTurnCoordinator(float turnDeg, float slipDeg)` — beide Nadeln, ein Aufruf aus
  `handleFrame`.
- `home()` — beide `zero()`, nacheinander.
- `calibratePoint(Needle, int16_t deg)` — aktuelle `getPosition()` des Motors als Punkt;
  `false`, wenn die Tabelle voll ist.
- `wipeCalibration()` — beide Tabellen leeren und speichern; Nadeln parken auf dem
  Home-Anschlag, bis der erste Punkt gelernt ist.
- `setBrightness(uint8_t)`, `loop()` (beide `motor->loop()`), `calibration(Needle)`.

### `CAN`

Filter: RXB0 `turnCoordinator` exakt (beide Filter), RXB1 `lights` + `gatewayHeartbeat`
(`MASK_EXACT`).

`handleFrame`:
- `0x103`, `len >= 4`: zwei `int16` BE → `/100.` → `setTurnCoordinator`. Kein Log pro Frame.
- `0x203`, `len >= 8`: `panelDim1000` → PWM → `setBrightness`.

`onStartupFail` / `onGatewayHeartbeatTimeout`: LED aus. `onGatewayHeartbeatDiscovered`:
LED an. `instrumentBegin` fährt beide Nadeln auf 0 (über Tabelle) und LED aus.
Firmware-Info `{nodeId, 1, 0}`.

### BenchDebug

| Befehl | Wirkung |
|---|---|
| `tr<deg>` / `sl<deg>` | Nadel über Tabelle, wie vom Bus |
| `ct<deg>` / `cs<deg>` | Rohgrad ab Home-Anschlag (Kalibrierfahrt) |
| `pt<deg>` / `ps<deg>` | aktuelle Position als Punkt für `<deg>` speichern |
| `ho` | beide Nadeln homen |
| `cw` | beide Tabellen löschen |
| `br<0..255>` | LED-Helligkeit |
| `co` / `cx` | Dauertest: Zufallsziele für beide Nadeln, interpoliert und mit 50 Hz zugestellt wie echte CAN-Telemetrie |
| `?` | Hilfe mit Tabellenausgabe |

Während `co` sind `tr/sl/ct/cs/pt/ps/ho/cw` gesperrt (wie VSI).

Ablauf am Tisch: `ho`; `ct<x>` tasten, bis das Flugzeugsymbol auf der linken Marke steht →
`pt-…`, Mitte → `pt0`, rechte Marke → `pt…`; dasselbe mit `cs`/`ps` für die Kugel bei
−8 / 0 / 8. Weitere Punkte nach Bedarf.

## DCU

`DCUReceiver`:
- Case `SerialMessageTurnCoordinator`: `len != sizeof(TurnCoordinatorMessage)` → return;
  `memcpy`; `attitudeDeg100()` auf beide Floats; bei Änderung eines der beiden Werte
  `sendTurnCoordinator()`.
- `sendTurnCoordinator()`: `[0..1]` Turn Rate, `[2..3]` Slip, BE, `[4..7]` 0.
- `turnCoordinatorMeta {0, 5000}` im Konstruktor und in `checkMaxAgeResync`.
- Zustand: `int16_t turnRate100`, `slip100`.

`BenchDebug` (DCU): `tc<turn>,<slip>` setzt beide Werte und sendet 0x103 — ein Befehl, weil
ein Frame. Hilfe-Zeile dazu.

CAN-Filter des DCU unverändert — es sendet 0x103 nur.

## Plugin

`DataRefManager`: `dr_turn_rate`, `dr_slip` auf
`sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot` bzw. `slip_deg`;
`getTurnRateDeg()`, `getSlipDeg()` über `readFloat`.

`DCUProvider`: `TURN_COORDINATOR_RATE = 50.0f`, `turnCoordinatorAccumulator_`,
`TurnCoordinatorMessage` befüllen, `enqueueTx(MessageType::SerialMessageTurnCoordinator, …)`.
Block direkt nach dem Attitude-Block.

## Tests

### Native (`pio test -e native`)

`TurnCoordinatorCAN/test/test_calibration`: VSI-Suite auf `value` umbenannt (Sortierung,
Ersetzen, volle Tabelle, Interpolation über den Vorzeichenwechsel, Parken an den Enden,
0 ist nicht der Home-Step, leere Tabelle hält den Anschlag) plus ein Fall mit Slip-Werten:
Punkte −8/0/8 mit asymmetrischen Steps (z. B. 0 / 950 / 1764), −4 und +4 interpolieren
jeweils auf ihrer eigenen Steigung.

`DCU/test`: unverändert — `attitudeDeg100` ist bereits abgedeckt.

### Builds

- `pio run -e nano` in `TurnCoordinatorCAN`
- `pio run` in `DCU`
- `./build-macos.sh` in `DCUProviderPlugin`

### Bench-Checkliste

1. Start: beide Nadeln fahren an den Anschlag, LED blinkt dreimal.
2. `ho` wiederholt → beide am Anschlag, kein Überspringen.
3. Kalibrierablauf wie oben; `?` zeigt beide Tabellen.
4. `tr0` / `sl0` → beide Nadeln auf Mitte; `tr90`, `tr-90`, `sl8`, `sl-8` auf den Marken;
   `tr200` parkt am äußersten Punkt.
5. `co`: beide Nadeln folgen weich, kein Zittern.
6. `br0` / `br255`.
7. Am Rig: Gateway-Heartbeat → LED an; Rechtskurve in X-Plane → Flugzeugsymbol kippt
   nach rechts, Kugel wandert bei Schiebeflug in die richtige Richtung. Falsche Richtung →
   Punkte neu anlernen, kein Code.

## Dokumentation

- Root `CLAUDE.md`: `TurnCoordinatorCAN` in Board-Liste und Native-Test-Liste,
  `TurnCoordinatorCalibration.h` zu den Arduino-freien Headern, Fallback-Satz: `StepperBoard`
  treibt Turn Rate / Side Slip weiter, bis das CAN-Board am Rig bestätigt ist.
- `DCU/CLAUDE.md`: `SerialMessageTurnCoordinator` in die Receiver-Liste, `tc`-Befehl.

## Nicht im Scope

- Bus-Spannungs-Flag im Protokoll (s. Entscheidungen).
- Stepper-Idle-Abschaltung (bestehendes Ticket `2026-09-30-stepper-idle-shutdown-ticket.md`).
- Setpoint-Deadband.
- Änderungen an `StepperBoard`.
- Eine zweite Node für die Kugel.

## Rollback

`StepperBoard` bleibt flashbar und verdrahtbar wie heute; Turn Rate und Side Slip werden
dafür zurück an den Mega gehängt. AirManager-Seite unverändert.
