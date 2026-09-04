# AltimeterCAN — Port des Altimeters von AirManager auf CAN

Datum: 2026-09-04
Branch: `AltimeterCAN`

## Ziel

`AltimeterDriver` ist die letzte Instrumenten-Firmware, die noch direkt per USB von
AirManager gefüttert wird. Sie wird nach `AltimeterCAN` portiert und hängt danach wie
alle anderen Instrumente am DCU: Höhe kommt über CAN herein, der Baro-Knopf geht über
CAN zurück nach X-Plane.

Der Port ist ausdrücklich **kein Umbau**. Homing, Stepper-Ansteuerung und Achsmathematik
sind an echter Hardware erprobt und bleiben inhaltlich unangetastet. Neu ist die
Anbindung, und neu ist die im Betrieb justierbare Nullpunktkorrektur, die den fest
kompilierten `kZeroAdjustDegree` ablöst.

`AltimeterDriver` bleibt vorerst im Repo stehen — als Rückfallebene, solange
`AltimeterCAN` nicht am Rig bestätigt ist.

## Ausgangslage

Das Instrument hat drei Zeigerachsen (100er, 1000er, 10k), die über einen MCP23017
als Portextender angesteuert werden, je einen Hall-Sensor pro Achse zum Homen, einen
Flag-Servo und ein Poti als Baro-Knopf.

Die Höhe wird in `moveToHeight()` auf drei Winkel abgebildet:

```
hundertsDegree     = fmod(ft,          1000) * 360 / 1000
thousandsDegree    = fmod(ft / 1000,     10) * 360 / 10
tenThousandsDegree = fmod(ft / 10000,    10) * 360 / 10
```

Der Flag wird dort gleich mitberechnet: voll sichtbar unter 9000 ft, linear ausblendend
bis 10000 ft, darüber verborgen. Diese Logik ist bereits aktiv und wird übernommen.
Der auskommentierte, konkurrierende Block in `Altimeter::loop()` (Schwellen 10000/10100
mit Hysterese) wird gelöscht.

`COUPLED_MODE` ist 0 und bleibt es. Der gekoppelte Zweig ist toter Code, wird aber nicht
im Rahmen dieses Ports entfernt.

## Entscheidungen

| Thema | Entscheidung |
|---|---|
| Zuschnitt | Port nach `VerticalSpeedCAN`-Dateilayout, `Altimeter` inhaltlich unangetastet |
| NodeId | `altimeterNodeId = 0x09` |
| Downlink Höhe | `altimeterVsi` (`0x102`), Bytes `[0..3]`, `int32` ft — existiert bereits |
| Uplink Baro | neue ID `0x340`, fest inHg, onChange plus 5s-Refresh |
| Hall 10k | wandert von D10 auf D7 (Hardware-Änderung, siehe unten) |
| Homing | kooperativ aus `loop()`, Start bei Gateway-Discovery |
| Flag | vorhandene Blende aus `moveToHeight()` beibehalten |
| Nullpunkt | EEPROM statt `kZeroAdjustDegree`, drei Bench-Befehle `zh`/`zt`/`ze` |
| Baro-Kalibrierung | zwei Punkte (min/max), linear interpoliert, EEPROM, `bn`/`bx` |
| `AltimeterDriver` | bleibt stehen |

## Hardware und Pins

### Pin-Belegung

| Pin | Funktion | Status |
|---|---|---|
| D2 | Hall 100er, `INPUT_PULLUP`, aktiv LOW | unverändert |
| D4 | CAN `/INT` | neu |
| D5 | Instrumentenbeleuchtung (PWM) | neu |
| D7 | Hall 10k | **verschoben von D10** |
| D8 | Hall 1000er | unverändert |
| D9 | Flag-Servo | unverändert |
| D10 | CAN `/CS` | neu |
| D11 | SPI MOSI | neu |
| D12 | SPI MISO | neu |
| D13 | SPI SCK | neu |
| A0 | Poti (Baro) | unverändert |
| A4/A5 | I2C zum MCP23017 | unverändert |

Frei bleiben D3, D6, A1, A2, A3.

`/CS` liegt bewusst auf D10 und nicht auf einem beliebigen freien Pin. D10 ist das
`/SS`-Pin (siehe unten) und muss im Master-Betrieb zwingend Ausgang sein oder dauerhaft
HIGH liegen. Liegt `/CS` darauf, erledigt `SPI.begin()` das von selbst — bliebe D10
ungenutzt, stünde es nach dem Reset als floatender Eingang da und ein eingekoppelter
LOW-Pegel würfe die SPI erneut aus dem Master-Modus.

Dass D6 dafür frei bleibt, ist der zweite Grund: die Servo-Bibliothek belegt Timer1 und
schaltet PWM auf D9 und D10 ab, D10 kann also ohnehin kein `analogWrite`. D6 hängt an
Timer0 und behält seine PWM-Fähigkeit für spätere Verwendung.

### Warum der Hall-Sensor umziehen muss

D10 ist beim ATmega328 `PB2` und damit das `/SS`-Pin der SPI-Hardware. Ist `/SS` als
Eingang konfiguriert und wird von außen auf LOW gezogen, löscht die SPI-Einheit das
`MSTR`-Bit und fällt in den Slave-Modus — sie interpretiert den Pegel als „ein anderer
Master adressiert mich".

Der Hall-Sensor der 10k-Achse liegt heute auf D10 als `INPUT_PULLUP` und ist aktiv LOW.
Jeder Nulldurchgang der 10k-Nadel — beim Homing und im Flug — würde den CAN-Bus aus dem
Master-Modus werfen. Ein anderes `/CS`-Pin hilft nicht, das Verhalten hängt allein an
D10.

**Hardware-Änderung: ein Jumper von D10 nach D7.** Ohne diese Umverdrahtung ist die
Firmware nicht betriebsfähig.

### Timer

`analogWrite` auf D5 nutzt Timer0, die Servo-Bibliothek auf dem Nano Timer1. Kein
Konflikt. Timer1 kostet allerdings PWM auf D9 und D10 — D9 trägt den Servo selbst, D10
ist als `/CS` reiner Digitalausgang. Beides unkritisch.

Die Onboard-LED des Nano hängt an D13 und flackert künftig mit dem SPI-Verkehr. Der
Heartbeat-Blinker aus `BenchDebug` (`kLedPin = 13`) entfällt ersatzlos.

## CAN-Anbindung

`CAN` erbt von `InstrumentCAN` und implementiert `instrumentBegin()` und `handleFrame()`,
exakt nach dem Muster von `VerticalSpeedCAN/src/CAN.cpp`.

### Empfang

| ID | Inhalt | Verwendung |
|---|---|---|
| `0x102` | `[0..3]` Höhe `int32` ft (BE), `[4..5]` VSI | Bytes `[0..3]`, Rest ignoriert |
| `0x203` | `[0..1]` PanelDim × 1000 `uint16` (BE) | PWM auf D5 |
| `0x300` | Gateway-Heartbeat | von `InstrumentCAN` behandelt |

Filter wie bei `VerticalSpeedCAN`: `MASK_EXACT` auf beiden Puffern, RXB0 auf `0x102`,
RXB1 auf `0x203` und `0x300`.

Höhen-Frames werden verworfen, solange `isHomed` false ist — die Achspositionen sind
vor dem Homing bedeutungslos.

Bei Gateway-Timeout geht die Beleuchtung auf 0, bei Discovery auf volle Helligkeit.
Die Zeiger bleiben stehen, wo sie sind.

### Senden

Neue Nachricht `altimeterBaro = 0x340`, Instrument → DCU, DLC 8:

| Byte | Inhalt | Typ | Skalierung |
|---|---|---|---|
| 0..1 | Baro | uint16 BE | inHg × 100 |
| 2 | Unit | uint8 | fest `1` (= inHg) |
| 3..7 | reserved | | 0 |

Gesendet wird bei Änderung des gerundeten inHg×100-Werts, zusätzlich alle 5000 ms
unverändert nachgeschoben. Der Refresh sorgt dafür, dass der Sim nach einem Neustart
von DCU oder Plugin von selbst wieder den richtigen Wert bekommt, ohne dass jemand am
Knopf drehen muss — dasselbe Muster wie `checkMaxAgeResync()` im `DCUReceiver`.

Das Unit-Byte bleibt fest auf inHg, weil das Poti in inHg kalibriert wird und die
Ziel-Dataref `barometer_setting_in_hg_pilot` ohnehin inHg führt.

## Homing

`homeAllAxis()` verliert seine `while`-Schleife. Die Zustandsmaschine `nextHomingState()`
ist bereits nicht-blockierend und wird künftig aus `loop()` getaktet, solange das Homing
läuft. Damit läuft der Heartbeat in beide Richtungen weiter — bei der heutigen
blockierenden Variante würden sich DCU und Board während der mehrere Sekunden dauernden
Suche gegenseitig für tot erklären.

Ausgelöst wird das Homing, sobald zum ersten Mal ein Gateway-Heartbeat gesehen wird
(`onGatewayHeartbeatDiscovered()`), und danach nur noch auf Bench-Befehl. Ein
Gateway-Timeout bricht ein laufendes Homing nicht ab.

Der Flag fährt nach Abschluss des Homings auf seine Ruhelage, wie bisher.

## Kalibrierung und EEPROM

Ein Konfigurationsblock ab Adresse 0:

```c
struct Config {
    uint32_t magic;                  // 'A','L','T','1'
    uint16_t version;                // 1
    int16_t  zeroAdjustDegree[3];    // 100er, 1000er, 10k
    BaroCalibration baro;
};
```

Magic oder Version falsch → Defaults schreiben. Defaults sind bewusst die heutigen
Werte, damit sich ein frisch geflashtes Board wie das alte verhält:

- `zeroAdjustDegree = {5, 2, 15}` (heutiger `kZeroAdjustDegree`)
- Baro: Rohwert 0 → 28.10 inHg, Rohwert 1023 → 31.00 inHg (übliche Kollsman-Spanne)

### Nullpunktkorrektur

Heute wird `kZeroAdjustDegree[axis]` am Ende des Homings angefahren und die Position
danach auf 0 gesetzt ([Altimeter.cpp:557-558](../../../AltimeterDriver/src/Altimeter.cpp#L557-L558)).
Der EEPROM-Wert tritt an genau diese Stelle, sonst ändert sich am Ablauf nichts.

Wichtig ist die Semantik beim Speichern. Nach dem Homing steht die Achse per Definition
auf Position 0, der alte Offset ist bereits eingerechnet. Fährt man mit `hu`/`th`/`te`
um D Grad auf die optisch richtige Null, ist der neue Offset **alt + D**, nicht D allein.
Wer hier ersetzt statt zu addieren, verliert bei jedem Kalibrierdurchgang den vorherigen
Wert und wandert bei wiederholtem Justieren immer weiter weg.

Das Ergebnis wird auf `-180..179` normalisiert, damit sich über viele Durchgänge kein
Vielfaches von 360 ansammelt.

### Baro

Zwei Stützstellen, linear interpoliert:

```c
struct BaroCalibrationPoint { uint16_t raw; uint16_t inHg100; };
struct BaroCalibration { BaroCalibrationPoint low, high; };
```

`bn`/`bx` speichern den *aktuellen* Rohwert von A0 zusammen mit dem eingetippten
inHg-Wert. Rohwerte außerhalb der Stützstellen werden geklemmt. Sind beide Rohwerte
gleich (versehentlich zweimal dieselbe Stellung kalibriert), wird die Interpolation
umgangen und `low.inHg100` zurückgegeben, statt durch null zu teilen.

Annahme: das Poti ist hinreichend linear. Falls sich das am Rig nicht bestätigt, wird
daraus eine Folge-Story mit einer Stützstellentabelle, analog zu
`VerticalSpeedCalibration.h`.

Die Glättung aus `fetchPressureRatio()` (Rundung auf 1/500) bleibt sinngemäß erhalten,
damit ein zitterndes Poti nicht den Bus flutet — die Entprellung sitzt künftig auf dem
gerundeten inHg×100-Wert.

## BenchDebug

Bestehende Befehle bleiben: `ho`, `hu`, `th`, `te`, `fl`, `cf`, `he`, `st`.

Neu:

| Befehl | Wirkung |
|---|---|
| `zh` | aktuelle Position der 100er-Achse als true Zero speichern |
| `zt` | dito 1000er |
| `ze` | dito 10k |
| `bn<inHg>` | aktuelle Poti-Stellung als unterer Baro-Punkt |
| `bx<inHg>` | aktuelle Poti-Stellung als oberer Baro-Punkt |
| `cw` | **gesamte** Kalibrierung auf Defaults zurücksetzen — Nullpunkte *und* Baro |
| `ba` | aktuellen Baro-Wert und Poti-Rohwert anzeigen |

`st` wird um Offsets, Baro-Stützstellen und den aktuellen Baro-Wert erweitert.

Ablauf am Tisch: `ho`, dann pro Achse mit `hu`/`th`/`te` auf die Nullmarke fahren und
mit `zh`/`zt`/`ze` festschreiben. Für den Baro das Poti an den unteren Anschlag,
`bn28.10`, dann an den oberen, `bx31.00`.

## Änderungen am DCU

### Filter

RXB1 wechselt von `MASK_EXACT` auf den Bereichsfilter `0x7F0`:

```
Maske 0 = MASK_EXACT: F0 = 0x301, F1 = 0x301   (Instrument-Heartbeat)
Maske 1 = 0x7F0:      F2 = 0x310   (Transponder 0x311)
                      F3 = 0x330   (Handbrake 0x330)
                      F4 = 0x300   (Rudder 0x303)
                      F5 = 0x340   (Cluster-Block 0x340-0x34F)
```

F5 deckt damit Altimeter, HSI, Annunciator, GNS und Switchboard ab, plus elf freie
Plätze. Der DCU muss für künftige Cluster-Inputs nicht mehr angefasst werden.
Durchrutschende Nachbar-IDs landen im `default`-Zweig von `handleFrame` und kosten
ein paar Takte.

### Baro-Weiterleitung

Neuer `MessageType::SerialMessageBaro = 0x0A`, Payload `float inHg` (4 Byte,
Host-Order), Richtung DCU → Plugin. `updateBaro()` entpackt `0x340` mit `unpackBE16`,
teilt durch 100 und reicht weiter über `DCUSender`.

Kein maxAge-Resync auf DCU-Seite: die Nachricht kommt vom Instrument, und das schiebt
bereits alle 5 s von sich aus nach.

### Heartbeat-Verwaltung

Unabhängig vom Altimeter, aber im selben Zug:

`checkInstrumentHeartbeats()` schiebt heute Pseudo-IDs `0x301 + nodeId` in
`canIdErrors[]`. Das kollidiert mit echten Bus-IDs (`0x303` Rudder liegt auf der
Pseudo-ID von Knoten 2) und belegt bis zu 15 der Plätze, die für echte TX/RX-Fehler
gedacht sind.

Liveness wird aus der Fehlertabelle herausgelöst:

- `lastInstrumentHeartbeatMs[16]` bleibt als Zeitbasis.
- `bool instrumentAlive[16]` wird zu `uint16_t instrumentSeenMask` und
  `uint16_t instrumentAliveMask`. 4 Byte statt 16.
- Alarm-LED: `!isStarted || anyCanIdHasError(...) || (seen & ~alive)`.
- `canIdErrors[]` enthält danach nur echte Bus-IDs, maximal acht.
  `kMaxCanIdErrors` geht von 24 zurück auf 12.
- `CanErrorType::HEARTBEAT_TIMEOUT` wird dort nicht mehr gesetzt.

Verhalten bleibt gleich: ein Knoten, der nie gesendet hat, löst keinen Alarm aus —
`heartbeatAlive()` liefert bei `lastSeenMs == 0` false, und ein Knoten kommt erst in
`seenMask`, wenn er sich gemeldet hat.

`BenchDebug` bekommt einen Befehl, der die Maske der stummen Knoten ausgibt.

## Änderungen am Plugin

`DataRefManager` hat `setBarometerSetting(float inHg)` bereits — es ruft es nur niemand
auf. Im Uplink-`switch` von `DCUProvider::updateUplink()` kommt ein Zweig für
`SerialMessageBaro` dazu, der genau das tut.

Kein Downlink-Block, keine neue Dataref, keine Rate-Limitierung: die Nachricht kommt
vom Instrument und ist bereits entprellt.

## Tests

Nach Repo-Konvention wandert die Arduino-freie Rechenlogik in Header unter `include/`
und wird nativ mit Unity getestet (`pio test -e native`).

`AltimeterCAN/include/AltimeterCalibration.h`:

- `normalizeZeroAdjust(int32_t degrees) -> int16_t` — Wrap auf `-180..179`,
  inklusive negativer Eingaben und Vielfachen von 360
- `accumulateZeroAdjust(int16_t stored, int32_t jogDegrees) -> int16_t` — die
  Addition oben, mit Test gegen das Ersetzen-statt-Addieren
- `baroInHg100(const BaroCalibration&, uint16_t raw) -> uint16_t` — Interpolation,
  Klemmung an beiden Enden, Sonderfall `low.raw == high.raw`
- `baroCalibrationSet(BaroCalibration&, bool isHigh, uint16_t raw, uint16_t inHg100)`

`DCU/include/InstrumentLiveness.h`:

- `instrumentMarkSeen(uint16_t& seen, uint8_t nodeId)`
- `instrumentSetAlive(uint16_t& alive, uint8_t nodeId, bool alive)`
- `anyKnownInstrumentSilent(uint16_t seen, uint16_t alive) -> bool`

Getestet wird insbesondere, dass ein nie gesehener Knoten keinen Alarm auslöst und
dass ein Knoten, der einmal da war und dann schweigt, es tut.

Nicht automatisiert testbar und daher am Rig zu prüfen: Homing über CAN, Nadelbewegung,
Flag-Blende, Baro-Kalibrierung.

## Reihenfolge der Umsetzung

1. `DCU`: Liveness-Umbau plus `InstrumentLiveness.h` mit Tests. Unabhängig, kann als
   erstes committet und geflasht werden.
2. `shared`: `altimeterNodeId = 0x09`, `altimeterBaro = 0x340`, `SerialMessageBaro`.
3. `AltimeterCAN`: Projekt aus `AltimeterDriver` kopieren, `AirManager.*` und
   `lib/SiMessagePort/` entfernen, `CAN.cpp/h` und `Configuration.h` nach
   `VerticalSpeedCAN`-Muster, Pins setzen.
4. `AltimeterCAN`: `AltimeterCalibration.h` plus native Tests, dann EEPROM-Anbindung
   in `Altimeter`, `kZeroAdjustDegree` entfernen.
5. `AltimeterCAN`: Homing entblocken, `BenchDebug` um die neuen Befehle erweitern.
6. `DCU`: Filterumbau und Baro-Weiterleitung.
7. `DCUProviderPlugin`: Uplink-Zweig für `SerialMessageBaro`.
8. Rig-Test.

Nach Schritt 5 ist das Board am Tisch vollständig prüfbar; die Schritte 6 und 7
schließen die Kette zum Sim.

## Nicht in Scope

- `AltimeterDriver` wird nicht gelöscht.
- Der gekoppelte Modus (`COUPLED_MODE`) bleibt toter Code.
- Die Little-Endian-Eigenheit des Transponder-Frames `0x311` wird dokumentiert, nicht
  behoben — eine Korrektur träfe TransponderBoard und DCU gleichzeitig und erzwänge
  ein Flashen im Gleichschritt, ohne dass heute etwas kaputt wäre.
- Eine Stützstellentabelle für ein nichtlineares Baro-Poti.
- HSI, Annunciator, GNS 430 und Switchboard. Ihre IDs sind reserviert, mehr nicht.
