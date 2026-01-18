USB-Transportprotokoll Plugin ↔ DCU (Gateway)

Diese Datei beschreibt das aktuelle Transportprotokoll zwischen dem X-Plane Plugin (PC) und dem DCU/Gateway (Arduino). Sie bildet den Ist-Stand der Implementierung ab und ist bewusst einfach, robust und erweiterbar gehalten.

⸻

1. Zweck des Protokolls

Das USB-Protokoll verbindet zwei klar getrennte Verantwortungsbereiche:
	•	X-Plane Plugin (PC)
	•	Lesen/Schreiben von Datarefs & Commands
	•	Rate-Steuerung und Change-Detection
	•	Semantische Aufbereitung der Sim-Daten
	•	DCU / Gateway (Arduino)
	•	CAN-Scheduling und CAN-Failsafe
	•	Mapping auf Instrument-CAN-Frames
	•	Überwachung der Instrumente (Heartbeat)

Wichtig:
USB transportiert keine CAN-Frames, sondern semantische Zustandsnachrichten.

⸻

2. Physikalische Ebene

Eigenschaft	Wert
Medium	USB CDC (Serial)
Baudrate	115200
Richtung	Bidirektional
Blocking	Non-blocking / Best-effort
Endianness	Little Endian


⸻

3. Frame-Format

Alle Nachrichten sind binär geframed:

+--------+--------+--------+--------+-------------------+
| 0xAA   | 0x55   | Type   | Length | Payload (Length)  |
+--------+--------+--------+--------+-------------------+
   1B       1B        1B        1B        0..255 B

Bedeutung
	•	0xAA 0x55 – Synchronisationsmarker
	•	Type (u8) – Nachrichtentyp
	•	Length (u8) – Länge des Payloads in Bytes
	•	Payload – Typ-spezifische Nutzdaten

Aktuell kein CRC. USB + Framing + CAN-Heartbeat gelten als ausreichend. CRC kann später ergänzt werden.

⸻

4. Aktive Message-Types

4.1 0x01 – Fuel

Richtung: Plugin → Gateway
Rate: ca. 5 Hz (Testwert)

Payload (8 Byte):

float fuelLeft
float fuelRight

Semantik:
	•	Einheiten: Cockpit-Units (kg oder gal, wie im Sim)
	•	Gateway cached die Werte und mappt sie auf CAN (z. B. 0x202)

⸻

4.2 0x02 – Lights

Richtung: Plugin → Gateway
Rate: 2 Hz

Payload (8 Byte):

uint16 panelDim1000   // 0..1000
uint16 radioDim1000   // 0..1000
uint8  domeOn         // 0 = off, 1 = on
uint8  pad[3]         // 0

Datenquellen (Plugin):
	•	Panel Dim: sim/cockpit/electrical/instrument_brightness
	•	Radio Dim: sim/cockpit2/switches/instrument_brightness_ratio[0]
	•	Dome Light: sim/cockpit2/switches/panel_brightness_ratio[0] > 0

Gateway-Verhalten:
	•	Werte werden gecached
	•	Bei Änderung → CAN 0x203 (Lights)

⸻

5. Parsing im Gateway

RX-State-Machine
	•	Byte-weises Lesen
	•	Zustände:
	•	WAIT_AA → WAIT_55 → TYPE → LEN → PAYLOAD
	•	Robust gegen:
	•	Teillieferungen
	•	verlorene Bytes
	•	Resync mitten im Stream

Dispatch

switch (type) {
  case 0x01: handleFuel(payload);   break;
  case 0x02: handleLights(payload); break;
}

Neue Message-Types lassen sich durch weitere cases ergänzen.

⸻

6. Rate- und Filterstrategie

Plugin
	•	Kennt die Sim-Semantik
	•	Bestimmt Sendrate
	•	Führt Change-Detection durch

Gateway
	•	Bestimmt CAN-Sendrate
	•	minInterval pro CAN-ID
	•	Eigene Change-Detection
	•	Failsafe bei Ausfällen

→ Zweistufige Entkopplung (bewusstes Design)

⸻

7. Rückkanal (vorbereitet)

USB ist bidirektional, aktuell jedoch nur drainend genutzt.

Geplante Message-Types:

Type	Zweck
0x10	Input-Events (Encoder, Switches)
0x20	Status / Health (Heartbeat-Summary, Errors)
0x30	Commands / Konfiguration

Frame-Format bleibt unverändert.

⸻

8. Beziehung zu CAN-Heartbeat

Das USB-Protokoll verlässt sich auf die CAN-Heartbeat-Ebene:
	•	0x300 – Gateway Heartbeat (DCU → Instrumente)
	•	0x301 – Instrument Heartbeat (Instrumente → DCU)

Der Gateway überwacht Instrument-Health unabhängig vom PC.

→ USB benötigt aktuell keinen eigenen Heartbeat.

⸻

9. Design-Zusammenfassung
	•	USB transportiert semantische Zustände
	•	CAN transportiert instrumentennahe Realität
	•	Plugin und Gateway bleiben strikt entkoppelt
	•	Skalierbar auf 30–40+ Datarefs
	•	Robust gegen USB-Glitches
	•	Sehr gut debuggbar (socat / Hexdump)

⸻

Merksatz

USB beschreibt, was der Simulator meint.
CAN beschreibt, was das Cockpit tut.