#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0
#define DEBUGLOG_ENABLE 0

const CanNodeId kNodeId = CanNodeId::starterNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately: D10 is the AVR /SS pin and must be an output in
// master mode, so making it the chip select lets SPI.begin() drive it.
// /INT must be D2 or D3 (INT0/INT1), BaseCAN hangs its RX handler off
// attachInterrupt().
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// Starter LED. D3 is Timer2 PWM; analogWrite there leaves Timer0 and thus
// millis()/micros() alone. Driven with a static brightness so a dimmer value
// than full on can be baked in here.
const uint8_t kStarterLedPin = 3;
const uint8_t kStarterLedBrightness = 128;

// Magneto/starter rotary switch, one pin per detent, internal pull-ups, the
// active position is pulled to GND. Array order is the wire order of CAN 0x345
// byte 0: 0 = OFF, 1 = R, 2 = L, 3 = BOTH, 4 = START.
const uint8_t kSwitchPositionCount = 5;
const uint8_t kSwitchPins[kSwitchPositionCount] = {4, 5, 6, 7, 8};

// A new detent must read stable for this long before it is reported. Between
// detents no pin is grounded at all; the last position is kept then.
const uint16_t kSwitchDebounceMs = 20;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
