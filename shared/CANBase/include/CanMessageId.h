#ifndef CAN_MESSAGE_ID_H
#define CAN_MESSAGE_ID_H

#include <Arduino.h>

// Message_IDs for CAN messages
enum class CanMessageId : uint16_t {
  // 0x100: Airspeed (Gateway -> Instruments, 50Hz)
  // [0..1] IAS  uint16, knots * 10 (sim/cockpit2/gauges/indicators/airspeed_kts_pilot)
  // [2..3] TAS  uint16, knots * 10, optional - the ASI's TAS ring is mechanical,
  //             so AirspeedCAN ignores these bytes
  // [4..7] reserved
  airspeed = 0x100,

  // 0x102: Altimeter + VSI (Gateway -> Instruments, 50Hz)
  // [0..3] altitude int32, feet     (sim/cockpit2/gauges/indicators/altitude_ft_pilot)
  // [4..5] vsi      int16, feet/min (sim/cockpit2/gauges/indicators/vvi_fpm_pilot)
  // [6..7] reserved
  // Both values are sent unscaled. One message feeds two boards: VerticalSpeedCAN
  // reads bytes 4..5 only, the altimeter board reads bytes 0..3 only.
  altimeterVsi = 0x102,

  // 0x107: Magnetic compass (Gateway -> Instrument, 50Hz)
  // [0..1] heading uint16, degrees * 100, 0..35999
  //        (sim/cockpit2/gauges/indicators/compass_heading_deg_mag)
  // [2..7] reserved
  compass = 0x107,

  // 0x104: HSI heading / OBS / bug / flags (Gateway -> Instrument, 50Hz)
  // [0..1] compass heading uint16, degrees * 100, 0..35999
  //        (sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot)
  // [2..3] OBS course    uint16, degrees * 100
  //        (sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot)
  // [4..5] HDG bug       uint16, degrees * 100
  //        (sim/cockpit2/autopilot/heading_dial_deg_mag_pilot)
  // [6]    flags uint8: bit0 TO, bit1 FROM, bit2 GS valid, bits 3..7 reserved.
  //        TO/FROM come from sim/cockpit/radios/nav1_fromto (1 = TO, 2 = FROM);
  //        GS valid is sim/cockpit/radios/nav1_CDI, whose name is misleading —
  //        Laminar documents it as "Are we receiving an expected glide slope".
  // [7]    reserved
  hsiHeading = 0x104,

  // 0x105: HSI deviation needles (Gateway -> Instrument, 10Hz)
  // [0..1] CDI deviation int16, dots * 1000 (sim/cockpit/radios/nav1_hdef_dot)
  // [2..3] GS  deviation int16, dots * 1000 (sim/cockpit/radios/nav1_vdef_dot)
  // [4..7] reserved
  // int16 at dots*1000 spans +-32.7 dots; nav1_hdef_dot reaches +-2.5. The
  // dots -> servo degree conversion happens on the board and is calibratable.
  hsiNav = 0x105,

  rpm = 0x106,

  odometer = 0x1F0,

  fuelLevel = 0x202,
  lights = 0x203,
  transponder = 0x201,

  // Heartbeats (Variante 2)
  // 0x300: Gateway heartbeat (Instrumente überwachen den DCU)
  gatewayHeartbeat = 0x300,

  // 0x301: Instrument heartbeat (DCU überwacht Instrumente; nodeId im Payload)
  instrumentHeartbeat = 0x301,

  // 0x303: Rudder pedals + toe brakes (Instrument -> DCU, onChange)
  // [0..1] rudder      int16,  -1000..1000 (sim/joystick/yoke_heading_ratio      * 1000)
  // [2..3] leftBrake   uint16,     0..1000 (sim/cockpit2/controls/left_brake_ratio  * 1000)
  // [4..5] rightBrake  uint16,     0..1000 (sim/cockpit2/controls/right_brake_ratio * 1000)
  // [6..7] reserved
  rudder = 0x303,

  transponderInput = 0x311,
  handbrakeStatus = 0x330,

  // 0x340..0x34F: cluster inputs (Instruments -> Gateway, onChange, plus an
  // unconditional resend every 5s). The periodic resend is load-bearing: it is
  // how the sim recovers the current knob position after a DCU or plugin
  // restart, since a restart has no prior onChange to react to.
  // The DCU covers this whole block with a single range filter (mask 0x7F0),
  // so new clusters in here need no filter change on the gateway.
  //
  // 0x340: Altimeter barometer knob
  // [0..1] baro   uint16, inHg * 100 (sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot)
  // [2]    unit   uint8, 0 = hPa, 1 = inHg — AltimeterCAN always sends 1
  // [3..7] reserved
  altimeterBaro = 0x340,

  // 0x341: HSI knob input (Instrument -> Gateway, onChange)
  // [0]    control   uint8, 0 = OBS knob, 1 = HDG knob
  // [1]    action    uint8, 0 = delta, 1 = push
  // [2..3] delta     int16 BE, encoder clicks, signed (0 when action = push)
  // [4]    pushState uint8, 0/1 (0 when action = delta)
  // [5..7] reserved
  //
  // EXCEPTION to the block rule above: this message carries *deltas*, so it
  // must NOT take part in the unconditional 5s resend. A resend would apply
  // the same clicks a second time. onChange only, never on a timer.
  // The board holds no absolute state: the OBS pointer follows the sim's own
  // value via 0x104 bytes 2..3, so the loop closes through X-Plane.
  hsiInput = 0x341
};

// Filtering
constexpr uint32_t CAN_STD_ID(CanMessageId id) {
  return static_cast<uint32_t>(id) << 16;
}

#endif // CAN_MESSAGE_ID_H