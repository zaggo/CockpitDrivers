#include <Arduino.h>
#include "GyroDrive.h"
#include "Configuration.h"
#include "X25Motors.h"
#include <si_message_port.hpp>

SiMessagePort* messagePort;
GyroDrive gyroDrive;
SwitecX25* x25Steppers[kX25MotorCount];

#define DEBUG_MODE 0

#if DEBUG_MODE
int16_t pitchTargetDegrees = 0;
int16_t rollTargetDegrees = 0;
#else
enum MessageId {
  kTurnrate = 1,
  kSideslip = 2,
  kAirSpeed = 3,
  kVerticalSpeed = 4,
  kAttitudeIndicator = 5
};

static void new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
    if (payload == NULL) { return; }
    switch(message_id) {
      case kAirSpeed:
      case kTurnrate:
      case kSideslip:
      case kVerticalSpeed: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          float relPos = payload->data_float[0];
          // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received position: "+relPos+" for X25 motor: "+message_id);
          // Motor-ID prüfen und Position setzen
          if (message_id > 0 && message_id <= kX25MotorCount) {
            x25Steppers[message_id - 1]->setPosition(relPos * kX25TotalSteps);
          }
        }
        break;
      case kAttitudeIndicator: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          // uint8_t dataLength = payload->len;
          double rollToDegree = static_cast<double>(payload->data_float[0]);
          double pitchToDegree = static_cast<double>(payload->data_float[1]) * kAdjustmentFactor;
          // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"AttitudeIndicator (len="+dataLength+") roll: "+rollToDegree+" pitch: "+pitchToDegree);
          gyroDrive.moveToDegree(rollToDegree, pitchToDegree);
        }
        break;
    }
}
#endif

void setup() {
  #if DEBUG_MODE
    Serial.begin(9600);
    Serial.println("GyroDrive alive!");
  #else
    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560, SI_MESSAGE_PORT_CHANNEL_P, new_message_callback);
  #endif

  initX25Steppers();
  #if !DEBUG_MODE
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)kX25MotorCount + " Servos zeroed and driver ready");
  #endif

  gyroDrive.homeAllAxis();
  #if !DEBUG_MODE
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Attitude Indicator zeroed and driver ready");
  #endif

  #if DEBUG_MODE
    Serial.println("Gib eine Eingabe im Format 'o', 'p000', 'r000' oder 'p000 r000' ein:");
  #endif
}

#if DEBUG_MODE
void handleUserInput() {
  static String inputBuffer = ""; // Zwischenspeicher für serielle Eingaben

  while (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Einzelnes Zeichen lesen
    if (receivedChar == '\n') {        // Enter erkannt
      Serial.println(); // Neue Zeile
      inputBuffer.trim();              // Eingabe bereinigen (Leerzeichen etc.)
      
      if (inputBuffer.startsWith("o")) {
        gyroDrive.offAllAxes();
      } else if (inputBuffer.startsWith("h")) {
        gyroDrive.homeAllAxis();
      } else if (inputBuffer.startsWith("C")) {
        gyroDrive.moveSteps(kRollTotalSteps, 0);
      } else if (inputBuffer.startsWith("R")) { 
          String rString = inputBuffer.substring(1, inputBuffer.indexOf(" "));
          rString.trim();
          int32_t steps = rString.toInt();
          gyroDrive.moveSteps(steps, 0);
      } else if (inputBuffer.startsWith("p") || inputBuffer.startsWith("r")) { 
        int pTemp = pitchTargetDegrees;
        int rTemp = rollTargetDegrees;

        if (inputBuffer.startsWith("p")) {
          String pString = inputBuffer.substring(1, inputBuffer.indexOf(" "));
          pString.trim();
          pTemp = (int)((double)pString.toInt() * kAdjustmentFactor);

          // if (pTemp < -60 || pTemp > 60) {
          //   Serial.println("Fehler: 'p'-Wert muss zwischen -60 und 60 liegen.");
          //   inputBuffer = ""; // Buffer leeren
          //   return;
          // }
        }

        if (inputBuffer.indexOf("r") >= 0) {
          String rString = inputBuffer.substring(inputBuffer.indexOf("r") + 1);
          rString.trim();
          rTemp = rString.toInt();

          // if (rTemp < -360 || rTemp > 360) {
          //   Serial.println("Fehler: 'r'-Wert muss zwischen -360 und 360 liegen.");
          //   inputBuffer = ""; // Buffer leeren
          //   return;
          // }
        }

        gyroDrive.moveToDegree(rTemp, pTemp);
        pitchTargetDegrees = pTemp;
        rollTargetDegrees = rTemp;

        Serial.print("Aktuelle Werte: pitchTargetDegrees = ");
        Serial.print(pitchTargetDegrees);
        Serial.print(", rollTargetDegrees = ");
        Serial.println(rollTargetDegrees);
      } else {
        Serial.println("Fehler: Falsches Eingabeformat. Bitte 's', 'p000', 'r000' oder 'p000 r000' verwenden.");
      }

      inputBuffer = ""; // Buffer leeren
    } else {
      inputBuffer += receivedChar; // Zeichen an den Buffer anhängen
      Serial.print(receivedChar); // Eingabe zurückgeben
    }
  }
}
#endif

void loop() {
  #if DEBUG_MODE
    handleUserInput();
  #else
   messagePort->Tick();
  #endif

  updateAllX25Steppers();
  gyroDrive.runAllAxes();
}