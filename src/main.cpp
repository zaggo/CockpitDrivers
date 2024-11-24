#include <Arduino.h>
#include "GyroDrive.h"
#include "Configuration.h"

GyroDrive gyroDrive;
int pitchTargetDegrees = 0;
int rollTargetDegrees = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("GyroDrive alive!");

  gyroDrive.homeAllAxis();

  Serial.println("Gib eine Eingabe im Format 'o', 'p000', 'r000' oder 'p000 r000' ein:");
}

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
          pTemp = pString.toInt();

          if (pTemp < -60 || pTemp > 60) {
            Serial.println("Fehler: 'p'-Wert muss zwischen -60 und 60 liegen.");
            inputBuffer = ""; // Buffer leeren
            return;
          }
        }

        if (inputBuffer.indexOf("r") >= 0) {
          String rString = inputBuffer.substring(inputBuffer.indexOf("r") + 1);
          rString.trim();
          rTemp = rString.toInt();

          if (rTemp < -360 || rTemp > 360) {
            Serial.println("Fehler: 'r'-Wert muss zwischen -360 und 360 liegen.");
            inputBuffer = ""; // Buffer leeren
            return;
          }
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

void loop() {
   handleUserInput();
   gyroDrive.runAllAxes();
}