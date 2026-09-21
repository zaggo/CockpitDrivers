#ifndef WHISKEYCOMPASS_H
#define WHISKEYCOMPASS_H
#include <Arduino.h>
#include <CheapStepper.h>
#include "Configuration.h"
#include "CompassGeometry.h"

class WhiskeyCompass
{
public:
    enum CompassResult {
        success = 0,
        notHomed,
        homingTimeout,
        cardStateReached
    };

    enum HomingPhase {
        unknown = 0,
        leaveZero,
        searchZero,
        searchZeroEnd,
        returnToZeroEnd,
        searchZeroStart,
        moveToTrueZero,
        moveToAdjustedZero,
        homed,
        timeout
    };

    WhiskeyCompass();

    // True once the card has found its Hall zero and moved onto the painted N
    // mark. Before that a heading setpoint means nothing.
    bool isHomed = false;

    void loop();

    // Starts a homing run that loop() drives forward. Unlike a blocking run this
    // returns immediately, so the CAN heartbeat keeps flowing in both directions
    // while the card searches — a run takes several seconds, well past the 1500ms
    // both ends use to declare each other dead.
    void beginHoming();
    bool isHoming() const { return homingActive; }

    // True from a failed homing run until the next beginHoming(). While set, the
    // panel lights blink instead of following the commanded brightness.
    bool hasHomingFailed() const { return homingFailed; }

    CompassResult moveToHeading(double degMag);

    // One brightness for all populated LEDs.
    void setBrightness(uint8_t brightness);

    // Records the card's current position as the painted N mark and persists it.
    // Must be homed first; the card is re-zeroed on success.
    bool calibrateZero();

    // Factory reset: zero offset back to the compiled-in default.
    void wipeCalibration();

    int16_t zeroAdjustDegree() const { return config.zeroAdjustDegree; }

    void stop();
    void off();

    int32_t position() const { return card->getPosition(); }

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree;
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    void applyLights(uint8_t brightness);
    void updateFailBlink();

    void fetchZeroedState();
    void moveSteps(int32_t steps);
    void moveDegree(int32_t degree);
    CompassResult lookForZeroChange(bool targetZeroedState);
    CompassResult nextHomingState();
    void runHomingStep();

    // Prints the phase the state machine just entered, plus the travel budget it
    // was given. Compiles away with DEBUGLOG_ENABLE off.
    void logHomingPhase();

    Config config;
    CheapStepper *card;

    HomingPhase homingState = unknown;
    uint8_t commandedBrightness = 0;
    bool homingFailed = false;
    bool failBlinkOn = false;
    uint32_t failBlinkLastToggleMs = 0;
    uint32_t zeroEndPosition = 0;
    bool zeroedState = false;
    bool homingActive = false;
};

#endif // WHISKEYCOMPASS_H
