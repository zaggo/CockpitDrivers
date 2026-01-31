#ifndef TRANSPONDER_H
#define TRANSPONDER_H

#include "Arduino.h"
#include <TM1637Display.h>
#include "Configuration.h"
#include <MCP23017.h>

class Transponder
{
public:
    // Transponder mode (off=0, stdby=1, on (mode A)=2, alt (mode C)=3, test=4, GND (mode S)=5, ta_only (mode S)=6, ta/ra=7)
    enum TransponderMode
    {
        off = 0,
        stdby,
        on,
        alt,
        test,
        gnd,
        ta_only,
        ta_ra
    };

    enum TransponderError
    {
        no_error = 0,
        can_fail,
        can_gateway_timeout
    };

    enum TransponderButton
    {
        btn_ident = 15,
        btn_vfr = 14,
        btn_sby = 13,
        btn_on = 12,
        btn_alt = 11,
        btn_zero = 10,
        btn_one = 9,
        btn_two = 8,
        btn_three = 7,
        btn_four = 6,
        btn_five = 5,
        btn_six = 4,
        btn_seven = 3,
        btn_eight = 2,
        btn_nine = 1,
        btn_pwr = 0
    };

    Transponder();
    ~Transponder();

    void tick();

    void setIdent(bool ident)
    {
        identActive = ident;
        isSquawkEntryMode = false;
    }

    void setSquawkCode(const String squawk)
    {
        // Ignore incoming squawk code if user is currently entering one manually
        if (!isSquawkEntryMode)
        {
            currentSquawkCode = squawk;
        }
    }
    void setMode(TransponderMode mode) { currentMode = mode; }
    void setError(TransponderError error) { currentError = error; }
    void setBrightness(uint8_t level)
    {
        if (level > 7)
            level = 7;
        brightness = level;
    }
    void setTransponderLight(bool on)
    {
        transponderLightOn = on;
    }
    void setKeyBacklight(uint8_t level);

    void pressNumberButton(uint8_t button); // 0-9

    String getSquawkCode() { return currentSquawkCode; }
    TransponderMode getMode() { return currentMode; }
    uint8_t getBrightness() { return brightness; }

    // Singleton instance
    static Transponder *instance;

    volatile bool mcpInterrupt = false;

    bool identRequest = false;
    bool squawkCodeUpdated = false;
    bool modeUpdated = false;

private:
    void bufferSquawk(const String &squawk);
    void bufferMode(TransponderMode mode, bool identActive);
    void handleInterrupt();
    void didPressButton(TransponderButton button);
    void didReleaseButton(TransponderButton button);
    void commitSquawk();

#if BENCHDEBUG
    String buttonName(TransponderButton button);
#endif
private:
    MCP23017 *mcp;
    uint16_t currentButtonState = 0xffff;

    TransponderError currentError = no_error;

    uint8_t brightness = 3;
    bool transponderLightOn = false;

    uint32_t pwrButtonLongPressTimer = 0L;
    uint32_t commitTimer = 0L;
    uint32_t identTimer = 0L;

    String currentSquawkCode = "";

    TransponderMode currentMode = off;
    bool identActive = false;

    String displaySquakCode = "";
    TransponderMode displayMode = off;
    bool displayIdentActive = false;
    uint8_t displayBrightness = 0;
    bool displayTransponderLightOn = false;
    TransponderError displayError = no_error;

    bool isSquawkEntryMode = false;
    uint32_t squawkEntryModeTimer = 0L;
    uint32_t squawkEntryBlinkTimer = 0L;
    unsigned int squawkEntryPosition = 0; // 0-3

    String vfrSquawkCode = "1200";

    TM1637Display *display;
    uint8_t data[kLEDDigits] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    uint8_t offData[kLEDDigits] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // {SEG_G, SEG_G, SEG_G, SEG_G, SEG_G, SEG_G};

    uint32_t flashTimer = 0L;
    const uint32_t stbyFlashInterval = 1000L;          // 1 second
    const uint32_t identFlashInterval = 250L;          // 0.25 seconds
    const uint32_t squawkEntryTimeout = 3000L;         // 3 seconds
    const uint32_t squawkEntryBlinkInterval = 500L;    // 0.5 seconds
    const uint32_t pwrButtonLongPressDuration = 2000L; // 2 seconds

    const uint8_t squakIndex[4] = {0, 5, 4, 3}; // Indexes in data array for squawk digits

    const uint8_t SEG_SBY[2] = {
        SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, // S
        SEG_F | SEG_E | SEG_D | SEG_G | SEG_C, // b
    };
    const uint8_t SEG_ON[2] = {
        SEG_G | SEG_E | SEG_C | SEG_D, // o
        SEG_E | SEG_G | SEG_C,         // n
    };
    const uint8_t SEG_AL[2] = {
        SEG_A | SEG_F | SEG_B | SEG_G | SEG_E | SEG_C, // A
        SEG_F | SEG_E | SEG_D,                         // L
    };
    const uint8_t SEG_ID[2] = {
        SEG_B | SEG_C,                         // I
        SEG_B | SEG_G | SEG_E | SEG_C | SEG_D, // d
    };

    // Display "CANErr"
    const uint8_t SEG_CAN_FAIL[6] = {
        SEG_A | SEG_F | SEG_B | SEG_E | SEG_C,                         // N
        SEG_A | SEG_F | SEG_B | SEG_G | SEG_E | SEG_C, // A
        SEG_A | SEG_F | SEG_E | SEG_D,                 // C
        SEG_E | SEG_G,                                 // r
        SEG_E | SEG_G,                                 // r
        SEG_A | SEG_F | SEG_G | SEG_E | SEG_D,         // E
    };

    // Display "CANtou"
    const uint8_t SEG_CAN_TIMEOUT[6] = {
        SEG_A | SEG_F | SEG_B | SEG_E | SEG_C,                  // N
        SEG_A | SEG_F | SEG_B | SEG_G | SEG_E | SEG_C, // A
        SEG_A | SEG_F | SEG_E | SEG_D,                 // C
        SEG_E | SEG_D | SEG_C,                         // u
        SEG_G | SEG_E | SEG_C | SEG_D,                 // o
        SEG_F | SEG_G | SEG_E | SEG_D,                 // t
    };
};
#endif
