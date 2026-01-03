#include "Transponder.h"
// #include <Wire.h>

Transponder *Transponder::instance = nullptr;
// ISR(PCINT1_vect) // Comp Interrupt
// {
//     if (Transponder::instance)
//     {
//         Transponder::instance->mcpInterrupt = true;
//     }
// }

Transponder::Transponder()
{
    instance = this;
    display = new TM1637Display(kTransponderClkPin, kTransponderDioPin); // Example CLK and DIO pins
    display->setBrightness(0x01);
    memcpy(data, offData, 6);
    display->setSegments(data, 6);

    // Setup zeroing pins & interrupts
    Wire.begin();

    mcp = new MCP23017(kMCP23017Address);
    mcp->init();
    mcp->portMode(MCP23017Port::A, 0xff); // Port A as input
    mcp->portMode(MCP23017Port::B, 0xff); // Port B as input

    mcp->writeRegister(MCP23017Register::GPPU_A, 0xFF);
    mcp->writeRegister(MCP23017Register::GPPU_B, 0xFF);

    mcp->interruptMode(MCP23017InterruptMode::Or);
    mcp->interrupt(MCP23017Port::A, CHANGE);
    mcp->interrupt(MCP23017Port::B, CHANGE);

    mcp->writeRegister(MCP23017Register::IPOL_A, 0x00);
    mcp->writeRegister(MCP23017Register::IPOL_B, 0x00);

    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00);
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00);
    mcp->clearInterrupts();

    pinMode(kMCP23017InterruptPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(kMCP23017InterruptPin), []()
                    {
        if (Transponder::instance)
        {
            Transponder::instance->mcpInterrupt = true;
        } }, CHANGE);

    // PCICR  = (1 << PCIE2);      // Gruppe PCINT[16..23] aktivieren
    // PCMSK2 = (1 << PCINT20);    // nur PCINT20 freigeben
}

Transponder::~Transponder()
{
    instance = NULL;
}

void Transponder::pressNumberButton(uint8_t button)
{
    if (button > 9 || currentMode == off)
        return; // Invalid button

    if (!isSquawkEntryMode)
    {
        // Enter squawk entry mode
        isSquawkEntryMode = true;
        squawkEntryBlinkTimer = millis() + squawkEntryBlinkInterval; // Blink after 0.5 seconds
        squawkEntryPosition = 0;
    }

    // Update squawk code at current position
    char c = '0' + button;
    currentSquawkCode.setCharAt(squawkEntryPosition, c);
    bufferSquawk(currentSquawkCode);

    // Move to next position
    squawkEntryPosition = (squawkEntryPosition + 1) % 4;
    squawkEntryModeTimer = millis() + squawkEntryTimeout; // Reset timeout
}

void Transponder::bufferSquawk(const String &squawk)
{
    if (squawk.length() != 4)
        return; // Invalid squawk code length

    for (int i = 0; i < 4; ++i)
    {
        char c = squawk.charAt(i);
        if (c >= '0' && c <= '9')
        {
            data[squakIndex[i]] = display->encodeDigit(c - '0');
        }
        else
        {
            data[squakIndex[i]] = 0x00; // Blank for non-digit characters
        }
    }
}

void Transponder::bufferMode(TransponderMode mode, bool identActive)
{
    if (identActive)
    {
        data[2] = SEG_ID[0];                        // 'I'
        data[1] = SEG_ID[1];                        // 'd'
        flashTimer = millis() + identFlashInterval; // Start flashing timer
        return;
    }
    switch (mode)
    {
    case stdby:
        data[2] = SEG_SBY[0];                      // 'S'
        data[1] = SEG_SBY[1];                      // 'b'
        flashTimer = millis() + stbyFlashInterval; // Start flashing timer
        return;
    case off:
        memcpy(data, offData, kLEDDigits); // All segments '-' (off)
        return;
    case on:
        data[2] = SEG_ON[0]; // 'O'
        data[1] = SEG_ON[1]; // 'n'
        return;
    case alt:
        data[2] = SEG_AL[0]; // 'A'
        data[1] = SEG_AL[1]; // 'L'
        return;
    default:
        return;
    }
}

void Transponder::tick()
{
    uint32_t now = millis();

    if (pwrButtonLongPressTimer != 0L && now > pwrButtonLongPressTimer)
    {
        // Long press detected
        pwrButtonLongPressTimer = 0L;
        if (currentMode != off)
        {
            Serial.println("Power Button Long Pressed: Turning OFF");
            setMode(off);
            commitSquawk();
            modeUpdated = true;
        }
        else
        {
            Serial.println("Power Button Long Pressed: Turning ON to STDBY");
            setMode(stdby);
            commitSquawk();
            modeUpdated = true;
        }
    }

    if (mcp)
    {
        if (mcpInterrupt)
        {
            // Serial.println("MCP23017 Interrupt detected");
            // mcpInterrupt = false;
            handleInterrupt();
        }
        else
        {
            mcp->clearInterrupts();
        }
    }

#if BENCHDEBUG
    if (identTimer != 0L && now > identTimer)
    {
        // IDENT duration expired
        setIdent(false);
        identTimer = 0L;
    }
#endif

    bool updated = false;
    if (commitTimer != 0L)
    {
        if (now < commitTimer)
        {
            return;
        }
        else
        {
            commitTimer = 0L;
            bufferSquawk(displaySquakCode);
            updated = true;
        }
    }

    if (displaySquakCode != currentSquawkCode)
    {
        displaySquakCode = currentSquawkCode;
        bufferSquawk(displaySquakCode);
        updated = true;
    }

    if (displayMode != currentMode || displayIdentActive != identActive)
    {
        bufferMode(currentMode, identActive);
        if (displayMode == off && currentMode != off)
        {
            bufferSquawk(displaySquakCode);
        }
        if (displayIdentActive && !identActive)
        {
            bufferSquawk(displaySquakCode);
        }
        displayMode = currentMode;
        displayIdentActive = identActive;
        updated = true;
    }

    if (displayIdentActive)
    {
        if (now > flashTimer)
        {
            // Flash the ident segments off and on every 0.25 seconds
            for (int i = 0; i < 4; ++i)
            {
                data[squakIndex[i]] ^= SEG_DP; // Toggle points
            }
            flashTimer = now + identFlashInterval;
            updated = true;
        }
    }
    else if (displayMode == stdby)
    {
        if (now > flashTimer)
        {
            // Flash the display off and on every second
            if (data[2] == SEG_SBY[0])
            {
                data[2] = 0x00; // Blank 'S'
                data[1] = 0x00; // Blank 'b
            }
            else
            {
                bufferMode(displayMode, displayIdentActive); // Restore 'Sb'
            }
            flashTimer = now + stbyFlashInterval;
            updated = true;
        }
    }

    if (isSquawkEntryMode)
    {
        // Handle squawk entry mode timing
        if (now > squawkEntryModeTimer)
        {
            commitSquawk();
        }
        else if (now > squawkEntryBlinkTimer)
        {
            // Blink the current position
            data[squakIndex[squawkEntryPosition]] ^= SEG_DP; // Toggle segments
            squawkEntryBlinkTimer = now + 500L;              // Blink every 0.5 seconds
            updated = true;
        }
    }

    if (transponderLightOn != displayTransponderLightOn && !identActive)
    {
        if (transponderLightOn)
        {
            data[1] |= SEG_DP;
        }
        else
        {
            data[1] &= ~SEG_DP;
        }
        displayTransponderLightOn = transponderLightOn;
        updated = true;
    }

    if (brightness != displayBrightness)
    {
        display->setBrightness(brightness);
        displayBrightness = brightness;
        updated = true;
    }

    if (updated)
    {
        display->setSegments(data, kLEDDigits);
    }
}

void Transponder::commitSquawk()
{
    if (!isSquawkEntryMode)
        return;
    isSquawkEntryMode = false;
    for (int i = 0; i < 4; ++i)
    {
        data[squakIndex[i]] = 0x00;
    }
    commitTimer = millis() + 200L; // Small delay before updating display
    display->setSegments(data, kLEDDigits);
    squawkCodeUpdated = true;
}

void Transponder::didPressButton(TransponderButton button)
{
    switch (button)
    {
    case btn_pwr:
        if (currentMode == off)
        {
            setMode(stdby);
            commitSquawk();
            modeUpdated = true;
        }
        else
        {
            pwrButtonLongPressTimer = millis() + kPwrButtonLongPressDuration;
        }
        break;
    case btn_zero:
    case btn_one:
    case btn_two:
    case btn_three:
    case btn_four:
    case btn_five:
    case btn_six:
    case btn_seven:
    case btn_eight:
    case btn_nine:
        pressNumberButton(10 - static_cast<uint8_t>(button));
        break;
    case btn_ident:
#if BENCHDEBUG
        setIdent(true);
        identTimer = millis() + 3000L; // IDENT active for 3 seconds
#else
        identRequest = true;
#endif
        break;
    case btn_vfr:
        currentSquawkCode = vfrSquawkCode;
        bufferSquawk(currentSquawkCode);
        commitSquawk();
        squawkCodeUpdated = true;
        break;
    case btn_sby:
        setMode(stdby);
        commitSquawk();
        modeUpdated = true;
        break;
    case btn_on:
        setMode(on);
        commitSquawk();
        modeUpdated = true;
        break;
    case btn_alt:
        setMode(alt);
        commitSquawk();
        modeUpdated = true;
        break;
    }
}

void Transponder::didReleaseButton(TransponderButton button)
{
    switch (button)
    {
    case btn_pwr:
        pwrButtonLongPressTimer = 0L;
        break;
    default:
        break;
    }
}

void Transponder::handleInterrupt()
{
    uint8_t captureA, captureB;
    mcp->clearInterrupts(captureA, captureB);
    mcpInterrupt = false;
    uint16_t newButtonState = static_cast<uint16_t>(captureA) << 8 | static_cast<uint16_t>(captureB);
    if (newButtonState != currentButtonState)
    {
        uint16_t changedButtons = currentButtonState ^ newButtonState;
        for (int i = 0; i < 16; ++i)
        {
            if (changedButtons & (static_cast<uint16_t>(1) << i))
            {
                TransponderButton button = static_cast<TransponderButton>(i);
                if (!(newButtonState & (static_cast<uint16_t>(1) << i)))
                {
                    didPressButton(button);
                }
                if (newButtonState & (static_cast<uint16_t>(1) << i))
                {
                    didReleaseButton(button);
                }
            }
        }
        currentButtonState = newButtonState;
    }
}

#if BENCHDEBUG
String Transponder::buttonName(Transponder::TransponderButton button)
{
    switch (button)
    {
    case btn_ident:
        return F("IDENT");
    case btn_vfr:
        return F("VFR");
    case btn_sby:
        return F("SBY");
    case btn_on:
        return F("ON");
    case btn_alt:
        return F("ALT");
    case btn_zero:
        return F("0");
    case btn_one:
        return F("1");
    case btn_two:
        return F("2");
    case btn_three:
        return F("3");
    case btn_four:
        return F("4");
    case btn_five:
        return F("5");
    case btn_six:
        return F("6");
    case btn_seven:
        return F("7");
    case btn_eight:
        return F("8");
    case btn_nine:
        return F("9");
    case btn_pwr:
        return F("PWR");
    default:
        return "UNKNOWN";
    }
}
#endif