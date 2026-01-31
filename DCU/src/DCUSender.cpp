#include "DCUSender.h"
#include "DebugLog.h"

DCUSender::DCUSender()
{
    // Serial is already initialized by DCUReceiver, no need to initialize again
}

DCUSender::~DCUSender()
{
}

void DCUSender::sendTransponderInput(uint16_t code, uint8_t mode, uint8_t ident)
{
    // Payload: uint16_t code, uint8_t mode, uint8_t ident (4 bytes)
    uint8_t payload[4];
    payload[0] = static_cast<uint8_t>((code >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(code & 0xFF);
    payload[2] = mode;
    payload[3] = ident;

    sendFrame(MessageType::SerialMessageTransponder, sizeof(payload), payload);
    
    DEBUGLOG_PRINTLN(String(F("Sent Transponder Input: code ")) + String(code) + String(F(" mode ")) + String(mode) + String(F(" ident ")) + String(ident));
}

void DCUSender::sendFrame(MessageType type, uint8_t len, const uint8_t* payload)
{
    // Frame format: 0xAA 0x55 TYPE LEN PAYLOAD
    Serial.write(0xAA);
    Serial.write(0x55);
    Serial.write(static_cast<uint8_t>(type));
    Serial.write(len);
    
    for (uint8_t i = 0; i < len; ++i)
    {
        Serial.write(payload[i]);
    }
}
