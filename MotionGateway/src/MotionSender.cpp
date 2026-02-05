#include "MotionSender.h"
#include "DebugLog.h"

MotionSender::MotionSender()
{
    // Serial is already initialized by MotionGateway, no need to initialize again
}

MotionSender::~MotionSender()
{
}

void MotionSender::sendFrame(MessageType type, uint8_t len, const uint8_t* payload)
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