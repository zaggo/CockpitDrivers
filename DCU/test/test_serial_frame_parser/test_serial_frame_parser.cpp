#include <unity.h>
#include <stddef.h>
#include "SerialFrameParser.h"

void setUp(void) {}
void tearDown(void) {}

static bool feedAll(SerialFrameParser& parser, const uint8_t* bytes, size_t n,
                     MessageType* outType, uint8_t* outLen, uint8_t* outPayload) {
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(bytes[i], outType, outLen, outPayload)) {
            got = true;
        }
    }
    return got;
}

void test_parses_a_valid_frame(void) {
    SerialFrameParser parser;
    uint8_t frame[] = {0xAA, 0x55, 0x01, 0x02, 0x10, 0x20};
    MessageType type;
    uint8_t len;
    uint8_t payload[SerialFrameParser::kMaxPayload];

    TEST_ASSERT_TRUE(feedAll(parser, frame, sizeof(frame), &type, &len, payload));
    TEST_ASSERT_EQUAL_UINT8(0x01, static_cast<uint8_t>(type));
    TEST_ASSERT_EQUAL_UINT8(2, len);
    TEST_ASSERT_EQUAL_UINT8(0x10, payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0x20, payload[1]);
}

void test_ignores_noise_before_sync(void) {
    SerialFrameParser parser;
    uint8_t frame[] = {0x00, 0xFF, 0xAA, 0x55, 0x02, 0x01, 0x99};
    MessageType type;
    uint8_t len;
    uint8_t payload[SerialFrameParser::kMaxPayload];

    TEST_ASSERT_TRUE(feedAll(parser, frame, sizeof(frame), &type, &len, payload));
    TEST_ASSERT_EQUAL_UINT8(0x02, static_cast<uint8_t>(type));
    TEST_ASSERT_EQUAL_UINT8(1, len);
    TEST_ASSERT_EQUAL_UINT8(0x99, payload[0]);
}

void test_oversized_length_resyncs_without_emitting_a_frame(void) {
    SerialFrameParser parser;
    // LEN=200 is invalid (> kMaxPayload); parser should drop back to
    // sync-hunting instead of emitting garbage or getting stuck.
    uint8_t garbage[] = {0xAA, 0x55, 0x01, 200};
    MessageType type;
    uint8_t len;
    uint8_t payload[SerialFrameParser::kMaxPayload];

    TEST_ASSERT_FALSE(feedAll(parser, garbage, sizeof(garbage), &type, &len, payload));

    // A subsequent valid frame must still parse correctly after the resync.
    uint8_t validFrame[] = {0xAA, 0x55, 0x03, 0x01, 0x7F};
    TEST_ASSERT_TRUE(feedAll(parser, validFrame, sizeof(validFrame), &type, &len, payload));
    TEST_ASSERT_EQUAL_UINT8(0x03, static_cast<uint8_t>(type));
    TEST_ASSERT_EQUAL_UINT8(1, len);
    TEST_ASSERT_EQUAL_UINT8(0x7F, payload[0]);
}

void test_max_length_payload_is_accepted(void) {
    SerialFrameParser parser;
    uint8_t frame[4 + SerialFrameParser::kMaxPayload];
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = 0x01;
    frame[3] = SerialFrameParser::kMaxPayload;
    for (uint8_t i = 0; i < SerialFrameParser::kMaxPayload; ++i) {
        frame[4 + i] = i;
    }
    MessageType type;
    uint8_t len;
    uint8_t payload[SerialFrameParser::kMaxPayload];

    TEST_ASSERT_TRUE(feedAll(parser, frame, sizeof(frame), &type, &len, payload));
    TEST_ASSERT_EQUAL_UINT8(SerialFrameParser::kMaxPayload, len);
    TEST_ASSERT_EQUAL_UINT8(0, payload[0]);
    TEST_ASSERT_EQUAL_UINT8(SerialFrameParser::kMaxPayload - 1, payload[SerialFrameParser::kMaxPayload - 1]);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_parses_a_valid_frame);
    RUN_TEST(test_ignores_noise_before_sync);
    RUN_TEST(test_oversized_length_resyncs_without_emitting_a_frame);
    RUN_TEST(test_max_length_payload_is_accepted);
    return UNITY_END();
}
