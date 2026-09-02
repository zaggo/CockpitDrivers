#include <unity.h>
#include "WireEncoding.h"

void setUp(void) {}
void tearDown(void) {}

void test_packBE16_writes_high_byte_first(void) {
    uint8_t buf[2] = {0};
    packBE16(buf, 0x1234);
    TEST_ASSERT_EQUAL_UINT8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buf[1]);
}

void test_unpackBE16_reads_high_byte_first(void) {
    uint8_t buf[2] = {0xAB, 0xCD};
    TEST_ASSERT_EQUAL_UINT16(0xABCD, unpackBE16(buf));
}

void test_packBE16_unpackBE16_roundtrip(void) {
    uint8_t buf[2];
    packBE16(buf, 0xBEEF);
    TEST_ASSERT_EQUAL_UINT16(0xBEEF, unpackBE16(buf));
}

// Regression test for the BenchDebug fuel-encoding bug: a kg*100 value must
// survive the same big-endian uint16 round trip that FuelGaugeCAN decodes on
// the wire (CAN ID 0x202, bytes 0..1 = left tank, per the DCU CAN spec).
void test_fuel_level_kg100_matches_spec_layout(void) {
    uint8_t data[4] = {0, 0, 0, 0};
    uint16_t leftKg100 = static_cast<uint16_t>(123.45f * 100.f);
    uint16_t rightKg100 = static_cast<uint16_t>(67.0f * 100.f);
    packBE16(data + 0, leftKg100);
    packBE16(data + 2, rightKg100);

    TEST_ASSERT_EQUAL_UINT16(12345, unpackBE16(data + 0));
    TEST_ASSERT_EQUAL_UINT16(6700, unpackBE16(data + 2));
}

void test_packBE32_writes_most_significant_byte_first(void) {
    uint8_t buf[4] = {0};
    packBE32(buf, 0x12345678);
    TEST_ASSERT_EQUAL_UINT8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x78, buf[3]);
}

void test_unpackBE32_reads_most_significant_byte_first(void) {
    uint8_t buf[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, unpackBE32(buf));
}

void test_packBE32_unpackBE32_roundtrip(void) {
    uint8_t buf[4];
    packBE32(buf, 0xCAFEBABE);
    TEST_ASSERT_EQUAL_UINT32(0xCAFEBABE, unpackBE32(buf));
}

// The altimeter/VSI frame (CAN ID 0x102) packs two signed fields of different
// widths into one payload: [0..3] altitude int32 ft, [4..5] VSI int16 ft/min,
// [6..7] reserved. Both halves must survive the round trip with their sign
// intact — a descent below sea level exercises both at once.
void test_altimeter_vsi_frame_matches_spec_layout(void) {
    uint8_t data[8] = {0};
    const int32_t altitudeFt = -1200;
    const int16_t vsiFpm = -750;
    packBE32(data + 0, static_cast<uint32_t>(altitudeFt));
    packBE16(data + 4, static_cast<uint16_t>(vsiFpm));

    TEST_ASSERT_EQUAL_INT32(-1200, static_cast<int32_t>(unpackBE32(data + 0)));
    TEST_ASSERT_EQUAL_INT16(-750, static_cast<int16_t>(unpackBE16(data + 4)));
    TEST_ASSERT_EQUAL_UINT8(0, data[6]);
    TEST_ASSERT_EQUAL_UINT8(0, data[7]);
}

// Climbing, well above sea level: the positive side of the same layout.
void test_altimeter_vsi_frame_carries_positive_values(void) {
    uint8_t data[8] = {0};
    packBE32(data + 0, static_cast<uint32_t>(static_cast<int32_t>(35000)));
    packBE16(data + 4, static_cast<uint16_t>(static_cast<int16_t>(1800)));

    TEST_ASSERT_EQUAL_INT32(35000, static_cast<int32_t>(unpackBE32(data + 0)));
    TEST_ASSERT_EQUAL_INT16(1800, static_cast<int16_t>(unpackBE16(data + 4)));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_packBE16_writes_high_byte_first);
    RUN_TEST(test_unpackBE16_reads_high_byte_first);
    RUN_TEST(test_packBE16_unpackBE16_roundtrip);
    RUN_TEST(test_fuel_level_kg100_matches_spec_layout);
    RUN_TEST(test_packBE32_writes_most_significant_byte_first);
    RUN_TEST(test_unpackBE32_reads_most_significant_byte_first);
    RUN_TEST(test_packBE32_unpackBE32_roundtrip);
    RUN_TEST(test_altimeter_vsi_frame_matches_spec_layout);
    RUN_TEST(test_altimeter_vsi_frame_carries_positive_values);
    return UNITY_END();
}
