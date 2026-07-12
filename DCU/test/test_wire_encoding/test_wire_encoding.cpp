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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_packBE16_writes_high_byte_first);
    RUN_TEST(test_unpackBE16_reads_high_byte_first);
    RUN_TEST(test_packBE16_unpackBE16_roundtrip);
    RUN_TEST(test_fuel_level_kg100_matches_spec_layout);
    return UNITY_END();
}
