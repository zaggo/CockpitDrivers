#include <unity.h>
#include <string.h>
#include "CommandTokenizer.h"

void setUp(void) {}
void tearDown(void) {}

void test_tokenizeCommands_single_token(void) {
    char buf[32];
    strcpy(buf, "lt10");
    char* tokens[10];
    size_t count = tokenizeCommands(buf, tokens, 10);
    TEST_ASSERT_EQUAL_UINT(1, count);
    TEST_ASSERT_EQUAL_STRING("lt10", tokens[0]);
}

void test_tokenizeCommands_two_tokens(void) {
    char buf[32];
    strcpy(buf, "lt 10");
    char* tokens[10];
    size_t count = tokenizeCommands(buf, tokens, 10);
    TEST_ASSERT_EQUAL_UINT(2, count);
    TEST_ASSERT_EQUAL_STRING("lt", tokens[0]);
    TEST_ASSERT_EQUAL_STRING("10", tokens[1]);
}

void test_tokenizeCommands_collapses_repeated_and_edge_spaces(void) {
    char buf[32];
    strcpy(buf, "  lt   10  ");
    char* tokens[10];
    size_t count = tokenizeCommands(buf, tokens, 10);
    TEST_ASSERT_EQUAL_UINT(2, count);
    TEST_ASSERT_EQUAL_STRING("lt", tokens[0]);
    TEST_ASSERT_EQUAL_STRING("10", tokens[1]);
}

void test_tokenizeCommands_empty_input(void) {
    char buf[32];
    strcpy(buf, "");
    char* tokens[10];
    size_t count = tokenizeCommands(buf, tokens, 10);
    TEST_ASSERT_EQUAL_UINT(0, count);
}

void test_tokenizeCommands_caps_at_maxTokens(void) {
    char buf[32];
    strcpy(buf, "a b c d e");
    char* tokens[3];
    size_t count = tokenizeCommands(buf, tokens, 3);
    TEST_ASSERT_EQUAL_UINT(3, count);
    TEST_ASSERT_EQUAL_STRING("a", tokens[0]);
    TEST_ASSERT_EQUAL_STRING("b", tokens[1]);
    TEST_ASSERT_EQUAL_STRING("c", tokens[2]);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_tokenizeCommands_single_token);
    RUN_TEST(test_tokenizeCommands_two_tokens);
    RUN_TEST(test_tokenizeCommands_collapses_repeated_and_edge_spaces);
    RUN_TEST(test_tokenizeCommands_empty_input);
    RUN_TEST(test_tokenizeCommands_caps_at_maxTokens);
    return UNITY_END();
}
