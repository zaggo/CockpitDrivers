#ifndef SI_INPUT_BUFFER_H
#define SI_INPUT_BUFFER_H

#include <stdint.h>

#include "si_base.h"
#include "si_circular_data.h"

#ifdef __cplusplus
extern "C"{
#endif

#define SI_INPUT_BUFFER_MODE1_HEADER_SIZE 4
#define SI_INPUT_BUFFER_MODE2_HEADER_SIZE 2
#define SI_INPUT_BUFFER_MODE5_HEADER_SIZE 8

struct SiInputBuffer;

typedef void (*si_input_buffer_mode1_message_callback_type)(uint8_t* data, size_t len, const void* tag);
typedef void (*si_input_buffer_mode234_message_callback_type)(const struct SiInputBuffer* input_buffer, const uint8_t len, const void* tag);
typedef void (*si_input_buffer_mode5_message_callback_type)(const struct SiInputBuffer* input_buffer, uint8_t* data, const uint32_t len, const void* tag);
typedef void (*si_input_buffer_mode6_message_callback_type)(const struct SiInputBuffer* input_buffer, const char* text, const void* tag);

enum SiInputBufferMode {
	SI_INPUT_BUFFER_MODE1, // Each message has a unsigned int (4 bytes) proceeding it. No CRC checking
	SI_INPUT_BUFFER_MODE2, // Each message has a unique starting byte and a 1 byte len (max payload is 254 bytes)
    SI_INPUT_BUFFER_MODE3, // Each message starts with 0x00 and ends with 0xFF and is exact 25 bytes in length
	SI_INPUT_BUFFER_MODE4, // Each message starts with 0x00 and ends with 0xFF and is exact 6 bytes in length
	SI_INPUT_BUFFER_MODE5, // Each message has a unique 4 byte starting byte sequence and a 4 byte len (max payload is 4,294,967,295 bytes)
	SI_INPUT_BUFFER_MODE6  // String based message with fixed \n delimiter
};

enum SiInputBufferLenModeState {
	LEN_MODE_STATE_WAITING_FOR_START_TAG,
	LEN_MODE_STATE_WAITING_FOR_LEN,
	LEN_MODE_STATE_WAITING_FOR_DATA
};

enum SiInputBufferStartLenModeState {
	START_LEN_MODE_STATE_WAITING_FOR_START_TAG,
	START_LEN_MODE_STATE_WAITING_FOR_LEN,
	START_LEN_MODE_STATE_WAITING_FOR_PAYLOAD
};

struct SiInputBuffer {
	enum SiInputBufferMode mode;

#ifndef SI_EMBEDDED
	// Specific for mode 1 and 5
    uint8_t* buffer;
	size_t buffer_size;
	size_t buffer_pos;

	size_t len_mode_expected_len;
	enum SiInputBufferLenModeState len_mode_state;

	si_input_buffer_mode1_message_callback_type mode1_new_message_callback;
	si_input_buffer_mode5_message_callback_type mode5_new_message_callback;
	si_input_buffer_mode6_message_callback_type mode6_new_message_callback;
#endif

	// Specific for mode 2, 3, 4
	struct SiCircularData* mode234_circular_data_buffer;
	
	si_input_buffer_mode234_message_callback_type mode234_new_message_callback;
	
	// All modes
	const void* tag;
};

// Mode 1
#ifndef SI_EMBEDDED
struct SiInputBuffer* si_input_buffer_mode1_create(si_input_buffer_mode1_message_callback_type new_message_callback, void* tag);
size_t si_input_buffer_mode1_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, size_t len);
#endif

// Mode 2
struct SiInputBuffer* si_input_buffer_mode2_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag);
void si_input_buffer_mode2_set_callback(struct SiInputBuffer* input_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag);
enum SiResult si_input_buffer_mode2_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len);
void si_input_buffer_mode2_evaluate(const struct SiInputBuffer* input_buffer);

// Mode 3
struct SiInputBuffer* si_input_buffer_mode3_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag);
enum SiResult si_input_buffer_mode3_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len);
void si_input_buffer_mode3_evaluate(const struct SiInputBuffer* input_buffer);

// Mode 4
struct SiInputBuffer* si_input_buffer_mode4_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag);
enum SiResult si_input_buffer_mode4_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len);
void si_input_buffer_mode4_evaluate(const struct SiInputBuffer* input_buffer);

#ifndef SI_EMBEDDED
// Mode 5
struct SiInputBuffer* si_input_buffer_mode5_create(si_input_buffer_mode5_message_callback_type new_message_callback, void* tag);
void si_input_buffer_mode5_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, const size_t len);

// Mode 6
struct SiInputBuffer* si_input_buffer_mode6_create(si_input_buffer_mode6_message_callback_type new_message_callback, void* tag);
void si_input_buffer_mode6_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, const size_t len);
#endif

void si_input_buffer_flush(struct SiInputBuffer* input_buffer);
void si_input_buffer_close(struct SiInputBuffer* input_buffer);

#ifdef __cplusplus
}
#endif

#endif
