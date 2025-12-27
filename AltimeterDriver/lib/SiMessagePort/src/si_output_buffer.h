#pragma once

#include <stdint.h>

#include "si_base.h"

#ifndef SI_EMBEDDED
#include "si_list.h"
#endif

#include "si_circular_data.h"

#define SI_OUTPUT_BUFFER_MODE1_HEADER_SIZE 4
#define SI_OUTPUT_BUFFER_MODE2_HEADER_SIZE 2

typedef size_t(*si_output_buffer_callback_type)(uint8_t* data, size_t len, void* tag);

enum SiOutputBufferMode {
	SI_OUTPUT_BUFFER_MODE1, // Each message has a unsigned int (4 bytes) proceeding it. No CRC checking
	SI_OUTPUT_BUFFER_MODE2, // Each message has a unique starting byte and a 1 byte len (max payload is 254 bytes)
    SI_OUTPUT_BUFFER_MODE3, // Each message starts with 0x00, and ends with 0xFF
	SI_OUTPUT_BUFFER_MODE4, // Each message starts with 0x00 and ends with 0xFF and is exact 6 bytes in length
	SI_OUTPUT_BUFFER_MODE5, // Each message has a unique 4 byte starting byte sequence and a 4 byte len (max payload is 4,294,967,295 bytes)
	SI_OUTPUT_BUFFER_MODE6  // String based message with fixed \n delimiter
};

#ifndef SI_EMBEDDED
struct SiOutputBufferMessage {
	uint8_t* data;
	size_t len;
	size_t pos;
};
#endif

struct SiOutputBuffer {
	enum SiOutputBufferMode mode;

#ifndef SI_EMBEDDED
	// Specific for mode 1 and 5
	si_output_buffer_callback_type callback;
	void* tag;

	// struct SiOutputBufferMessage
	struct SiList* message_queue;
#endif

	// Specific for mode 2 and 3
	struct SiCircularData* circular_data_buffer;
};

#ifndef SI_EMBEDDED
struct SiOutputBuffer* si_output_buffer_mode1_create(si_output_buffer_callback_type callback, void* tag);
size_t si_output_buffer_mode1_handle_data(struct SiOutputBuffer* output_buffer, size_t len);
void si_output_buffer_mode1_push_data(struct SiOutputBuffer* output_buffer, uint8_t* data, size_t len);
#endif

void si_output_buffer_mode2_init(struct SiOutputBuffer* output_buffer, struct SiCircularData* circulair_data_buffer);
enum SiResult si_output_buffer_mode2_push_data(const struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
uint8_t si_output_buffer_mode2_peek_data(struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
enum SiResult si_output_buffer_mode2_forward_data(struct SiOutputBuffer* output_buffer, uint8_t len);

void si_output_buffer_mode3_init(struct SiOutputBuffer* output_buffer, struct SiCircularData* circulair_data_buffer);
enum SiResult si_output_buffer_mode3_push_data(const struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
uint8_t si_output_buffer_mode3_peek_data(struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
enum SiResult si_output_buffer_mode3_forward_data(struct SiOutputBuffer* output_buffer, uint8_t len);

void si_output_buffer_mode4_init(struct SiOutputBuffer* output_buffer, struct SiCircularData* circulair_data_buffer);
enum SiResult si_output_buffer_mode4_push_data(const struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
uint8_t si_output_buffer_mode4_peek_data(struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len);
enum SiResult si_output_buffer_mode4_forward_data(struct SiOutputBuffer* output_buffer, uint8_t len);

#ifndef SI_EMBEDDED
struct SiOutputBuffer* si_output_buffer_mode5_create(si_output_buffer_callback_type callback, void* tag);
size_t si_output_buffer_mode5_handle_data(struct SiOutputBuffer* output_buffer, size_t len);
void si_output_buffer_mode5_push_data(struct SiOutputBuffer* output_buffer, uint8_t* data, size_t len);

struct SiOutputBuffer* si_output_buffer_mode6_create(si_output_buffer_callback_type callback, void* tag);
size_t si_output_buffer_mode6_handle_data(struct SiOutputBuffer* output_buffer, size_t len);
void si_output_buffer_mode6_push_data(struct SiOutputBuffer* output_buffer, const char* text);
#endif

void si_output_buffer_flush(struct SiOutputBuffer* output_buffer);
void si_output_buffer_close(struct SiOutputBuffer* output_buffer);
