#include "si_input_buffer.h"

#include "si_network.h"

#include <stdlib.h>
#include <string.h>

#define MODE1_LEN_SIZE 4  // Number of bytes of the len prefix

#define MODE2_HEADER_SIZE 2	 // Start tag and payload len
#define MODE2_START_BYTE  0xFF

#define MODE3_RCV_MESSAGE_LEN 25
#define MODE4_RCV_MESSAGE_LEN 6

#define MODE5_HEADER_SIZE	  8	 // Start tag and payload len
#define MODE5_LEN_SIZE		  4	 // Number of bytes of the len prefix
#define MODE5_START_BYTE_SIZE 4
#define MODE5_START_BYTE_SIGN 0xFF
#define MODE5_START_BYTE_SEQ  0xFFFFFFFF

#ifndef SI_EMBEDDED
struct SiInputBuffer* si_input_buffer_mode1_create(si_input_buffer_mode1_message_callback_type new_message_callback, void* tag) {
	struct SiInputBuffer* input_buffer = SI_BASE_MALLOC(struct SiInputBuffer);

	input_buffer->mode = SI_INPUT_BUFFER_MODE1;

	input_buffer->tag = tag;
	input_buffer->mode1_new_message_callback = new_message_callback;

	input_buffer->buffer = si_base_malloc(0);
	input_buffer->buffer_size = 0;
	input_buffer->buffer_pos = 0;

	input_buffer->len_mode_expected_len = 0;
	input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_LEN;

	return input_buffer;
}
#endif

struct SiInputBuffer* si_input_buffer_mode2_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag) {
	input_buffer->mode = SI_INPUT_BUFFER_MODE2;

	input_buffer->mode234_new_message_callback = new_message_callback;
	input_buffer->mode234_circular_data_buffer = circular_data_buffer;
	input_buffer->tag = tag;

	return input_buffer;
}

void si_input_buffer_mode2_set_callback(struct SiInputBuffer* input_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag) {
	input_buffer->mode234_new_message_callback = new_message_callback;
	input_buffer->tag = tag;
}

#ifndef SI_EMBEDDED
static void push_data_into_buffer(struct SiInputBuffer* input_buffer, uint8_t* data, size_t len) {
	// Check if we should increase buffer size
	if ((input_buffer->buffer_pos + len) > input_buffer->buffer_size) {
		input_buffer->buffer = (uint8_t*)si_base_realloc(input_buffer->buffer, input_buffer->buffer_pos + len);
		input_buffer->buffer_size = input_buffer->buffer_pos + len;
	}

	// Copy the data into the buffer
	memcpy(input_buffer->buffer + input_buffer->buffer_pos, data, len);

	// Increase position
	input_buffer->buffer_pos += len;
}

size_t si_input_buffer_mode1_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, size_t len) {
	size_t handled_bytes = 0;
	size_t output_bytes = 0;
	while (len > handled_bytes) {

		size_t remaining_bytes = (len - handled_bytes);

		switch (input_buffer->len_mode_state) {
		case LEN_MODE_STATE_WAITING_FOR_LEN: {
			size_t required_bytes_to_read = (MODE1_LEN_SIZE - input_buffer->buffer_pos);
			size_t bytes_to_read = required_bytes_to_read < remaining_bytes ? required_bytes_to_read : remaining_bytes;

			push_data_into_buffer(input_buffer, data + handled_bytes, bytes_to_read);
			handled_bytes += bytes_to_read;

			// Check if we have enough bytes for the LEN header
			if (input_buffer->buffer_pos >= MODE1_LEN_SIZE) {
				input_buffer->len_mode_expected_len = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)input_buffer->buffer);
				input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_DATA;
			}

			break;
		}
		case LEN_MODE_STATE_WAITING_FOR_DATA: {
			size_t required_bytes_to_read = (MODE1_LEN_SIZE + input_buffer->len_mode_expected_len - input_buffer->buffer_pos);
			size_t bytes_to_read = required_bytes_to_read < remaining_bytes ? required_bytes_to_read : remaining_bytes;

			push_data_into_buffer(input_buffer, data + handled_bytes, bytes_to_read);
			handled_bytes += bytes_to_read;

			if (input_buffer->buffer_pos >= (MODE1_LEN_SIZE + input_buffer->len_mode_expected_len)) {
				input_buffer->mode1_new_message_callback(input_buffer->buffer + MODE1_LEN_SIZE, (uint32_t)input_buffer->len_mode_expected_len, input_buffer->tag);
				output_bytes += input_buffer->len_mode_expected_len;

				input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_LEN;
				input_buffer->len_mode_expected_len = 0;
				input_buffer->buffer_pos = 0;
			}

			break;
		}
		default:
			/* Not all states need to be handled */
			break;
		}
	}
	return output_bytes;
}
#endif

enum SiResult si_input_buffer_mode2_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len) {
	if (si_circular_data_free(input_buffer->mode234_circular_data_buffer) < len) {
		return SI_ERROR;
	}
	else {
		uint8_t i;
		for (i = 0; i < len; i++) {
			si_circular_push(input_buffer->mode234_circular_data_buffer, data[i]);
		}
		return SI_OK;
	}
}

void si_input_buffer_mode2_evaluate(const struct SiInputBuffer* input_buffer) {
	uint8_t len;

	// The start of the circular buffer should always contain the start of a message
	// We will purge all non start byte data from the start of the buffer
	while (si_circular_data_available(input_buffer->mode234_circular_data_buffer) > 0) {
		uint8_t byte = 0;
		si_circular_peek(input_buffer->mode234_circular_data_buffer, 0, &byte);

		if (byte == MODE2_START_BYTE) {
			// The buffer now indeed starts with a message start tag, we are done
			break;
		}
		else {
			// Remove this byte from the buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);
		}
	}

	// Check if we can find any full message in the buffer
	// There might be more then one
	while (si_circular_data_available(input_buffer->mode234_circular_data_buffer) >= 2) {
		// Check if we have the start byte
		uint8_t start_byte;
		uint8_t payload_len_byte;

		si_circular_peek(input_buffer->mode234_circular_data_buffer, 0, &start_byte);
		si_circular_peek(input_buffer->mode234_circular_data_buffer, 1, &payload_len_byte);

		if ((start_byte == MODE2_START_BYTE) && (payload_len_byte != MODE2_START_BYTE)) {

			const uint8_t expected_message_len = (MODE2_HEADER_SIZE + payload_len_byte);

			// Check if the circular buffer is even big enough to house this message
			if (expected_message_len > si_circular_data_total_size(input_buffer->mode234_circular_data_buffer)) {
				si_circular_forward(input_buffer->mode234_circular_data_buffer, si_circular_data_available(input_buffer->mode234_circular_data_buffer));
			}

			// if the buffer contains at least '2 + payload len' number of bytes, we should have a complete message!
			if (si_circular_data_available(input_buffer->mode234_circular_data_buffer) >= expected_message_len) {

				// Count number of 0xFF in payload
				uint8_t i;
				for (i = 1; i < payload_len_byte; i++) {
					uint8_t prev_byte = 0;
					uint8_t byte = 0;

					si_circular_peek(input_buffer->mode234_circular_data_buffer, MODE2_HEADER_SIZE + i - 1, &prev_byte);
					si_circular_peek(input_buffer->mode234_circular_data_buffer, MODE2_HEADER_SIZE + i, &byte);

					if ((prev_byte == MODE2_START_BYTE) && (byte == MODE2_START_BYTE)) {
						// Poll away one of the 0xFF padding bytes
						si_circular_poll(input_buffer->mode234_circular_data_buffer, MODE2_HEADER_SIZE + i, NULL);

						// Payload just became one byte shorter, since we removed it
						payload_len_byte--;
					}
				}

				// Skip start and len bytes
				si_circular_forward(input_buffer->mode234_circular_data_buffer, MODE2_HEADER_SIZE);

				// Let the application know we have new data
				if (input_buffer->mode234_new_message_callback != NULL) {
					input_buffer->mode234_new_message_callback(input_buffer, payload_len_byte, input_buffer->tag);
				}

				// Skip payload bytes
				si_circular_forward(input_buffer->mode234_circular_data_buffer, payload_len_byte);
			}
			else {
				// There is still an uncompleted message in the buffer
				// We break from the while above and wait for new data to arrive
				break;
			}
		}
		else {
			// Illegal start sequence
			// Remove one of these bytes from the input buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);
		}
	}

	// We will find the last start tag starting from the back so we will find the last message
	// Any data before this last start tag is corrupted (since it was not removed by the code above)
	len = si_circular_data_available(input_buffer->mode234_circular_data_buffer);
	if (len > 3) {
		uint8_t i;
		for (i = (len - 2); i > 0; i--) {
			uint8_t prev_prev_byte = 0;
			uint8_t prev_byte = 0;
			uint8_t byte = 0;

			si_circular_peek(input_buffer->mode234_circular_data_buffer, i - 2, &prev_prev_byte);
			si_circular_peek(input_buffer->mode234_circular_data_buffer, i - 1, &prev_byte);
			si_circular_peek(input_buffer->mode234_circular_data_buffer, i, &byte);

			if ((prev_prev_byte != MODE2_START_BYTE) && (prev_byte == MODE2_START_BYTE) && (byte != MODE2_START_BYTE)) {
				// We have found the latest start tag!

				// Remove any data before this start tag (if there is any)
				if ((i - 1) > 0) {
					si_circular_forward(input_buffer->mode234_circular_data_buffer, i - 1);
				}
			}
		}
	}
}

struct SiInputBuffer* si_input_buffer_mode3_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag) {
	input_buffer->mode = SI_INPUT_BUFFER_MODE3;

	input_buffer->mode234_new_message_callback = new_message_callback;
	input_buffer->mode234_circular_data_buffer = circular_data_buffer;
	input_buffer->tag = tag;

	return input_buffer;
}

enum SiResult si_input_buffer_mode3_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len) {
	if (si_circular_data_free(input_buffer->mode234_circular_data_buffer) < len) {
		return SI_ERROR;
	}
	else {
		uint8_t i;
		for (i = 0; i < len; i++) {
			si_circular_push(input_buffer->mode234_circular_data_buffer, data[i]);
		}
		return SI_OK;
	}
}

void si_input_buffer_mode3_evaluate(const struct SiInputBuffer* input_buffer) {
	if (si_circular_data_available(input_buffer->mode234_circular_data_buffer) >= MODE3_RCV_MESSAGE_LEN) {
		// This might be a complete message

		// Get the first and last byte
		uint8_t first_byte, last_byte;
		si_circular_peek(input_buffer->mode234_circular_data_buffer, 0, &first_byte);
		si_circular_peek(input_buffer->mode234_circular_data_buffer, MODE3_RCV_MESSAGE_LEN - 1, &last_byte);

		// Are the first and last byte okay?
		if ((first_byte == 0x00) && (last_byte == 0xFF)) {
			// Remove first byte
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);

			// Let the application know we have new data
			if (input_buffer->mode234_new_message_callback != NULL) {
				input_buffer->mode234_new_message_callback(input_buffer, MODE3_RCV_MESSAGE_LEN - 2, input_buffer->tag);
			}

			// Remove the remaining message from input buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, MODE3_RCV_MESSAGE_LEN - 1);
		}
		else if (first_byte == 0x00) {
			// Something is wrong, there is an incomplete message in the buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);
		}
	}

	// Strip all preceeding bytes that are not 0x00, since each message should start with 0x00
	for (uint8_t i = 0; i < si_circular_data_available(input_buffer->mode234_circular_data_buffer); i++) {
		uint8_t byte;
		si_circular_peek(input_buffer->mode234_circular_data_buffer, i, &byte);

		if (byte != 0x00) {
			// This byte is not 0x00, should be removed
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);

			// Start again from begin
			i = -1;
		}
		else {
			// Byte is 0x00, we are done
			break;
		}
	}
}

struct SiInputBuffer* si_input_buffer_mode4_init(struct SiInputBuffer* input_buffer, struct SiCircularData* circular_data_buffer, si_input_buffer_mode234_message_callback_type new_message_callback, void* tag) {
	input_buffer->mode = SI_INPUT_BUFFER_MODE3;

	input_buffer->mode234_new_message_callback = new_message_callback;
	input_buffer->mode234_circular_data_buffer = circular_data_buffer;
	input_buffer->tag = tag;

	return input_buffer;
}

enum SiResult si_input_buffer_mode4_push_data(const struct SiInputBuffer* input_buffer, const uint8_t* data, const uint8_t len) {
	if (si_circular_data_free(input_buffer->mode234_circular_data_buffer) < len) {
		return SI_ERROR;
	}
	else {
		uint8_t i;
		for (i = 0; i < len; i++) {
			si_circular_push(input_buffer->mode234_circular_data_buffer, data[i]);
		}
		return SI_OK;
	}
}

void si_input_buffer_mode4_evaluate(const struct SiInputBuffer* input_buffer) {
	if (si_circular_data_available(input_buffer->mode234_circular_data_buffer) >= MODE4_RCV_MESSAGE_LEN) {
		// This might be a complete message

		// Get the first and last byte
		uint8_t first_byte, last_byte;
		si_circular_peek(input_buffer->mode234_circular_data_buffer, 0, &first_byte);
		si_circular_peek(input_buffer->mode234_circular_data_buffer, MODE4_RCV_MESSAGE_LEN - 1, &last_byte);

		// Are the first and last byte okay?
		if ((first_byte == 0x00) && (last_byte == 0xFF)) {
			// Remove first byte
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);

			// Let the application know we have new data
			if (input_buffer->mode234_new_message_callback != NULL) {
				input_buffer->mode234_new_message_callback(input_buffer, MODE4_RCV_MESSAGE_LEN - 2, input_buffer->tag);
			}

			// Remove the remaining message from input buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, MODE4_RCV_MESSAGE_LEN - 1);
		}
		else if (first_byte == 0x00) {
			// Something is wrong, there is an incomplete message in the buffer
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);
		}
	}

	// Strip all preceding bytes that are not 0x00, since each message should start with 0x00
	for (uint8_t i = 0; i < si_circular_data_available(input_buffer->mode234_circular_data_buffer); i++) {
		uint8_t byte;
		si_circular_peek(input_buffer->mode234_circular_data_buffer, i, &byte);

		if (byte != 0x00) {
			// This byte is not 0x00, should be removed
			si_circular_forward(input_buffer->mode234_circular_data_buffer, 1);

			// Start again from begin
			i = -1;
		}
		else {
			// Byte is 0x00, we are done
			break;
		}
	}
}

#ifndef SI_EMBEDDED
struct SiInputBuffer* si_input_buffer_mode5_create(si_input_buffer_mode5_message_callback_type new_message_callback, void* tag) {
	struct SiInputBuffer* input_buffer = SI_BASE_MALLOC(struct SiInputBuffer);

	input_buffer->mode = SI_INPUT_BUFFER_MODE5;

	input_buffer->tag = tag;
	input_buffer->mode5_new_message_callback = new_message_callback;

	input_buffer->buffer = si_base_malloc(0);
	input_buffer->buffer_size = 0;
	input_buffer->buffer_pos = 0;

	input_buffer->len_mode_expected_len = 0;
	input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_START_TAG;

	return input_buffer;
}

void si_input_buffer_mode5_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, const size_t len) {
	const uint32_t start_seq = MODE5_START_BYTE_SEQ;
	const uint32_t start_seq_2[2] = {MODE5_START_BYTE_SEQ, MODE5_START_BYTE_SEQ};

	size_t handled_bytes = 0;
	while (len > handled_bytes) {

		size_t remaining_bytes = (len - handled_bytes);

		switch (input_buffer->len_mode_state) {
		case LEN_MODE_STATE_WAITING_FOR_START_TAG: {
			size_t required_bytes_to_read = (MODE5_START_BYTE_SIZE - input_buffer->buffer_pos);
			size_t bytes_to_read = required_bytes_to_read < remaining_bytes ? required_bytes_to_read : remaining_bytes;

			push_data_into_buffer(input_buffer, data + handled_bytes, bytes_to_read);
			handled_bytes += bytes_to_read;

			// Check if we have enough bytes for the start header
			if (input_buffer->buffer_pos >= MODE5_LEN_SIZE) {
				if (memcmp(input_buffer->buffer, &start_seq, MODE5_LEN_SIZE) == 0) {
					input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_LEN;
				}
				else {
					// Start of buffer is not the start tag. We will remove all non-leading 0xFF signs

					// Count number of non 0xFF signs
					size_t number_of_non_ff_found = 0;
					for (size_t i = 0; i < input_buffer->buffer_size; i++) {
						if (input_buffer->buffer[i] == MODE5_START_BYTE_SIGN) {
							break;
						}
						else {
							number_of_non_ff_found++;
						}
					}

					// Revert buffer back to get rid of non 0xFF
					memmove(input_buffer->buffer, input_buffer->buffer + number_of_non_ff_found, number_of_non_ff_found);
					input_buffer->buffer_pos -= number_of_non_ff_found;
				}
			}
			break;
		}

		case LEN_MODE_STATE_WAITING_FOR_LEN: {
			size_t required_bytes_to_read = (MODE5_HEADER_SIZE - input_buffer->buffer_pos);
			size_t bytes_to_read = required_bytes_to_read < remaining_bytes ? required_bytes_to_read : remaining_bytes;

			push_data_into_buffer(input_buffer, data + handled_bytes, bytes_to_read);
			handled_bytes += bytes_to_read;

			// Check if we have enough bytes for the LEN header
			if (input_buffer->buffer_pos >= MODE5_HEADER_SIZE) {
				if (memcmp(input_buffer->buffer + MODE5_START_BYTE_SIZE, &start_seq, MODE5_LEN_SIZE) == 0) {
					// Unexpected start sequence found

					// Revert buffer back to get rid of non 0xFF
					memmove(input_buffer->buffer, input_buffer->buffer + MODE5_HEADER_SIZE, MODE5_HEADER_SIZE);
					input_buffer->buffer_pos -= MODE5_HEADER_SIZE;

					input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_START_TAG;
				}
				else {
					input_buffer->len_mode_expected_len = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)(input_buffer->buffer + MODE5_START_BYTE_SIZE));
					input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_DATA;
				}
			}

			break;
		}
		case LEN_MODE_STATE_WAITING_FOR_DATA: {
			size_t required_bytes_to_read = (MODE5_HEADER_SIZE + input_buffer->len_mode_expected_len - input_buffer->buffer_pos);
			size_t bytes_to_read = required_bytes_to_read < remaining_bytes ? required_bytes_to_read : remaining_bytes;

			push_data_into_buffer(input_buffer, data + handled_bytes, bytes_to_read);
			handled_bytes += bytes_to_read;

			if (input_buffer->buffer_pos >= (MODE5_HEADER_SIZE + input_buffer->len_mode_expected_len)) {
				// Remove duplicate start sequences
				size_t real_len = input_buffer->len_mode_expected_len;
				uint8_t* payload_data = input_buffer->buffer + MODE5_HEADER_SIZE;

				if (input_buffer->len_mode_expected_len >= (MODE5_START_BYTE_SIZE * 2)) {
					for (size_t i = 0; i < input_buffer->len_mode_expected_len - (MODE5_START_BYTE_SIZE * 2) + 1; i++) {

						// Check for start sequence
						if (memcmp(payload_data + i, start_seq_2, sizeof(uint32_t) * 2) == 0) {
							memmove(payload_data + i, payload_data + i + MODE5_START_BYTE_SIZE, input_buffer->len_mode_expected_len - i - MODE5_START_BYTE_SIZE);
							i += MODE5_START_BYTE_SIZE - 1;

							real_len -= MODE5_START_BYTE_SIZE;
						}
					}
				}

				input_buffer->mode5_new_message_callback(input_buffer, payload_data, (uint32_t)real_len, input_buffer->tag);

				input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_LEN;
				input_buffer->buffer_pos = 0;
			}

			break;
		}
		}
	}
}

struct SiInputBuffer* si_input_buffer_mode6_create(si_input_buffer_mode6_message_callback_type new_message_callback, void* tag) {
	struct SiInputBuffer* input_buffer = SI_BASE_MALLOC(struct SiInputBuffer);

	input_buffer->mode = SI_INPUT_BUFFER_MODE6;

	input_buffer->tag = tag;
	input_buffer->mode6_new_message_callback = new_message_callback;

	input_buffer->buffer = si_base_malloc(0);
	input_buffer->buffer_size = 0;
	input_buffer->buffer_pos = 0;

	input_buffer->len_mode_expected_len = 0;
	input_buffer->len_mode_state = LEN_MODE_STATE_WAITING_FOR_START_TAG;

	return input_buffer;
}

void si_input_buffer_mode6_push_data(struct SiInputBuffer* input_buffer, uint8_t* data, const size_t len) {
	push_data_into_buffer(input_buffer, data, len);

	for (size_t i = 0; i < input_buffer->buffer_pos; i++) {
		if (input_buffer->buffer[i] == '\n') {
			input_buffer->buffer[i] = '\0';

			input_buffer->mode6_new_message_callback(input_buffer, (const char*)input_buffer->buffer, input_buffer->tag);

			// Move all data back to the front of the buffer
			const size_t message_size = i + 1;
			input_buffer->buffer_pos -= message_size;
			memmove(input_buffer->buffer, input_buffer->buffer + message_size, input_buffer->buffer_pos);

			i = -1;	 // -1 == 0 in c for loops
		}
	}
}

#endif	// SI_EMBEDDED

void si_input_buffer_flush(struct SiInputBuffer* input_buffer) {

	switch (input_buffer->mode) {
#ifndef SI_EMBEDDED
	case SI_INPUT_BUFFER_MODE1:
	case SI_INPUT_BUFFER_MODE5:
	case SI_INPUT_BUFFER_MODE6:
		input_buffer->buffer = si_base_realloc(input_buffer->buffer, 1);
		input_buffer->buffer_size = 1;
		input_buffer->buffer_pos = 0;
		input_buffer->len_mode_expected_len = 0;
		input_buffer->len_mode_state = input_buffer->mode == SI_INPUT_BUFFER_MODE1 ? LEN_MODE_STATE_WAITING_FOR_LEN : LEN_MODE_STATE_WAITING_FOR_START_TAG;
		break;
#endif

	case SI_INPUT_BUFFER_MODE2:
	case SI_INPUT_BUFFER_MODE3:
	case SI_INPUT_BUFFER_MODE4:
		si_circular_forward(input_buffer->mode234_circular_data_buffer, si_circular_data_available(input_buffer->mode234_circular_data_buffer));
		break;

	default:
		break;
	}
}

void si_input_buffer_close(struct SiInputBuffer* input_buffer) {
	assert(input_buffer != NULL);
	// TODO: Implement
}
