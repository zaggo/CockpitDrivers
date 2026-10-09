#include "si_output_buffer.h"
#include "si_network.h"

#ifndef SI_EMBEDDED
#include "si_string.h"
#endif

#define MODE1_LEN_SIZE	  4	 // Number of bytes of the len prefix
#define MODE2_HEADER_SIZE 2	 // Start tag and payload len

#define MODE3_OVERHEAD_SIZE 2  // Start tag and end tag

#define MODE5_HEADER_SIZE	  8	 // Start tag and payload len
#define MODE5_LEN_SIZE		  4	 // Number of bytes of the len prefix
#define MODE5_START_BYTE_SIZE 4
#define MODE5_START_BYTE_SEQ  0xFFFFFFFF

#define MODE6_POSTFIX_LEN 2	 // '\n' + '\0'

#ifndef SI_EMBEDDED
struct SiOutputBuffer* si_output_buffer_mode1_create(si_output_buffer_callback_type callback, void* tag) {
	struct SiOutputBuffer* output_buffer = SI_BASE_MALLOC(struct SiOutputBuffer);

	output_buffer->callback = callback;
	output_buffer->tag = tag;

	output_buffer->mode = SI_OUTPUT_BUFFER_MODE1;
	output_buffer->message_queue = si_list_create();

	return output_buffer;
}

void si_output_buffer_mode1_push_data(struct SiOutputBuffer* output_buffer, uint8_t* data, size_t len) {
	struct SiOutputBufferMessage* message = SI_BASE_MALLOC(struct SiOutputBufferMessage);

	message->data = (uint8_t*)si_base_malloc(len + MODE1_LEN_SIZE);

	uint32_t len_value = SI_NETWORK_UINT32_TO_NETWORK((uint32_t)len);

	memcpy(message->data, &len_value, MODE1_LEN_SIZE);
	memcpy(message->data + MODE1_LEN_SIZE, data, len);

	message->len = len + MODE1_LEN_SIZE;
	message->pos = 0;

	si_list_append(output_buffer->message_queue, message);
}

static size_t message_handle_data(struct SiOutputBuffer* output_buffer, struct SiOutputBufferMessage* message, size_t len) {
	size_t handled_bytes = 0;
	do {
		size_t bytes_in_message = (message->len - message->pos);
		size_t remaining_bytes_to_sent = (len - handled_bytes);
		size_t bytes_to_sent = bytes_in_message < remaining_bytes_to_sent ? bytes_in_message : remaining_bytes_to_sent;

		size_t bytes_sent = output_buffer->callback(message->data + message->pos, bytes_to_sent, output_buffer->tag);
		if (!bytes_sent) {
			break;
		}
		message->pos += bytes_sent;
		handled_bytes += bytes_sent;
	} while (handled_bytes < len && message->pos < message->len);
	return handled_bytes;
}

size_t si_output_buffer_mode1_handle_data(struct SiOutputBuffer* output_buffer, size_t len) {
	size_t handled_bytes = 0;

	// Iterate through our message queue to find and sent the requested number of bytes
	SI_LIST_FOREACH(struct SiOutputBufferMessage*, message, output_buffer->message_queue) {
		size_t bytes_sent = message_handle_data(output_buffer, message, len - handled_bytes);
		if (!bytes_sent) {
			break;
		}

		handled_bytes += bytes_sent;

		// Clean up message when we have successfully sent it
		if (message->pos >= message->len) {
			si_base_free(message->data);
			si_list_iter_remove_current(&SI_LIST_ITERATOR_FOR_VARIABLE(message), SI_TRUE);
		}

		// Check if we already found enough bytes
		if (handled_bytes >= len) {
			break;
		}
	}

	return handled_bytes;
}
#endif

void si_output_buffer_mode2_init(struct SiOutputBuffer* output_buffer, struct SiCircularData* circulair_data_buffer) {
	output_buffer->mode = SI_OUTPUT_BUFFER_MODE2;
	output_buffer->circular_data_buffer = circulair_data_buffer;
}

enum SiResult si_output_buffer_mode2_push_data(const struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len) {
	uint8_t required_message_len = 0;

	required_message_len += MODE2_HEADER_SIZE;	// Start and len byte
	required_message_len += len;  // bytes required for the data

	// Every data byte that is 0xFF should get another 0xFF
	for (uint8_t i = 0; i < len; i++) {
		if (data[i] == 0xFF) {
			required_message_len++;
		}
	}

	// Push actual bytes into circular buffer
	if (si_circular_data_free(output_buffer->circular_data_buffer) >= required_message_len) {
		si_circular_push(output_buffer->circular_data_buffer, 0xFF);  // Start byte
		si_circular_push(output_buffer->circular_data_buffer, (required_message_len - MODE2_HEADER_SIZE));	// Len byte

		for (uint8_t i = 0; i < len; i++) {
			si_circular_push(output_buffer->circular_data_buffer, data[i]);

			// Push another 0xFF when the data is 0xFF
			if (data[i] == 0xFF) {
				si_circular_push(output_buffer->circular_data_buffer, 0xFF);
			}
		}

		return SI_OK;
	}

	return SI_ERROR;
}

uint8_t si_output_buffer_mode2_peek_data(struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len) {
	uint8_t bytes_read = 0;

	for (uint8_t i = 0; i < len; i++) {
		if (si_circular_peek(output_buffer->circular_data_buffer, i, data + i) == SI_ERROR) {
			break;
		}
		bytes_read++;
	}

	return bytes_read;
}

enum SiResult si_output_buffer_mode2_forward_data(struct SiOutputBuffer* output_buffer, uint8_t len) {
	return si_circular_forward(output_buffer->circular_data_buffer, len);
}

void si_output_buffer_mode3_init(struct SiOutputBuffer* output_buffer, struct SiCircularData* circulair_data_buffer) {
	output_buffer->mode = SI_OUTPUT_BUFFER_MODE3;
	output_buffer->circular_data_buffer = circulair_data_buffer;
}

enum SiResult si_output_buffer_mode3_push_data(const struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len) {
	uint8_t required_message_len = 0;

	required_message_len += MODE3_OVERHEAD_SIZE;  // Start 0x00 and end 0xFF
	required_message_len += len;  // bytes required for the data

	// Push actual bytes into circular buffer
	if (si_circular_data_free(output_buffer->circular_data_buffer) >= required_message_len) {
		si_circular_push(output_buffer->circular_data_buffer, 0x00);  // Start byte
		for (uint8_t i = 0; i < len; i++) {
			si_circular_push(output_buffer->circular_data_buffer, data[i]);
		}
		si_circular_push(output_buffer->circular_data_buffer, 0xFF);  // End byte

		return SI_OK;
	}

	return SI_ERROR;
}

uint8_t si_output_buffer_mode3_peek_data(struct SiOutputBuffer* output_buffer, uint8_t* data, uint8_t len) {
	uint8_t bytes_read = 0;

	for (uint8_t i = 0; i < len; i++) {
		if (si_circular_peek(output_buffer->circular_data_buffer, i, data + i) == SI_ERROR) {
			break;
		}
		bytes_read++;
	}

	return bytes_read;
}

enum SiResult si_output_buffer_mode3_forward_data(struct SiOutputBuffer* output_buffer, uint8_t len) {
	return si_circular_forward(output_buffer->circular_data_buffer, len);
}

void si_output_buffer_flush(struct SiOutputBuffer* output_buffer) {

	switch (output_buffer->mode) {
#ifndef SI_EMBEDDED
	case SI_OUTPUT_BUFFER_MODE1:
		si_list_clear(output_buffer->message_queue, SI_TRUE);
		break;
#endif

	case SI_OUTPUT_BUFFER_MODE2:
	case SI_OUTPUT_BUFFER_MODE3:
		si_circular_forward(output_buffer->circular_data_buffer, si_circular_data_available(output_buffer->circular_data_buffer));
		break;

	default:
		break;
	}
}

#ifndef SI_EMBEDDED
struct SiOutputBuffer* si_output_buffer_mode5_create(si_output_buffer_callback_type callback, void* tag) {
	struct SiOutputBuffer* output_buffer = SI_BASE_MALLOC(struct SiOutputBuffer);

	output_buffer->callback = callback;
	output_buffer->tag = tag;

	output_buffer->mode = SI_OUTPUT_BUFFER_MODE5;
	output_buffer->message_queue = si_list_create();

	return output_buffer;
}

void si_output_buffer_mode5_push_data(struct SiOutputBuffer* output_buffer, uint8_t* data, size_t len) {
	struct SiOutputBufferMessage* message = SI_BASE_MALLOC(struct SiOutputBufferMessage);

	const uint32_t start_seq = MODE5_START_BYTE_SEQ;

	// Count number of 0xFFFFFFFF in data
	int nr_of_start_seq = 0;
	for (size_t i = 0; i < len - MODE5_START_BYTE_SIZE + 1; i++) {
		if (memcmp(data + i, &start_seq, sizeof(uint32_t)) == 0) {
			// Found start sequence in data
			nr_of_start_seq++;

			// skip the sequence, -1 since we will add +1 in for loop above
			i += (MODE5_START_BYTE_SIZE - 1);
		}
	}

	uint32_t payload_len = (uint32_t)(len + (nr_of_start_seq * MODE5_START_BYTE_SIZE));
	message->data = (uint8_t*)si_base_malloc(MODE5_HEADER_SIZE + payload_len);

	memcpy(message->data, &start_seq, MODE5_START_BYTE_SIZE);

	uint32_t len_value = SI_NETWORK_UINT32_TO_NETWORK(payload_len);
	memcpy(message->data + MODE5_START_BYTE_SIZE, &len_value, MODE5_LEN_SIZE);

	// Every data sequence that is 0xFFFFFFFF should get another 0xFFFFFFFF
	uint8_t* data_pointer = message->data + MODE5_HEADER_SIZE;
	for (uint8_t i = 0; i < len; i++) {
		if ((len - i) >= MODE5_START_BYTE_SIZE) {
			if (memcmp(data + i, &start_seq, sizeof(uint32_t)) == 0) {
				// Found start sequence in data

				// Add original data
				memcpy(data_pointer, data + i, sizeof(uint32_t));
				data_pointer += MODE5_START_BYTE_SIZE;

				// Add another 0xFFFFFFFF
				memcpy(data_pointer, &start_seq, sizeof(uint32_t));
				data_pointer += MODE5_START_BYTE_SIZE;

				// skip the sequence, -1 since we will add + 1 in for loop above
				i += (MODE5_START_BYTE_SIZE - 1);
			}
			else {
				// Not found, just add byte
				*data_pointer = data[i];
				data_pointer++;
			}
		}
		else {
			// No room left for start sequence, just add byte
			*data_pointer = data[i];
			data_pointer++;
		}
	}

	message->len = MODE5_HEADER_SIZE + payload_len;
	message->pos = 0;

	si_list_append(output_buffer->message_queue, message);
}

size_t si_output_buffer_mode5_handle_data(struct SiOutputBuffer* output_buffer, size_t len) {
	size_t handled_bytes = 0;

	// Iterate through our message queue to find and sent the requested number of bytes
	SI_LIST_FOREACH(struct SiOutputBufferMessage*, message, output_buffer->message_queue) {
		size_t bytes_in_message = (message->len - message->pos);
		size_t remaining_bytes_to_sent = (len - handled_bytes);
		size_t bytes_to_sent = bytes_in_message < remaining_bytes_to_sent ? bytes_in_message : remaining_bytes_to_sent;

		size_t bytes_sent = output_buffer->callback(message->data + message->pos, bytes_to_sent, output_buffer->tag);

		if (bytes_sent > 0) {
			message->pos += bytes_sent;
			handled_bytes += bytes_sent;

			// Clean up message when we have successfully sent it
			if (message->pos >= message->len) {
				si_base_free(message->data);
				si_list_iter_remove_current(&SI_LIST_ITERATOR_FOR_VARIABLE(message), SI_TRUE);
			}

			// Check if we already found enough bytes
			if (handled_bytes >= len) {
				break;
			}
		}
		else {
			// Something went wrong sending the bytes
			break;
		}
	}

	return handled_bytes;
}

struct SiOutputBuffer* si_output_buffer_mode6_create(si_output_buffer_callback_type callback, void* tag) {
	struct SiOutputBuffer* output_buffer = SI_BASE_MALLOC(struct SiOutputBuffer);

	output_buffer->callback = callback;
	output_buffer->tag = tag;

	output_buffer->mode = SI_OUTPUT_BUFFER_MODE6;
	output_buffer->message_queue = si_list_create();

	return output_buffer;
}

size_t si_output_buffer_mode6_handle_data(struct SiOutputBuffer* output_buffer, size_t len) {
	size_t handled_bytes = 0;

	// Iterate through our message queue to find and sent the requested number of bytes
	SI_LIST_FOREACH(struct SiOutputBufferMessage*, message, output_buffer->message_queue) {
		size_t bytes_in_message = (message->len - message->pos);
		size_t remaining_bytes_to_sent = (len - handled_bytes);
		size_t bytes_to_sent = bytes_in_message < remaining_bytes_to_sent ? bytes_in_message : remaining_bytes_to_sent;

		size_t bytes_sent = output_buffer->callback(message->data + message->pos, bytes_to_sent, output_buffer->tag);

		if (bytes_sent > 0) {
			message->pos += bytes_sent;
			handled_bytes += bytes_sent;

			// Clean up message when we have successfully sent it
			if (message->pos >= message->len) {
				si_base_free(message->data);
				si_list_iter_remove_current(&SI_LIST_ITERATOR_FOR_VARIABLE(message), SI_TRUE);
			}

			// Check if we already found enough bytes
			if (handled_bytes >= len) {
				break;
			}
		}
		else {
			// Something went wrong sending the bytes
			break;
		}
	}

	return handled_bytes;
}

void si_output_buffer_mode6_push_data(struct SiOutputBuffer* output_buffer, const char* text) {
	struct SiOutputBufferMessage* message = SI_BASE_MALLOC(struct SiOutputBufferMessage);

	const char* text_message = si_string_format("%s\n", text);
	message->data = (uint8_t*)text_message;
	message->len = strlen(text_message);
	message->pos = 0;

	si_list_append(output_buffer->message_queue, message);
}

#endif

void si_output_buffer_close(struct SiOutputBuffer* output_buffer) {
	// TODO: Implement
}
