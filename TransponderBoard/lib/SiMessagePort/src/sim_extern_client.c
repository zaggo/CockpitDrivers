#include "sim_extern_client.h"

static void input_buffer_callback(const struct SiInputBuffer* input_buffer, const uint8_t len, const void* tag) {
	union SimExternalMessageUnion message;
	struct SimExternClient* client = (struct SimExternClient*) tag;

	if (sim_extern_shared_create_message(input_buffer, len, (struct SimExternMessageBase*) &message) == SI_OK) {
		client->callback((struct SimExternMessageBase*) &message, client->tag);
	}
}

void sim_extern_client_init(struct SimExternClient* client, struct SiInputBuffer* input_buffer, struct SiOutputBuffer* output_buffer, void (*callback)(const struct SimExternMessageBase* message, void* tag), void* tag) {
	client->input_buffer = input_buffer;
	client->output_buffer = output_buffer;

	client->callback = callback;
	client->tag = tag;

	si_input_buffer_mode2_set_callback(client->input_buffer, input_buffer_callback, client);
}

enum SiResult sim_extern_client_push_message(struct SimExternClient* client, struct SimExternMessageBase* message) {
	return sim_extern_shared_push_message(message, client->output_buffer);
}

void sim_extern_client_evaluate_data(const struct SimExternClient* client) {
	si_input_buffer_mode2_evaluate(client->input_buffer);
}
