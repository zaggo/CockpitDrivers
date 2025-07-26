#pragma once

#include "si_input_buffer.h"
#include "si_output_buffer.h"

#include "sim_extern_shared.h"

struct SimExternClient {
	void(*callback)(const struct SimExternMessageBase* message, void* tag);
	void* tag;

	struct SiInputBuffer* input_buffer;
	struct SiOutputBuffer* output_buffer;
};

void sim_extern_client_init(struct SimExternClient* client, struct SiInputBuffer* input_buffer, struct SiOutputBuffer* output_buffer, void (*callback)(const struct SimExternMessageBase* message, void* tag), void* tag);

enum SiResult sim_extern_client_push_message(struct SimExternClient* client, struct SimExternMessageBase* message);
void sim_extern_client_evaluate_data(const struct SimExternClient* client);
