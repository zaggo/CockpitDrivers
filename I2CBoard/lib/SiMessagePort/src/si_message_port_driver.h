#pragma once

#include "si_base.h"
#include "si_circular_data.h"
#include "sim_extern_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

void si_message_port_driver_init();

void si_message_port_driver_sync(struct SiCircularData* input_buffer, struct SiCircularData* output_buffer);

#ifdef __cplusplus
}
#endif
