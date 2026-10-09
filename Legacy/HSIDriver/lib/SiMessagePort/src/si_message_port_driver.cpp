#include "si_message_port_driver.h"

#include "Arduino.h"

void si_message_port_driver_init() {
	Serial.begin(115200);
}

void si_message_port_driver_sync(struct SiCircularData* input_buffer, struct SiCircularData* output_buffer) {
	uint8_t byte;
	if (si_circular_poll(output_buffer, 0, &byte) == SI_OK) {
		Serial.write(byte);
	}

	while ( (Serial.available() > 0) && (si_circular_data_free(input_buffer) > 0) ) {
		si_circular_push(input_buffer, Serial.read());
	}
}
