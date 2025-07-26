#include "si_message_port.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "si_circular_data.h"
#include "si_input_buffer.h"
#include "si_output_buffer.h"
#include "sim_extern_client.h"

#include "si_message_port_driver.h"

#define HW_LIB_INPUT_BUFFER_SIZE (SI_INPUT_BUFFER_MODE2_HEADER_SIZE + SIM_EXTERN_MAX_MESSAGE_SIZE)
#define HW_LIB_OUTPUT_BUFFER_SIZE (SI_OUTPUT_BUFFER_MODE2_HEADER_SIZE + SIM_EXTERN_MAX_MESSAGE_SIZE)

struct SiHwLib {
	enum SiMessagePortChannel channel;
	
	enum SimExternDeviceType device;

    struct SiCircularData main_input_circular_data;
    uint8_t main_input_buffer[HW_LIB_INPUT_BUFFER_SIZE];
    struct SiCircularData main_output_circular_data;
    uint8_t main_output_buffer[HW_LIB_OUTPUT_BUFFER_SIZE];
    
	struct SiInputBuffer input_buffer;
    struct SiOutputBuffer output_buffer;
	
    struct SimExternClient sim_extern_client;
    
	void (*message_callback)(uint16_t message_id, struct SiMessagePortPayload* payload);
};

static struct SiHwLib lib;

static void sim_extern_client_callback(const struct SimExternMessageBase* message, void* tag) {
	switch(message->type) {
		case SIM_EXTERN_MESSAGE_TYPE_REQUEST_DEVICE_INFORMATION: {
			struct SimExternDeviceInformationMessage message;
			message.base.type = SIM_EXTERN_MESSAGE_TYPE_DEVICE_INFORMATION;
			message.api_version = SIM_EXTERN_API_VERSION;
			message.type = lib.device;
			message.mode = SIM_EXTERN_DEVICE_MODE_MESSAGE_PORT;
			message.nr_pins = 0;
			message.nr_digital_pins = 0;
			message.nr_analog_pins = 0;
			message.nr_groups = 0;
			message.channel = (uint8_t) lib.channel;
			
			sim_extern_client_push_message(&lib.sim_extern_client, &message.base);
			break;
		}
		
		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_EMPTY: {
			struct SimExternMessagePortHostToDeviceMessage* push_message = (struct SimExternMessagePortHostToDeviceMessage*) message;
			if (lib.message_callback != NULL) {
				lib.message_callback(push_message->base.message_id, NULL);
			}
			break;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_BYTES: {
			struct SimExternMessagePortHostToDeviceBytesMessage* push_message = (struct SimExternMessagePortHostToDeviceBytesMessage*) message;
			if (lib.message_callback != NULL) {
				struct SiMessagePortPayload payload;
				payload.data_byte = push_message->base.buffer;
				payload.len = push_message->base.len;
				payload.type = SI_MESSAGE_PORT_DATA_TYPE_BYTE;
				
				lib.message_callback(push_message->base.base.message_id, &payload);
			}
			break;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_STRING: {
			struct SimExternMessagePortHostToDeviceStringMessage* push_message = (struct SimExternMessagePortHostToDeviceStringMessage*) message;
			if (lib.message_callback != NULL) {
				struct SiMessagePortPayload payload;				
				payload.data_string = push_message->base.buffer;
				payload.len = push_message->base.len;
				payload.type = SI_MESSAGE_PORT_DATA_TYPE_STRING;

				lib.message_callback(push_message->base.base.message_id, &payload);
			}
			break;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_INTEGERS: {
			struct SimExternMessagePortHostToDeviceIntegersMessage* push_message = (struct SimExternMessagePortHostToDeviceIntegersMessage*) message;
			if (lib.message_callback != NULL) {
				struct SiMessagePortPayload payload;				
				payload.data_int = push_message->base.buffer;
				payload.len = push_message->base.len;
				payload.type = SI_MESSAGE_PORT_DATA_TYPE_INTEGER;

				lib.message_callback(push_message->base.base.message_id, &payload);
			}
			break;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_FLOATS: {
			struct SimExternMessagePortHostToDeviceFloatsMessage* push_message = (struct SimExternMessagePortHostToDeviceFloatsMessage*) message;
			if (lib.message_callback != NULL) {
				struct SiMessagePortPayload payload;
				payload.data_float = push_message->base.buffer;
				payload.len = push_message->base.len;
				payload.type = SI_MESSAGE_PORT_DATA_TYPE_FLOAT;

				lib.message_callback(push_message->base.base.message_id, &payload);
			}
			break;
		}
		
		default:
			// Cannot handle this message
			break;
	}
}

// Call once on boot
enum SiMessagePortResult si_message_port_init(enum SiMessagePortDevice device, enum SiMessagePortChannel channel, void (*message_callback)(uint16_t message_id, struct SiMessagePortPayload* payload)) {
	lib.channel = channel;
	lib.message_callback = message_callback;

	si_circular_data_init(&lib.main_input_circular_data, lib.main_input_buffer, HW_LIB_INPUT_BUFFER_SIZE);
	si_circular_data_init(&lib.main_output_circular_data, lib.main_output_buffer, HW_LIB_OUTPUT_BUFFER_SIZE);
	
	si_input_buffer_mode2_init(&lib.input_buffer, &lib.main_input_circular_data, NULL, NULL);
	si_output_buffer_mode2_init(&lib.output_buffer, &lib.main_output_circular_data);

	sim_extern_client_init(&lib.sim_extern_client, &lib.input_buffer, &lib.output_buffer, sim_extern_client_callback, NULL);

	si_message_port_driver_init();

	switch(device) {
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_2560;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_UNO:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_UNO;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_NANO;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_LEONARDO:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_LEONARDO;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_MICRO:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_MICRO;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO_EVERY:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_NANO_EVERY;
		break;
	case SI_MESSAGE_PORT_DEVICE_ARDUINO_DUE:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ARDUINO_DUE;
		break;
    case SI_MESSAGE_PORT_DEVICE_ESP32:
        lib.device = SIM_EXTERN_DEVICE_TYPE_ESP32;
        break;
    case SI_MESSAGE_PORT_DEVICE_NODE_MCU:
        lib.device = SIM_EXTERN_DEVICE_TYPE_NODE_MCU;
        break;
	case SI_MESSAGE_PORT_DEVICE_ESP8266:
		lib.device = SIM_EXTERN_DEVICE_TYPE_ESP8266;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_2_0:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_2_0;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_PP_2_0:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_PP_2_0;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_LC:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_LC;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_3_2:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_3_2;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_3_5:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_3_5;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_3_6:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_3_6;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_4_0:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_4_0;
		break;
	case SI_MESSAGE_PORT_DEVICE_TEENSY_4_1:
		lib.device = SIM_EXTERN_DEVICE_TYPE_TEENSY_4_1;
		break;
	case SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_32:
		lib.device = SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_32;
		break;
	case SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_64:
		lib.device = SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_64;
		break;
	case SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_144:
		lib.device = SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_144;
		break;
	case SI_MESSAGE_PORT_DEVICE_HW_PORT:
		lib.device = SIM_EXTERN_DEVICE_TYPE_HW_PORT;
		break;


	default:
		// Illegal device type
		lib.device = SIM_EXTERN_DEVICE_TYPE_UNKNOWN;
		return SI_MESSAGE_PORT_RESULT_ILLEGAL_DEVICE;
	}

	return SI_MESSAGE_PORT_RESULT_OK;
}

// Call on every tick
void si_message_port_tick() {
	si_message_port_driver_sync(&lib.main_input_circular_data, &lib.main_output_circular_data);

	sim_extern_client_evaluate_data(&lib.sim_extern_client);
}

// Push a new message that will be send to Air Manager or Air Player
enum SiMessagePortResult si_message_port_send(uint16_t message_id) {
	struct SimExternMessagePortDeviceToHostMessage message;
	message.base.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_EMPTY;
	message.base.message_id = message_id;
	
	return sim_extern_client_push_message(&lib.sim_extern_client, &message.base.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
};

enum SiMessagePortResult si_message_port_send_byte(uint16_t message_id, uint8_t byte) {
	return si_message_port_send_bytes(message_id, &byte, 1);
};


enum SiMessagePortResult si_message_port_send_bytes(uint16_t message_id, uint8_t* data, uint8_t len) {
	if (len > SIM_EXTERN_MESSAGE_PORT_PACKET_LEN) {
		return SI_MESSAGE_PORT_RESULT_ILLEGAL_LEN;
	}
	
	struct SimExternMessagePortDeviceToHostBytesMessage message;
	message.base.base.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_BYTES;
	message.base.base.message_id = message_id;
	if (len > 0) {
		if (data != NULL) {
			for(uint8_t i = 0; i < len; i++) {
				message.base.buffer[i] = data[i];
			}
		}
		else {
			return SI_MESSAGE_PORT_RESULT_ILLEGAL_DATA;
		}
	}
	message.base.len = len;
	
	return sim_extern_client_push_message(&lib.sim_extern_client,&message.base.base.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
};


enum SiMessagePortResult si_message_port_send_string(uint16_t message_id, const char* string) {
	uint8_t len = string == NULL ? 0 : strlen(string);

	if (len > SIM_EXTERN_MESSAGE_PORT_PACKET_LEN) {
		return SI_MESSAGE_PORT_RESULT_ILLEGAL_LEN;
	}
	
	struct SimExternMessagePortDeviceToHostBytesMessage message;
	message.base.base.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_STRING;
	message.base.base.message_id = message_id;

	if (len > 0) {
		if (string != NULL) {
			for(uint8_t i = 0; i < len; i++) {
				message.base.buffer[i] = string[i];
			}
		}
		else {
			return SI_MESSAGE_PORT_RESULT_ILLEGAL_DATA;
		}
	}

	message.base.len = len;
	
	return sim_extern_client_push_message(&lib.sim_extern_client,&message.base.base.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
};

enum SiMessagePortResult si_message_port_send_integer(uint16_t message_id, int32_t number) {
	return si_message_port_send_integers(message_id, &number, 1);
};

enum SiMessagePortResult si_message_port_send_integers(uint16_t message_id, int32_t* numbers, uint8_t len) {
	if (len > SIM_EXTERN_MESSAGE_PORT_INT_PACKET_LEN) {
		return SI_MESSAGE_PORT_RESULT_ILLEGAL_LEN;
	}
	
	struct SimExternMessagePortDeviceToHostIntegersMessage message;
	message.base.base.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_INTEGERS;
	message.base.base.message_id = message_id;

	if (len > 0) {
		if (numbers != NULL) {
			for(uint8_t i = 0; i < len; i++) {
				message.base.buffer[i] = numbers[i];
			}
		}
		else {
			return SI_MESSAGE_PORT_RESULT_ILLEGAL_DATA;
		}
	}
	message.base.len = len;
	
	return sim_extern_client_push_message(&lib.sim_extern_client,&message.base.base.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
};

enum SiMessagePortResult si_message_port_send_float(uint16_t message_id, float number) {
	return si_message_port_send_floats(message_id, &number, 1);
};

enum SiMessagePortResult si_message_port_send_floats(uint16_t message_id, float* numbers, uint8_t len) {
	if (len > SIM_EXTERN_MESSAGE_PORT_FLOAT_PACKET_LEN) {
		return SI_MESSAGE_PORT_RESULT_ILLEGAL_LEN;
	}
	
	struct SimExternMessagePortDeviceToHostFloatsMessage message;
	message.base.base.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_FLOATS;
	message.base.base.message_id = message_id;

	if (len > 0) {
		if (numbers != NULL) {
			for(uint8_t i = 0; i < len; i++) {
				message.base.buffer[i] = numbers[i];
			}
		}
		else {
			return SI_MESSAGE_PORT_RESULT_ILLEGAL_DATA;
		}
	}
	message.base.len = len;
	
	return sim_extern_client_push_message(&lib.sim_extern_client,&message.base.base.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
};


enum SiMessagePortResult si_message_port_print(enum SiMessagePortLogLevel level, const char* message) {
	struct SimExternMessagePortPrintMessage print_message;
	print_message.base.type = SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_PRINT;
	print_message.len = (uint8_t) strlen(message);

	switch(level) {
		case SI_MESSAGE_PORT_LOG_LEVEL_TRACE: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_TRACE; break;
		case SI_MESSAGE_PORT_LOG_LEVEL_DEBUG: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_DEBUG; break;
		case SI_MESSAGE_PORT_LOG_LEVEL_INFO: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_INFO; break;
		case SI_MESSAGE_PORT_LOG_LEVEL_WARN: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_WARN; break;
		case SI_MESSAGE_PORT_LOG_LEVEL_ERROR: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_ERROR; break;
		default: print_message.level = SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_UNKNOWN; break;
	}

	strncpy(print_message.buffer, message, SIM_EXTERN_MESSAGE_PORT_PACKET_LEN);

	return sim_extern_client_push_message(&lib.sim_extern_client, &print_message.base) == SI_OK ? SI_MESSAGE_PORT_RESULT_OK : SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW;
}
