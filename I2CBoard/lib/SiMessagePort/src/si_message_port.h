#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum SiMessagePortChannel {
	SI_MESSAGE_PORT_CHANNEL_A = 0,
	SI_MESSAGE_PORT_CHANNEL_B = 1,
	SI_MESSAGE_PORT_CHANNEL_C = 2,
	SI_MESSAGE_PORT_CHANNEL_D = 3,
	SI_MESSAGE_PORT_CHANNEL_E = 4,
	SI_MESSAGE_PORT_CHANNEL_F = 5,
	SI_MESSAGE_PORT_CHANNEL_G = 6,
	SI_MESSAGE_PORT_CHANNEL_H = 7,
	SI_MESSAGE_PORT_CHANNEL_I = 8,
	SI_MESSAGE_PORT_CHANNEL_J = 9,
	SI_MESSAGE_PORT_CHANNEL_K = 10,
	SI_MESSAGE_PORT_CHANNEL_L = 11,
	SI_MESSAGE_PORT_CHANNEL_M = 12,
	SI_MESSAGE_PORT_CHANNEL_N = 13,
	SI_MESSAGE_PORT_CHANNEL_O = 14,
	SI_MESSAGE_PORT_CHANNEL_P = 15
};

enum SiMessagePortDevice {
	SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_UNO,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_LEONARDO,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_MICRO,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO_EVERY,
	SI_MESSAGE_PORT_DEVICE_ARDUINO_DUE,
    SI_MESSAGE_PORT_DEVICE_ESP32,
    SI_MESSAGE_PORT_DEVICE_NODE_MCU,
	SI_MESSAGE_PORT_DEVICE_ESP8266,
	SI_MESSAGE_PORT_DEVICE_TEENSY_2_0,
	SI_MESSAGE_PORT_DEVICE_TEENSY_PP_2_0,
	SI_MESSAGE_PORT_DEVICE_TEENSY_LC,
	SI_MESSAGE_PORT_DEVICE_TEENSY_3_2,
	SI_MESSAGE_PORT_DEVICE_TEENSY_3_5,
	SI_MESSAGE_PORT_DEVICE_TEENSY_3_6,
	SI_MESSAGE_PORT_DEVICE_TEENSY_4_0,
	SI_MESSAGE_PORT_DEVICE_TEENSY_4_1,
	SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_32,
	SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_64,
	SI_MESSAGE_PORT_DEVICE_STM_NUCLEO_144,

	SI_MESSAGE_PORT_DEVICE_HW_PORT = 99
};

enum SiMessagePortResult {
	SI_MESSAGE_PORT_RESULT_OK,				// Everything went fine
	SI_MESSAGE_PORT_RESULT_BUFFER_OVERFLOW, // The buffer is full at the moment, try sending your data again later.
	SI_MESSAGE_PORT_RESULT_ILLEGAL_LEN,		// You have supplied an illegal length, in most cases this means that you are willing to send to much data.
	SI_MESSAGE_PORT_RESULT_ILLEGAL_DATA,    // The data you have supplied is NULL when you the len > 0
	SI_MESSAGE_PORT_RESULT_ILLEGAL_DEVICE   // You did not supply a valid device	
};

enum SiMessagePortDataType {
	SI_MESSAGE_PORT_DATA_TYPE_BYTE = 0,
	SI_MESSAGE_PORT_DATA_TYPE_STRING = 1,
	SI_MESSAGE_PORT_DATA_TYPE_INTEGER = 2,
	SI_MESSAGE_PORT_DATA_TYPE_FLOAT = 3
};

struct SiMessagePortPayload {
	union {
		uint8_t* data_byte;
		char* data_string;
		int32_t* data_int;
		float* data_float;
	};

	enum SiMessagePortDataType type;
	uint8_t len;
};

enum SiMessagePortLogLevel {
	SI_MESSAGE_PORT_LOG_LEVEL_TRACE,
	SI_MESSAGE_PORT_LOG_LEVEL_DEBUG,
	SI_MESSAGE_PORT_LOG_LEVEL_INFO,
	SI_MESSAGE_PORT_LOG_LEVEL_WARN,
	SI_MESSAGE_PORT_LOG_LEVEL_ERROR
};

// Call once on boot
enum SiMessagePortResult si_message_port_init(enum SiMessagePortDevice device, enum SiMessagePortChannel channel, void (*message_callback)(uint16_t message_id, struct SiMessagePortPayload* payload));

// Call on every tick
void si_message_port_tick();

// Send a new message to Air Manager or Air Player
enum SiMessagePortResult si_message_port_send(uint16_t message_id);

enum SiMessagePortResult si_message_port_send_string(uint16_t message_id, const char* string);

enum SiMessagePortResult si_message_port_send_byte(uint16_t message_id, uint8_t byte);
enum SiMessagePortResult si_message_port_send_bytes(uint16_t message_id, uint8_t* data, uint8_t len);

enum SiMessagePortResult si_message_port_send_integer(uint16_t message_id, int32_t number);
enum SiMessagePortResult si_message_port_send_integers(uint16_t message_id, int32_t* numbers, uint8_t len);

enum SiMessagePortResult si_message_port_send_float(uint16_t message_id, float number);
enum SiMessagePortResult si_message_port_send_floats(uint16_t message_id, float* numbers, uint8_t len);

// Send print message (for debugging)
enum SiMessagePortResult si_message_port_print(enum SiMessagePortLogLevel level, const char* message);

#ifdef __cplusplus
}
#endif
