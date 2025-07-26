#include "si_message_port.hpp"

SiMessagePort::SiMessagePort(enum SiMessagePortDevice device, enum SiMessagePortChannel channel, void (*message_callback)(uint16_t message_id, struct SiMessagePortPayload* payload)) {
	si_message_port_init(device, channel, message_callback);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id) {
	return si_message_port_send(message_id);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, String string) {
	return si_message_port_send_string(message_id, string.c_str());
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, uint8_t byte) {
	return si_message_port_send_byte(message_id, byte);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, uint8_t* data, uint8_t len) {
	return si_message_port_send_bytes(message_id, data, len);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, int32_t number) {
	return si_message_port_send_integer(message_id, number);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, int32_t* numbers, uint8_t len) {
	return si_message_port_send_integers(message_id, numbers, len);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, float number) {
	return si_message_port_send_float(message_id, number);
}

enum SiMessagePortResult SiMessagePort::SendMessage(uint16_t message_id, float* numbers, uint8_t len) {
	return si_message_port_send_floats(message_id, numbers, len);
}

enum SiMessagePortResult SiMessagePort::DebugMessage(enum SiMessagePortLogLevel level, String message) {
	return si_message_port_print(level, message.c_str());
};

void SiMessagePort::Tick() {
	si_message_port_tick();
}
