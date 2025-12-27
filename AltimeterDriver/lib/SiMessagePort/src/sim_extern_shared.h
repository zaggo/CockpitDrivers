#pragma once

#include "si_base.h"
#include "si_input_buffer.h"
#include "si_output_buffer.h"

#define SIM_EXTERN_API_VERSION        7 // Current API version

#define SIM_EXTERN_DATA_PACKET_LEN                                                    4 // Number of bytes in a sim extern data packet
#define SIM_EXTERN_CHR_DISPLAY_MAX_DISPLAYS                                           4 // Maximum number of units in a chr display chain
#define SIM_EXTERN_CHR_DISPLAY_MAX_LINES                                              4 // Maximum number of lines for one display
#define SIM_EXTERN_CHR_DISPLAY_MAX_TEXT_LEN                                          32 // Number of bytes in a character display text message
#define SIM_EXTERN_CHR_DISPLAY_MAX_CHR_LEN                                            8 // Number of characters in a character display (raw)
#define SIM_EXTERN_CHR_DISPLAY_NR_SEGEMENTS                                           8 // Number of segments in character
#define SIM_EXTERN_BUTTON_ARRAY_MAX_COLUMNS                                           8 // Maximum number of colums (note that columns + rows should not exeed max group pins!)
#define SIM_EXTERN_BUTTON_ARRAY_MAX_ROWS                                              8 // Maximum number of rows  (note that columns + rows should not exeed max group pins!)
#define SIM_EXTERN_BUTTON_ARRAY_MAX_BUTTONS                                          ( SIM_EXTERN_BUTTON_ARRAY_MAX_ROWS * SIM_EXTERN_BUTTON_ARRAY_MAX_COLUMNS ) // Maximum number of virtual buttons
#define SIM_EXTERN_MESSAGE_PORT_PACKET_LEN                                           64 // Number of bytes in a sim extern message packet
#define SIM_EXTERN_MESSAGE_PORT_INT_PACKET_LEN   (SIM_EXTERN_MESSAGE_PORT_PACKET_LEN/sizeof(int32_t)) // Number of integers in a sim extern message packet
#define SIM_EXTERN_MESSAGE_PORT_FLOAT_PACKET_LEN (SIM_EXTERN_MESSAGE_PORT_PACKET_LEN/sizeof(float)) // Number of floats in a sim extern message packet

#define SIM_EXTERN_GROUP_MAX_PINS                                                    16 // Maximum number of pins in a group message

// This size calculation might not be accurate all of the time, but it works for now I guess...
#define SIM_EXTERN_MAX_MESSAGE_SIZE (uint8_t)sizeof(union SimExternalMessageUnion)

#ifdef __cplusplus
extern "C"{
#endif

enum SimExternMessageType {
	SIM_EXTERN_MESSAGE_TYPE_UNKNOWN = 0,
	SIM_EXTERN_MESSAGE_TYPE_REQUEST_DEVICE_INFORMATION = 1,
	SIM_EXTERN_MESSAGE_TYPE_DEVICE_INFORMATION = 2,
	SIM_EXTERN_MESSAGE_TYPE_PUSH_PIN_CONFIG = 3,
	SIM_EXTERN_MESSAGE_TYPE_PUSH_GROUP_CONFIG = 4,
	SIM_EXTERN_MESSAGE_TYPE_REQUEST_PIN_CAPABILITY = 5,
	SIM_EXTERN_MESSAGE_TYPE_PIN_CAPABILITY = 6,
	SIM_EXTERN_MESSAGE_TYPE_INPUT_UP = 7,
	SIM_EXTERN_MESSAGE_TYPE_INPUT_DOWN = 8,
	SIM_EXTERN_MESSAGE_TYPE_OUTPUT_UP = 9,
	SIM_EXTERN_MESSAGE_TYPE_OUTPUT_DOWN = 10,
	SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CW = 11,
	SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CCW = 12,
	SIM_EXTERN_MESSAGE_TYPE_BUTTON_PRESSED = 13,
	SIM_EXTERN_MESSAGE_TYPE_BUTTON_RELEASED = 14,
	SIM_EXTERN_MESSAGE_TYPE_LED_SET = 15,
	SIM_EXTERN_MESSAGE_TYPE_PWM_SET_DUTY_CYCLE = 16,
	SIM_EXTERN_MESSAGE_TYPE_ADC_VALUE_CHANGED = 17,
	SIM_EXTERN_MESSAGE_TYPE_PUSH_HOST_TO_DEVICE_DATA = 18,
	SIM_EXTERN_MESSAGE_TYPE_PUSH_DEVICE_TO_HOST_DATA = 19,
	SIM_EXTERN_MESSAGE_TYPE_PING = 20,
	SIM_EXTERN_MESSAGE_TYPE_PONG = 21,
	SIM_EXTERN_MESSAGE_TYPE_DISABLE_ALL = 22,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_BYTES = 23,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_BYTES = 24,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_INTEGERS = 25,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_INTEGERS = 26,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_FLOATS = 27,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_FLOATS = 28,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_STRING = 29,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_STRING = 30,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_EMPTY = 31,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_EMPTY = 32,
	SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_PRINT = 33,
	SIM_EXTERN_MESSAGE_TYPE_DAC_VALUE_SET = 34,
    SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_PUSH_TEXT = 35,
    SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_SET_BRIGHTNESS = 36,
	SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_PRESSED = 37,
	SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_RELEASED = 38,
	SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_CALIBRATE = 39,
	SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_POSITION = 40,
	SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_SPEED = 41,
	SIM_EXTERN_MESSAGE_TYPE_SET_CHANNEL = 42,
	SIM_EXTERN_MESSAGE_TYPE_MAX = 43
};

enum SimExternDeviceType {
	SIM_EXTERN_DEVICE_TYPE_UNKNOWN = 0,
	SIM_EXTERN_DEVICE_TYPE_HW_PORT = 1,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_2560 = 2,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_UNO = 3,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_LEONARDO = 4,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_MICRO = 5,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_NANO = 6,
	SIM_EXTERN_DEVICE_TYPE_KNOBSTER_V2 = 7,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_NANO_EVERY = 8,
	SIM_EXTERN_DEVICE_TYPE_ESP32 = 9,
	SIM_EXTERN_DEVICE_TYPE_NODE_MCU = 10,
	SIM_EXTERN_DEVICE_TYPE_ESP8266 = 11,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_2_0 = 12,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_PP_2_0 = 13,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_LC = 14,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_3_2 = 15,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_3_5 = 16,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_3_6 = 17,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_4_0 = 18,
	SIM_EXTERN_DEVICE_TYPE_TEENSY_4_1 = 19,
	SIM_EXTERN_DEVICE_TYPE_ARDUINO_DUE = 20,
	SIM_EXTERN_DEVICE_TYPE_RPI_PICO = 21,
	SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_32 = 22,
	SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_64 = 23,
	SIM_EXTERN_DEVICE_TYPE_STM_NUCLEO_144 = 24,
	SIM_EXTERN_DEVICE_TYPE_MAX = 25
};

enum SimExternDeviceMode {
	SIM_EXTERN_DEVICE_MODE_UNKNOWN = 0,
	SIM_EXTERN_DEVICE_MODE_PIN = 1,
	SIM_EXTERN_DEVICE_MODE_MESSAGE_PORT = 2,
	SIM_EXTERN_DEVICE_MODE_KNOBSTER = 3,
	SIM_EXTERN_DEVICE_MODE_MAX = 4
};

enum SimExternPinPullType {
	SIM_EXTERN_PIN_PULL_DISABLED = 0,
	SIM_EXTERN_PIN_PULL_UP = 1,
	SIM_EXTERN_PIN_PULL_DOWN = 2,
	SIM_EXTERN_PIN_PULL_MAX = 3
};

enum SimExternPinType {
	SIM_EXTERN_PIN_TYPE_DISABLED = 0,
	SIM_EXTERN_PIN_TYPE_INPUT = 1,
	SIM_EXTERN_PIN_TYPE_OUTPUT = 2,
	SIM_EXTERN_PIN_TYPE_LED = 3,
	SIM_EXTERN_PIN_TYPE_ROTARY = 4,
	SIM_EXTERN_PIN_TYPE_BUTTON = 5,
	SIM_EXTERN_PIN_TYPE_ADC = 6,
	SIM_EXTERN_PIN_TYPE_PWM = 7,
	SIM_EXTERN_PIN_TYPE_TX = 8,
	SIM_EXTERN_PIN_TYPE_RX = 9,
	SIM_EXTERN_PIN_TYPE_DAC = 10,
	SIM_EXTERN_PIN_TYPE_CHR_DISPLAY = 11,
	SIM_EXTERN_PIN_TYPE_BUTTON_ARRAY = 12,
	SIM_EXTERN_PIN_TYPE_STEPPER_MOTOR = 13,
	SIM_EXTERN_PIN_TYPE_MAX = 14
};

enum SimExternGroupType {
	SIM_EXTERN_GROUP_TYPE_DISABLED = 0,
	SIM_EXTERN_GROUP_TYPE_ROTARY_ENCODER = 1,
	SIM_EXTERN_GROUP_TYPE_SWITCH = 2,
    SIM_EXTERN_GROUP_TYPE_CHR_DISPLAY = 3,
	SIM_EXTERN_GROUP_TYPE_BUTTON_ARRAY = 4,
	SIM_EXTERN_GROUP_TYPE_STEPPER_MOTOR = 5,
	SIM_EXTERN_GROUP_TYPE_MAX = 6
};

enum SimExternPinState {
	SIM_EXTERN_PIN_STATE_DOWN = 0,
	SIM_EXTERN_PIN_STATE_UP = 1,
    SIM_EXTERN_PIN_STATE_UNKNOWN = 2,
	SIM_EXTERN_PIN_STATE_MAX = 3
};

enum SimExternDirection {
	SIM_EXTERN_DIRECTION_NONE = 0,
	SIM_EXTERN_DIRECTION_CW = 1,
	SIM_EXTERN_DIRECTION_CCW = -1
};

enum SimExternMessagePortPrintLogLevel {
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_UNKNOWN = 0,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_TRACE = 1,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_DEBUG = 2,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_INFO = 3,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_WARN = 4,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_ERROR = 5,
	SIM_EXTERN_MESSAGE_PORT_LOG_LEVEL_MAX = 6
};

enum SimExternStepperMotorDirection {
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_UNKNOWN = 0,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_FASTEST = 1,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_SLOWEST = 2,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_CW      = 3,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_CCW     = 4,
    SIM_EXTERN_STEPPER_MOTOR_DIRECTION_ENDLESS_CW = 5,
    SIM_EXTERN_STEPPER_MOTOR_DIRECTION_ENDLESS_CCW = 6,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_STOP = 7,
	SIM_EXTERN_STEPPER_MOTOR_DIRECTION_MAX = 8
};

struct SimExternMessageBase {
	enum SimExternMessageType type;
};

struct SimExternMessagePinBase {
	struct SimExternMessageBase base;
	uint8_t pin_id;
};

struct SimExternMessageGroupBase {
	struct SimExternMessageBase base;
	uint8_t group_id;
};

struct SimExternMessageDataBase {
	struct SimExternMessageBase base;
	uint8_t len;
	uint8_t buffer[SIM_EXTERN_DATA_PACKET_LEN];
};

struct SimExternPortMessageBase {
	struct SimExternMessageBase base;
	uint16_t message_id;
};

struct SimExternPortMessageBytesBase {
	struct SimExternPortMessageBase base;
	uint8_t len;
	uint8_t buffer[SIM_EXTERN_MESSAGE_PORT_PACKET_LEN];
};

struct SimExternPortMessageIntegersBase {
	struct SimExternPortMessageBase base;
	uint8_t len;
	int32_t buffer[SIM_EXTERN_MESSAGE_PORT_INT_PACKET_LEN];
};

struct SimExternPortMessageFloatsBase {
	struct SimExternPortMessageBase base;
	uint8_t len;
	float buffer[SIM_EXTERN_MESSAGE_PORT_FLOAT_PACKET_LEN];
};

struct SimExternPortMessageStringBase {
	struct SimExternPortMessageBase base;
	uint8_t len;
	char buffer[SIM_EXTERN_MESSAGE_PORT_PACKET_LEN+1];
};

/*
	Device information
*/
struct SimExternRequestDeviceInformationMessage {
	struct SimExternMessageBase base;
};

struct SimExternDeviceInformationMessage {
	struct SimExternMessageBase base;

	uint8_t api_version;

	enum SimExternDeviceType type;
	enum SimExternDeviceMode mode;

	uint8_t channel;

	uint8_t nr_pins;
	uint8_t nr_digital_pins;
	uint8_t nr_analog_pins;
	uint8_t nr_groups;
};

/*
	Pin config
*/
struct SimExternPinOutput {
	enum SimExternPinState state;
};

struct SimExternPinInput {
	enum SimExternPinPullType pull_state;
	enum SimExternPinState state;
};

struct SimExternPinLED {
	uint8_t brightness; // 0 - 100%
};

struct SimExternPinRotary {
	enum SimExternPinPullType pull_state;
    uint8_t debounce_ms;
};

struct SimExternPinButton {
	enum SimExternPinPullType pull_state;
    uint8_t debounce_ms;
};

struct SimExternPinPWM {
	uint32_t frequency;
	uint16_t duty_cycle;
};

struct SimExternPinDAC {
	uint16_t value;
};

struct SimExternPinADC {
    uint16_t hysteresis; // 0 - FFFF
};

struct SimExternPinButtonArray {
	enum SimExternPinPullType pull_state;
	uint8_t is_row;
};

union SimExternalPinConfig {
	struct SimExternPinOutput output;
	struct SimExternPinInput input;
	struct SimExternPinLED led;
	struct SimExternPinRotary rotary;
	struct SimExternPinButton button;
	struct SimExternPinPWM pwm;
	struct SimExternPinDAC dac;
    struct SimExternPinADC adc;
	struct SimExternPinButtonArray button_array;
};

struct SimExternPinConfigMessage {
	struct SimExternMessagePinBase base;

	enum SimExternPinType type;
	union SimExternalPinConfig config;
};

/*
	Pin capability
*/
struct SimExternPinCapabilityRequestMesssage {
	struct SimExternMessagePinBase base;
};

struct SimExternPinCapabilityMesssage {
	struct SimExternMessagePinBase base;

	// If a pin type is supported, that bit is set in the capability_map
	// SIM_EXTERN_PIN_TYPE_INPUT (0x01) => 00000000 00000000 00000000 00000010
	// SIM_EXTERN_PIN_TYPE_INPUT (0x02) => 00000000 00000000 00000000 00000100
	// SIM_EXTERN_PIN_TYPE_LED   (0x03) => 00000000 00000000 00000000 00001000
	// etc.
	uint32_t capability_map;
};

/*
	Group config
*/
enum SimExternRotaryEncoderType {
    SIM_EXTERN_ROTARY_ENCODER_TYPE_UNKNOWN   = 0,
    SIM_EXTERN_ROTARY_ENCODER_TYPE_1_DETENT  = 1,
    SIM_EXTERN_ROTARY_ENCODER_TYPE_2_DETENT  = 2,
	SIM_EXTERN_ROTARY_ENCODER_TYPE_4_DETENT  = 3,
    SIM_EXTERN_ROTARY_ENCODER_TYPE_MAX       = 4
};

enum SimExternChrDisplayType {
    SIM_EXTERN_CHR_DISPLAY_TYPE_UNKNOWN  = 0,
    SIM_EXTERN_CHR_DISPLAY_TYPE_MAX7219  = 1,
	SIM_EXTERN_CHR_DISPLAY_TYPE_8SEGMENT = 2,
	SIM_EXTERN_CHR_DISPLAY_TYPE_HD44780  = 3,
	SIM_EXTERN_CHR_DISPLAY_TYPE_TM1637   = 4,
	SIM_EXTERN_CHR_DISPLAY_TYPE_TM1638   = 5,
    SIM_EXTERN_CHR_DISPLAY_TYPE_MAX      = 6
};

enum SimExternChrDisplay8SegmentMode {
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_UNKNOWN = 0,
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_DIRECT_COMMON_ANODE = 1,
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_DIRECT_COMMON_CATHODE = 2,
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_TRANSISTOR_COMMON_ANODE = 3,
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_TRANSISTOR_COMMON_CATHODE = 4,
    SIM_EXTERN_CHR_DISPLAY_8SEGMENT_MODE_MAX = 5
};

enum SimExternChrDisplayTM163xMode {
	SIM_EXTERN_CHR_DISPLAY_TM163X_MODE_UNKNOWN = 0,
	SIM_EXTERN_CHR_DISPLAY_TM163X_MODE_COMMON_ANODE = 1,
	SIM_EXTERN_CHR_DISPLAY_TM163X_MODE_COMMON_CATHODE = 2,
	SIM_EXTERN_CHR_DISPLAY_TM163X_MODE_MAX = 3
};

struct SimExternChrDisplayMax7219Config {
	uint8_t nr_displays;
	uint8_t brightness[SIM_EXTERN_CHR_DISPLAY_MAX_DISPLAYS];
};

struct SimExternChrDisplay8SegmentConfig {
    enum SimExternChrDisplay8SegmentMode mode;

	uint8_t nr_characters;
	uint8_t brightness;
};

struct SimExternChrDisplayHD44780Config {
	uint8_t nr_lines;
	uint8_t nr_columns;
	uint8_t brightness;
};

struct SimExternChrDisplayTM1637Config {
	uint8_t nr_characters;
	uint8_t brightness;
};

struct SimExternChrDisplayTM1638Config {
	uint8_t nr_characters;
	uint8_t brightness;
};

struct SimExternChrDisplayConfig {
    enum SimExternChrDisplayType type;
	
	union {
		struct SimExternChrDisplayMax7219Config max7219;
		struct SimExternChrDisplay8SegmentConfig _8segment;
		struct SimExternChrDisplayHD44780Config hd44780;
		struct SimExternChrDisplayTM1637Config tm1637;
		struct SimExternChrDisplayTM1638Config tm1638;
	};
};

/*
	Stepper motor
*/

enum SimExternStepperMotorType {
	SIM_EXTERN_STEPPER_MOTOR_TYPE_UNKNOWN = 0,
	SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_4_STEP = 1,
	SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_6_STEP = 2,
    SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_8_STEP = 3,
	SIM_EXTERN_STEPPER_MOTOR_TYPE_VID66_06 = 4,
	SIM_EXTERN_STEPPER_MOTOR_TYPE_MAX = 5
};

enum SimExternStepperMotorDirectMode {
	SIM_EXTERN_STEPPER_MOTOR_DIRECT_MODE_UNKNOWN = 0,
	SIM_EXTERN_STEPPER_MOTOR_DIRECT_MODE_WAVE_DRIVE = 1,
	SIM_EXTERN_STEPPER_MOTOR_DIRECT_MODE_FULL_STEP = 2,
	SIM_EXTERN_STEPPER_MOTOR_DIRECT_MODE_HALF_STEP = 3,
	SIM_EXTERN_STEPPER_MOTOR_DIRECT_MODE_MAX = 4
};

struct SimExternStepperMotorDirectConfig {
	uint8_t nr_wires;
	enum SimExternStepperMotorDirectMode mode;
};

struct SimExternStepperMotorConfig {
	enum SimExternStepperMotorType type;
	uint16_t nr_steps;
	uint16_t waveform_delay_ms;
	
	uint16_t position;
	enum SimExternStepperMotorDirection direction;

	uint8_t circular;

	union {
		struct SimExternStepperMotorDirectConfig direct;
	} config;
};

/*
	Rotary encoder
*/

struct SimExternRotaryGroupConfig {
	enum SimExternRotaryEncoderType type;
};


struct SimExternButtonArrayConfig {
	uint8_t nr_rows;
	uint8_t nr_columns;
};

struct SimExternGroupConfigMessage {
	struct SimExternMessageGroupBase base;

	enum SimExternGroupType type;

	uint8_t pin_ids[SIM_EXTERN_GROUP_MAX_PINS];
	uint8_t pin_count;

    union SimExternalGroupConfig {
        struct SimExternRotaryGroupConfig rotary_encoder;
        struct SimExternChrDisplayConfig chr_display;
        struct SimExternButtonArrayConfig button_array;
		struct SimExternStepperMotorConfig stepper_motor;
    } config;
};

/*
	Output messages
*/
struct SimExternOutputUpMessage {
	struct SimExternMessagePinBase base;
};
struct SimExternOutputDownMessage {
	struct SimExternMessagePinBase base;
};


/*
	LED
*/
struct SimExternLEDSetMessage {
	struct SimExternMessagePinBase base;

	uint8_t brightness; // 0 - 100%
};


/*
	PWM
*/
struct SimExternPWMSetDutyCycleMessage {
	struct SimExternMessagePinBase base;

	uint16_t duty_cycle;
};


/*
	ADC
*/
struct SimExternADCValueChangedMessage {
	struct SimExternMessagePinBase base;

	uint16_t value; // Percentage 0 - max uint16_t
};

/*
	DAC
*/
struct SimExternDACValueSetMessage {
	struct SimExternMessagePinBase base;

	uint16_t value; // Percentage 0 - max uint16_t
};

/*
    Character display
*/
struct SimExternChrDisplayPushTextMessage {
    struct SimExternMessageGroupBase base;

    uint8_t display_offset;
    uint8_t line_offset;

    uint8_t text_offset;
    uint8_t text_len;

    uint32_t dot_buffer; // Stores dot's. One bit for each character

    char text[SIM_EXTERN_CHR_DISPLAY_MAX_TEXT_LEN+1];
};

struct SimExternChrDisplaySetBrightnessMessage {
    struct SimExternMessageGroupBase base;
    uint8_t display_offset;

    uint8_t brightness; // 0 -100%
};

/*
	Button array
*/
struct SimExternButtonArrayChangeMessage {
	struct SimExternMessageGroupBase base;

	uint8_t row;
	uint8_t column;	
};

/*
	Stepper motor
*/
struct SimExternStepperMotorCalibrateMessage {
	struct SimExternMessageGroupBase base;

	uint16_t position;
};

struct SimExternStepperMotorPositionMessage {
	struct SimExternMessageGroupBase base;

	uint16_t position;
	enum SimExternStepperMotorDirection direction;
};

struct SimExternStepperMotorSpeedMessage {
	struct SimExternMessageGroupBase base;

	uint16_t waveform_delay;
};


/*
	Data push
*/
struct SimExternDataPushHostToDeviceMessage {
	struct SimExternMessageDataBase base;
};
struct SimExternDataPushDeviceToHostMessage {
	struct SimExternMessageDataBase base;
};

/*
	Ping pong
*/
struct SimExternPingMessage {
	struct SimExternMessageBase base;
};
struct SimExternPongMessage {
	struct SimExternMessageBase base;
};

/*
	Disable all
*/
struct SimExternDisableAllMessage {
	struct SimExternMessageBase base;
};

/*
	Message (used mainly for external lib)
*/
struct SimExternMessagePortHostToDeviceMessage {
	struct SimExternPortMessageBase base;
};
struct SimExternMessagePortDeviceToHostMessage {
	struct SimExternPortMessageBase base;
};
struct SimExternMessagePortHostToDeviceBytesMessage {
	struct SimExternPortMessageBytesBase base;
};
struct SimExternMessagePortDeviceToHostBytesMessage {
	struct SimExternPortMessageBytesBase base;
};
struct SimExternMessagePortHostToDeviceIntegersMessage {
	struct SimExternPortMessageIntegersBase base;
};
struct SimExternMessagePortDeviceToHostIntegersMessage {
	struct SimExternPortMessageIntegersBase base;
};
struct SimExternMessagePortHostToDeviceFloatsMessage {
	struct SimExternPortMessageFloatsBase base;
};
struct SimExternMessagePortDeviceToHostFloatsMessage {
	struct SimExternPortMessageFloatsBase base;
};
struct SimExternMessagePortHostToDeviceStringMessage {
	struct SimExternPortMessageStringBase base;
};
struct SimExternMessagePortDeviceToHostStringMessage {
	struct SimExternPortMessageStringBase base;
};

struct SimExternMessagePortPrintMessage {
	struct SimExternMessageBase base;

	uint8_t len;
	char buffer[SIM_EXTERN_MESSAGE_PORT_PACKET_LEN+1];

	enum SimExternMessagePortPrintLogLevel level;
};

/*
	Set channel
*/
struct SimExternSetChannelMessage {
	struct SimExternMessageBase base;

	uint8_t channel;
};


// Make sure to add all available messages in this union
// The union is used to determine the size of the biggest message object
union SimExternalMessageUnion {
	struct SimExternDeviceInformationMessage device_info;
	struct SimExternPinConfigMessage pin_config;
	struct SimExternPinCapabilityRequestMesssage pin_request_pin_compatibility_message;
	struct SimExternPinCapabilityMesssage pin_compatibility_message;
	struct SimExternGroupConfigMessage group_config;
	struct SimExternOutputUpMessage output_up;
	struct SimExternOutputDownMessage output_down;
	struct SimExternLEDSetMessage led_set;
	struct SimExternPWMSetDutyCycleMessage pwm_set_duty_cycle;
	struct SimExternADCValueChangedMessage adc_value_changed;
	struct SimExternDataPushHostToDeviceMessage data_host_to_device;
	struct SimExternDataPushDeviceToHostMessage data_device_to_host;
	struct SimExternPingMessage ping_message;
	struct SimExternPongMessage pong_message;
	struct SimExternMessagePortHostToDeviceBytesMessage message_port_host_to_device_bytes_message;
	struct SimExternMessagePortDeviceToHostBytesMessage message_port_device_to_host_bytes_message;
	struct SimExternMessagePortHostToDeviceIntegersMessage message_port_host_to_device_integers_message;
	struct SimExternMessagePortDeviceToHostIntegersMessage message_port_device_to_host_integers_message;
	struct SimExternMessagePortHostToDeviceFloatsMessage message_port_host_to_device_floats_message;
	struct SimExternMessagePortDeviceToHostFloatsMessage message_port_device_to_host_floats_message;
	struct SimExternMessagePortHostToDeviceStringMessage message_port_host_to_device_string_message;
	struct SimExternMessagePortDeviceToHostStringMessage message_port_device_to_host_string_message;
	struct SimExternMessagePortPrintMessage message_port_print_message;
	struct SimExternDACValueSetMessage dac_set_message;
    struct SimExternChrDisplayPushTextMessage chr_display_text_push_message;
    struct SimExternChrDisplaySetBrightnessMessage chr_display_set_brightness_message;
	struct SimExternButtonArrayChangeMessage button_array_change_message;
};

extern const char* sim_extern_shared_device_type_names[SIM_EXTERN_DEVICE_TYPE_MAX];
extern const char* sim_extern_shared_device_mode_names[SIM_EXTERN_DEVICE_MODE_MAX];

enum SiResult sim_extern_shared_create_message(const struct SiInputBuffer* input_buffer, const uint8_t len, struct SimExternMessageBase* message);
enum SiResult sim_extern_shared_push_message(const struct SimExternMessageBase* message, const struct SiOutputBuffer* output_buffer);

#ifdef __cplusplus
}
#endif
