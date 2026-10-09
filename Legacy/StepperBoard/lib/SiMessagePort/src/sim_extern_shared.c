#include "sim_extern_shared.h"

#include "si_network.h"

const char* sim_extern_shared_device_type_names[SIM_EXTERN_DEVICE_TYPE_MAX] = {
	"UNKNOWN",
	"HW_PORT",
	"ARDUINO_2560",
	"ARDUINO_UNO",
	"ARDUINO_LEONARDO",
	"ARDUINO_MICRO",
	"ARDUINO_NANO",
	"KNOBSTER_V2",
	"ARDUINO_NANO_EVERY",
	"ESP32",
	"NODE_MCU",
	"ESP8266",
	"TEENSY_2_0",
	"TEENY++_2_0",
	"TEENSY_LC",
	"TEENSY_3_2",
	"TEENSY_3_5",
	"TEENSY_3_6",
	"TEENSY_4_0",
	"TEENSY_4_1",
	"ARDUINO_DUE",
	"RPI_PICO",
	"STM_NUCLEO_32",
	"STM_NUCLEO_64",
	"STM_NUCLEO_144"
};

const char* sim_extern_shared_device_mode_names[SIM_EXTERN_DEVICE_MODE_MAX] = {
	"UNKNOWN",
	"PIN",
	"MESSAGE_PORT",
	"KNOBSTER"
};

enum SiResult sim_extern_shared_create_message(const struct SiInputBuffer* input_buffer, const uint8_t len, struct SimExternMessageBase* message) {
	uint8_t byte;

	if (len == 0) {
		return SI_ERROR;
	}

	struct SiCircularData* circular_data = input_buffer->mode234_circular_data_buffer;

	si_circular_peek(circular_data, 0, &byte);

	// Check for valid message type id
	if (byte >= SIM_EXTERN_MESSAGE_TYPE_MAX) {
		return SI_ERROR;
	}
	
	// Every message has a message id
	message->type = (enum SimExternMessageType) byte;

	switch (message->type) {
		case SIM_EXTERN_MESSAGE_TYPE_REQUEST_DEVICE_INFORMATION:
			return (len == 1) ? SI_OK : SI_ERROR;

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
		case SIM_EXTERN_MESSAGE_TYPE_DEVICE_INFORMATION: {
			struct SimExternDeviceInformationMessage* device_information = (struct SimExternDeviceInformationMessage*) message;

			si_circular_peek(circular_data, 1, &device_information->api_version);
			si_circular_peek(circular_data, 2, &device_information->channel);
			si_circular_peek(circular_data, 3, &byte);
			device_information->type = (enum SimExternDeviceType) byte;

			si_circular_peek(circular_data, 4, &device_information->nr_pins);
			si_circular_peek(circular_data, 5, &device_information->nr_digital_pins);
			si_circular_peek(circular_data, 6, &device_information->nr_analog_pins);
			si_circular_peek(circular_data, 7, &device_information->nr_groups);

			if (len == 9) {
				si_circular_peek(circular_data, 8, &byte);
				device_information->mode = (enum SimExternDeviceMode) byte;
			}
			else {
				device_information->mode = SIM_EXTERN_DEVICE_MODE_PIN;
			}

			return ( (len == 8) || (len == 9) ) ? SI_OK : SI_ERROR;
		}
#endif

		case SIM_EXTERN_MESSAGE_TYPE_PUSH_PIN_CONFIG: {
			// We need at least three bytes to determine message id, pin id and pin type
			if (len >= 3) {
				struct SimExternPinConfigMessage* config_message = (struct SimExternPinConfigMessage*) message;

				si_circular_peek(circular_data, 1, &config_message->base.pin_id);

				si_circular_peek(circular_data, 2, &byte);
				config_message->type = (enum SimExternPinType) byte;

				switch (config_message->type) {
					case SIM_EXTERN_PIN_TYPE_DISABLED:
						return (len == 3) ? SI_OK : SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_INPUT:
						if (len == 4) {
							si_circular_peek(circular_data, 3, &byte);
							config_message->config.input.pull_state = (enum SimExternPinPullType) byte;
							return SI_OK;
						}
						return SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_OUTPUT:
						if (len == 4) {
							si_circular_peek(circular_data, 3, &byte);
							config_message->config.output.state = (enum SimExternPinState) byte;
							return SI_OK;
						}
						return SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_LED:
						if (len == 4) {
							si_circular_peek(circular_data, 3, &config_message->config.led.brightness);
							return SI_OK;
						}
						return SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_ROTARY:
						if (len == 5) {
							si_circular_peek(circular_data, 3, &byte);
							config_message->config.rotary.pull_state = (enum SimExternPinPullType) byte;
                            si_circular_peek(circular_data, 4, &config_message->config.rotary.debounce_ms);
							return SI_OK;
						}
						return SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_BUTTON:
						if (len == 5) {
							si_circular_peek(circular_data, 3, &byte);
							config_message->config.button.pull_state = (enum SimExternPinPullType) byte;
                            si_circular_peek(circular_data, 4, &config_message->config.button.debounce_ms);
							return SI_OK;
						}
						return SI_ERROR;

#ifndef SIM_EXTERN_OMIT_ADC
					case SIM_EXTERN_PIN_TYPE_ADC:
                        if (len == 5) {
                            si_circular_peek(circular_data, 3, &byte);
                            config_message->config.adc.hysteresis = ((uint16_t)byte * 256);
                            si_circular_peek(circular_data, 4, &byte);
                            config_message->config.adc.hysteresis += ((uint16_t)byte);
                            return SI_OK;
                        }
						return SI_ERROR;
#endif						

					case SIM_EXTERN_PIN_TYPE_PWM:
						if (len == 9) {
							config_message->config.pwm.frequency = 0;
							config_message->config.pwm.duty_cycle = 0;

							si_circular_peek(circular_data, 3, &byte);
							config_message->config.pwm.frequency += ((uint32_t)byte * 256 * 256 * 256);
							si_circular_peek(circular_data, 4, &byte);
							config_message->config.pwm.frequency += ((uint32_t)byte * 256 * 256);
							si_circular_peek(circular_data, 5, &byte);
							config_message->config.pwm.frequency += ((uint32_t)byte * 256);
							si_circular_peek(circular_data, 6, &byte);
							config_message->config.pwm.frequency += ((uint32_t)byte);

							si_circular_peek(circular_data, 7, &byte);
							config_message->config.pwm.duty_cycle += ((uint16_t)byte * 256);
							si_circular_peek(circular_data, 8, &byte);
							config_message->config.pwm.duty_cycle += ((uint16_t)byte);
							
							return SI_OK;
						}
						
						return SI_ERROR;

					case SIM_EXTERN_PIN_TYPE_RX:
					case SIM_EXTERN_PIN_TYPE_TX:
					case SIM_EXTERN_PIN_TYPE_CHR_DISPLAY:
					case SIM_EXTERN_PIN_TYPE_STEPPER_MOTOR:
						return (len == 3) ? SI_OK : SI_ERROR;

#ifndef SIM_EXTERN_OMIT_DAC
					case SIM_EXTERN_PIN_TYPE_DAC:
                        if (len == 5) {
							config_message->config.dac.value = 0;

                            si_circular_peek(circular_data, 3, &byte);
                            config_message->config.dac.value += ((uint16_t)byte * 256);
                            si_circular_peek(circular_data, 4, &byte);
                            config_message->config.dac.value += ((uint16_t)byte);

                            return SI_OK;
                        }
                        return SI_ERROR;
#endif

#ifndef SIM_EXTERN_OMIT_BUTTON_ARRAY
					case SIM_EXTERN_PIN_TYPE_BUTTON_ARRAY:
						if (len == 5) {
							si_circular_peek(circular_data, 3, &config_message->config.button_array.is_row);
							si_circular_peek(circular_data, 4, &byte);
							config_message->config.button_array.pull_state = (enum SimExternPinPullType) byte;
							return SI_OK;
						}
						return SI_ERROR;
#endif						

					default:
						return SI_ERROR;
				}
			}
			break;
		}

#ifndef SIM_EXTERN_OMIT_PIN_CAPABILITY
		case SIM_EXTERN_MESSAGE_TYPE_REQUEST_PIN_CAPABILITY: {
			struct SimExternPinCapabilityRequestMesssage* pin_message = (struct SimExternPinCapabilityRequestMesssage*) message;
			si_circular_peek(circular_data, 1, &pin_message->base.pin_id);

			return (len == 2) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_PIN_CAPABILITY: {

			struct SimExternPinCapabilityMesssage* pin_message = (struct SimExternPinCapabilityMesssage*) message;
			si_circular_peek(circular_data, 1, &pin_message->base.pin_id);
					
			si_circular_peek(circular_data, 2, &byte);
			pin_message->capability_map += ((uint32_t)byte * 256 * 256 * 256);
			si_circular_peek(circular_data, 3, &byte);
			pin_message->capability_map += ((uint32_t)byte * 256 * 256);
			si_circular_peek(circular_data, 4, &byte);
			pin_message->capability_map += ((uint32_t)byte * 256);
			si_circular_peek(circular_data, 5, &byte);
			pin_message->capability_map += ((uint32_t)byte);

			return (len == 6) ? SI_OK : SI_ERROR;
		}
#endif
		case SIM_EXTERN_MESSAGE_TYPE_PUSH_GROUP_CONFIG:
			// We need at least three bytes to determine message id, pin id count and pin type
			if (len >= 3) {
				struct SimExternGroupConfigMessage* config_message = (struct SimExternGroupConfigMessage*) message;

				si_circular_peek(circular_data, 1, &config_message->base.group_id);

				si_circular_peek(circular_data, 2, &byte);

				if (byte < SIM_EXTERN_GROUP_TYPE_MAX) {
					config_message->type = (enum SimExternGroupType) byte;
				}
				else {
					return SI_ERROR;
				}
				
				si_circular_peek(circular_data, 3, &config_message->pin_count);

				if (config_message->pin_count > SIM_EXTERN_GROUP_MAX_PINS) {
					return SI_ERROR;
				}

				for(uint8_t i = 0; i < config_message->pin_count; i++) {
					si_circular_peek(circular_data, 4 + i, config_message->pin_ids + i);
				}

				switch (config_message->type) {

					case SIM_EXTERN_GROUP_TYPE_DISABLED:
						return (len == (4 + config_message->pin_count)) ? SI_OK : SI_ERROR;

					case SIM_EXTERN_GROUP_TYPE_ROTARY_ENCODER:
                        if (len == 7) {
                            si_circular_peek(circular_data, 6, &byte);
                            config_message->config.rotary_encoder.type = (enum SimExternRotaryEncoderType) byte;
                            return SI_OK;
                        }
						return (len == 6) ? SI_OK : SI_ERROR;

#ifndef SIM_EXTERN_OMIT_CHR_DISPLAY
                    case SIM_EXTERN_GROUP_TYPE_CHR_DISPLAY:
                        if (len >= (5 + config_message->pin_count) ) {
                            si_circular_peek(circular_data, 4 + config_message->pin_count, &byte);
                            config_message->config.chr_display.type = (enum SimExternChrDisplayType) byte;

							switch (config_message->config.chr_display.type) {
							case SIM_EXTERN_CHR_DISPLAY_TYPE_MAX7219:
								si_circular_peek(circular_data, 5 + config_message->pin_count, &config_message->config.chr_display.max7219.nr_displays);
								if (len >= (6 + config_message->pin_count + config_message->config.chr_display.max7219.nr_displays)) {
									for (uint8_t i = 0; i < config_message->config.chr_display.max7219.nr_displays; i++) {
										si_circular_peek(circular_data, 6 + config_message->pin_count + i, config_message->config.chr_display.max7219.brightness + i);
									}
									return SI_OK;
								}
								break;

							case SIM_EXTERN_CHR_DISPLAY_TYPE_8SEGMENT:
								if (len == (8 + config_message->pin_count) ) {
                                    si_circular_peek(circular_data, 5 + config_message->pin_count, &byte);
                                    config_message->config.chr_display._8segment.mode = (enum SimExternChrDisplay8SegmentMode) byte;

									si_circular_peek(circular_data, 6 + config_message->pin_count, &config_message->config.chr_display._8segment.nr_characters);
									si_circular_peek(circular_data, 7 + config_message->pin_count, &config_message->config.chr_display._8segment.brightness);
									return SI_OK;
								}
								break;

							case SIM_EXTERN_CHR_DISPLAY_TYPE_HD44780:
								if (len == (8 + config_message->pin_count) ) {
									si_circular_peek(circular_data, 5 + config_message->pin_count, &config_message->config.chr_display.hd44780.nr_lines);
									si_circular_peek(circular_data, 6 + config_message->pin_count, &config_message->config.chr_display.hd44780.nr_columns);
									si_circular_peek(circular_data, 7 + config_message->pin_count, &config_message->config.chr_display.hd44780.brightness);
									return SI_OK;
								}
								break;

							case SIM_EXTERN_CHR_DISPLAY_TYPE_TM1637:
								if (len == (7 + config_message->pin_count)) {
									si_circular_peek(circular_data, 5 + config_message->pin_count, &config_message->config.chr_display.tm1637.nr_characters);
									si_circular_peek(circular_data, 6 + config_message->pin_count, &config_message->config.chr_display.tm1637.brightness);
									return SI_OK;
								}
								break;
								
							case SIM_EXTERN_CHR_DISPLAY_TYPE_TM1638:
								if (len == (7 + config_message->pin_count)) {
									si_circular_peek(circular_data, 5 + config_message->pin_count, &config_message->config.chr_display.tm1638.nr_characters);
									si_circular_peek(circular_data, 6 + config_message->pin_count, &config_message->config.chr_display.tm1638.brightness);
									return SI_OK;
								}							
								break;

							default:
								return SI_ERROR;
							}
                        }
                        return SI_ERROR;
#endif

#ifndef SIM_EXTERN_OMIT_BUTTON_ARRAY
					case SIM_EXTERN_GROUP_TYPE_BUTTON_ARRAY:
						if (len == (6 + config_message->pin_count) ) {
							si_circular_peek(circular_data, 4 + config_message->pin_count, &config_message->config.button_array.nr_rows);
							si_circular_peek(circular_data, 5 + config_message->pin_count, &config_message->config.button_array.nr_columns);							
							return SI_OK;
						}
						return SI_ERROR;
#endif						

#ifndef SIM_EXTERN_OMIT_STEPPER_MOTOR
					case SIM_EXTERN_GROUP_TYPE_STEPPER_MOTOR:
						if (len == (13 + config_message->pin_count) ) {
						    si_circular_peek(circular_data, 4 + config_message->pin_count, &byte);
						    config_message->config.stepper_motor.type = (enum SimExternStepperMotorType) byte;

							si_circular_peek(circular_data, 5 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.nr_steps = (byte << 8);
							si_circular_peek(circular_data, 6 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.nr_steps |= (byte << 0);

							si_circular_peek(circular_data, 7 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.waveform_delay_ms = (byte << 8);
							si_circular_peek(circular_data, 8 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.waveform_delay_ms |= (byte << 0);

							si_circular_peek(circular_data, 9 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.position = (byte << 8);
							si_circular_peek(circular_data, 10 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.position |= (byte << 0);

							si_circular_peek(circular_data, 11 + config_message->pin_count, &byte);
							config_message->config.stepper_motor.direction = (enum SimExternStepperMotorDirection) byte;

							si_circular_peek(circular_data, 12 + config_message->pin_count, &config_message->config.stepper_motor.circular);

							return SI_OK;
						}
						return SI_ERROR;
#endif						

					default:
					return SI_ERROR;
				}
			}
			break;

		case SIM_EXTERN_MESSAGE_TYPE_INPUT_UP:
		case SIM_EXTERN_MESSAGE_TYPE_INPUT_DOWN:
		case SIM_EXTERN_MESSAGE_TYPE_OUTPUT_UP:
		case SIM_EXTERN_MESSAGE_TYPE_OUTPUT_DOWN:
		case SIM_EXTERN_MESSAGE_TYPE_BUTTON_PRESSED:
		case SIM_EXTERN_MESSAGE_TYPE_BUTTON_RELEASED: {
			struct SimExternMessagePinBase* pin_message = (struct SimExternMessagePinBase*) message;
			si_circular_peek(circular_data, 1, &pin_message->pin_id);
			return (len == 2) ? SI_OK : SI_ERROR;
		}

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
		case SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CW:
		case SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CCW: {
			struct SimExternMessageGroupBase* group_message = (struct SimExternMessageGroupBase*) message;
			si_circular_peek(circular_data, 1, &group_message->group_id);
			return (len == 2) ? SI_OK : SI_ERROR;
		}
#endif

		case SIM_EXTERN_MESSAGE_TYPE_LED_SET: {
			struct SimExternLEDSetMessage* led_set_message = (struct SimExternLEDSetMessage*) message;
			si_circular_peek(circular_data, 1, &led_set_message->base.pin_id);
			si_circular_peek(circular_data, 2, &led_set_message->brightness);
			return (len == 3) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_PWM_SET_DUTY_CYCLE: {
			struct SimExternPWMSetDutyCycleMessage* pwm_set_duty_cycle_message = (struct SimExternPWMSetDutyCycleMessage*) message;
			si_circular_peek(circular_data, 1, &pwm_set_duty_cycle_message->base.pin_id);

			si_circular_peek(circular_data, 2, &byte);
			pwm_set_duty_cycle_message->duty_cycle = ((uint16_t)byte * 256);
			si_circular_peek(circular_data, 3, &byte);
			pwm_set_duty_cycle_message->duty_cycle += ((uint16_t)byte);

			return (len == 4) ? SI_OK : SI_ERROR;
		}

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
		case SIM_EXTERN_MESSAGE_TYPE_ADC_VALUE_CHANGED: {
			struct SimExternADCValueChangedMessage* adc_message = (struct SimExternADCValueChangedMessage*) message;
			si_circular_peek(circular_data, 1, &adc_message->base.pin_id);
			si_circular_peek(circular_data, 2, &byte);
			adc_message->value = (byte << 8);
			si_circular_peek(circular_data, 3, &byte);
			adc_message->value |= (byte << 0);
			return (len == 4) ? SI_OK : SI_ERROR;
		}
#endif


#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
		case SIM_EXTERN_MESSAGE_TYPE_PUSH_HOST_TO_DEVICE_DATA: 
		case SIM_EXTERN_MESSAGE_TYPE_PUSH_DEVICE_TO_HOST_DATA: {
			struct SimExternMessageDataBase* data_message = (struct SimExternMessageDataBase*) message;
			si_circular_peek(circular_data, 1, &data_message->len);

			for(uint8_t i = 0; (i < data_message->len) && (i < SIM_EXTERN_DATA_PACKET_LEN); i++) {
				si_circular_peek(circular_data, 2 + i, data_message->buffer + i);
			}
			
			return (len == (2 + data_message->len) ) ? SI_OK : SI_ERROR;
		}
#endif

#ifndef SIM_EXTERN_OMIT_MESSAGE_PORT
        case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_EMPTY:
        case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_EMPTY: {
            struct SimExternPortMessageBase* message_message = (struct SimExternPortMessageBase*) message;

            si_circular_peek(circular_data, 1, &byte);
            message_message->message_id = ((uint16_t)byte * 256);
            si_circular_peek(circular_data, 2, &byte);
            message_message->message_id += ((uint16_t)byte);

            return (len == 3) ? SI_OK : SI_ERROR;
        }

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_BYTES:
		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_BYTES: {
			struct SimExternPortMessageBytesBase* message_message = (struct SimExternPortMessageBytesBase*) message;

			si_circular_peek(circular_data, 1, &byte);
			message_message->base.message_id = ((uint16_t)byte * 256);
			si_circular_peek(circular_data, 2, &byte);
			message_message->base.message_id += ((uint16_t)byte);

			si_circular_peek(circular_data, 3, &message_message->len);

			for(uint8_t i = 0; (i < message_message->len) && (i < SIM_EXTERN_MESSAGE_PORT_PACKET_LEN); i++) {
				si_circular_peek(circular_data, 4 + i, message_message->buffer + i);
			}
			
			return (len == (4 + message_message->len) ) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_STRING:
		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_STRING: {
			struct SimExternPortMessageStringBase* message_message = (struct SimExternPortMessageStringBase*) message;

			si_circular_peek(circular_data, 1, &byte);
			message_message->base.message_id = ((uint16_t)byte * 256);
			si_circular_peek(circular_data, 2, &byte);
			message_message->base.message_id += ((uint16_t)byte);

			si_circular_peek(circular_data, 3, &message_message->len);

            uint8_t msg_len = (uint8_t) SI_MIN(SIM_EXTERN_MESSAGE_PORT_PACKET_LEN, message_message->len);
			for(uint8_t i = 0; i < msg_len; i++) {
				si_circular_peek(circular_data, 4 + i, (uint8_t*) (message_message->buffer + i) );
			}
            message_message->buffer[msg_len] = '\0';
			
			return (len == (4 + message_message->len) ) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_INTEGERS:
		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_INTEGERS: {
			struct SimExternPortMessageIntegersBase* message_message = (struct SimExternPortMessageIntegersBase*) message;

			si_circular_peek(circular_data, 1, &byte);
			message_message->base.message_id = ((uint16_t)byte * 256);
			si_circular_peek(circular_data, 2, &byte);
			message_message->base.message_id += ((uint16_t)byte);

			si_circular_peek(circular_data, 3, &message_message->len);

			for(uint8_t i = 0; (i < message_message->len) && (i < SIM_EXTERN_MESSAGE_PORT_INT_PACKET_LEN); i++) {
				int32_t value = 0;
				si_circular_peek(circular_data, 4 + (i * 4) + 0, &byte);
				value = byte;
				si_circular_peek(circular_data, 4 + (i * 4) + 1, &byte);
				value |= ((int32_t)byte << 8);
				si_circular_peek(circular_data, 4 + (i * 4) + 2, &byte);
				value |= ((int32_t)byte << 16);
				si_circular_peek(circular_data, 4 + (i * 4) + 3, &byte);
				value |= ((int32_t)byte << 24);

				message_message->buffer[i] = SI_NETWORK_INT32_TO_NETWORK(value);
			}
			
			return (len == (4 + (message_message->len * 4) ) ) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_FLOATS:
		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_FLOATS: {
			struct SimExternPortMessageFloatsBase* message_message = (struct SimExternPortMessageFloatsBase*) message;

			si_circular_peek(circular_data, 1, &byte);
			message_message->base.message_id = ((uint16_t)byte * 256);
			si_circular_peek(circular_data, 2, &byte);
			message_message->base.message_id += ((uint16_t)byte);

			si_circular_peek(circular_data, 3, &message_message->len);

			for(uint8_t i = 0; (i < message_message->len) && (i < SIM_EXTERN_MESSAGE_PORT_INT_PACKET_LEN); i++) {
                uint8_t value[4];
                si_circular_peek(circular_data, 4 + (i * 4) + 0, &byte);
                value[0] = byte;
                si_circular_peek(circular_data, 4 + (i * 4) + 1, &byte);
                value[1] = byte;
                si_circular_peek(circular_data, 4 + (i * 4) + 2, &byte);
                value[2] = byte;
                si_circular_peek(circular_data, 4 + (i * 4) + 3, &byte);
                value[3] = byte;

                message_message->buffer[i] = si_network_float(value);
			}
			
			return (len == (4 + (message_message->len * 4) ) ) ? SI_OK : SI_ERROR;
		}

		case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_PRINT: {
			struct SimExternMessagePortPrintMessage* print_message = (struct SimExternMessagePortPrintMessage*) message;

			si_circular_peek(circular_data, 1, &byte);
			print_message->level = (enum SimExternMessagePortPrintLogLevel) byte;

			si_circular_peek(circular_data, 2, &print_message->len);

            uint8_t msg_len = (uint8_t)SI_MIN(SIM_EXTERN_MESSAGE_PORT_PACKET_LEN, print_message->len);
			for(uint8_t i = 0; i < msg_len; i++) {
				si_circular_peek(circular_data, 3 + i, (uint8_t*) print_message->buffer + i);
			}
			print_message->buffer[msg_len] = '\0';
			
			return (len == (3 + print_message->len) ) ? SI_OK : SI_ERROR;
		}
#endif

		case SIM_EXTERN_MESSAGE_TYPE_PING:
		case SIM_EXTERN_MESSAGE_TYPE_PONG:
		case SIM_EXTERN_MESSAGE_TYPE_DISABLE_ALL:
			return (len == 1) ? SI_OK : SI_ERROR;

#ifndef SIM_EXTERN_OMIT_DAC
		case SIM_EXTERN_MESSAGE_TYPE_DAC_VALUE_SET: {
			struct SimExternDACValueSetMessage* dac_message = (struct SimExternDACValueSetMessage*) message;
			si_circular_peek(circular_data, 1, &dac_message->base.pin_id);
			si_circular_peek(circular_data, 2, &byte);
			dac_message->value = (byte << 8);
			si_circular_peek(circular_data, 3, &byte);
			dac_message->value |= (byte << 0);
			return (len == 4) ? SI_OK : SI_ERROR;
		}
#endif

#ifndef SIM_EXTERN_OMIT_CHR_DISPLAY
        case SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_PUSH_TEXT: {
            struct SimExternChrDisplayPushTextMessage* text_message = (struct SimExternChrDisplayPushTextMessage*) message;
            si_circular_peek(circular_data, 1, &text_message->base.group_id);

            si_circular_peek(circular_data, 2, &text_message->display_offset);
            si_circular_peek(circular_data, 3, &text_message->line_offset);
            si_circular_peek(circular_data, 4, &text_message->text_offset);
            si_circular_peek(circular_data, 5, &text_message->text_len);

            si_circular_peek(circular_data, 6, &byte);
            text_message->dot_buffer = ((uint32_t)byte << 24);
            si_circular_peek(circular_data, 7, &byte);
            text_message->dot_buffer |= ((uint32_t)byte << 16);
            si_circular_peek(circular_data, 8, &byte);
            text_message->dot_buffer |= ((uint32_t)byte << 8);
            si_circular_peek(circular_data, 9, &byte);
            text_message->dot_buffer |= ((uint32_t)byte << 0);

            for (uint8_t i = 0; i < text_message->text_len; i++) {
                si_circular_peek(circular_data, 10 + i, (uint8_t*)text_message->text + i);
            }
            text_message->text[text_message->text_len] = '\0';

            return (len == (10 + text_message->text_len) );
        }

        case SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_SET_BRIGHTNESS: {
            struct SimExternChrDisplaySetBrightnessMessage* brightness_message = (struct SimExternChrDisplaySetBrightnessMessage*) message;

            si_circular_peek(circular_data, 1, &brightness_message->base.group_id);
            si_circular_peek(circular_data, 2, &brightness_message->display_offset);
            si_circular_peek(circular_data, 3, &brightness_message->brightness);

            return (len == 4);
        }
#endif

#ifndef SIM_EXTERN_OMIT_STEPPER_MOTOR
		case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_CALIBRATE: {
			struct SimExternStepperMotorCalibrateMessage* calibrate_message = (struct SimExternStepperMotorCalibrateMessage*) message;

			si_circular_peek(circular_data, 1, &calibrate_message->base.group_id);
			si_circular_peek(circular_data, 2, &byte);
			calibrate_message->position = (byte << 8);
			si_circular_peek(circular_data, 3, &byte);
			calibrate_message->position |= (byte << 0);

			return (len == 4);
		}

		case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_SPEED: {
			struct SimExternStepperMotorSpeedMessage* speed_message = (struct SimExternStepperMotorSpeedMessage*) message;

			si_circular_peek(circular_data, 1, &speed_message->base.group_id);
			si_circular_peek(circular_data, 2, &byte);
			speed_message->waveform_delay = (byte << 8);
			si_circular_peek(circular_data, 3, &byte);
			speed_message->waveform_delay |= (byte << 0);

			return (len == 4);
		}

		case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_POSITION: {
			struct SimExternStepperMotorPositionMessage* position_message = (struct SimExternStepperMotorPositionMessage*) message;

			si_circular_peek(circular_data, 1, &position_message->base.group_id);
			si_circular_peek(circular_data, 2, &byte);
			position_message->position = (byte << 8);
			si_circular_peek(circular_data, 3, &byte);
			position_message->position |= (byte << 0);

			si_circular_peek(circular_data, 4, &byte);
			position_message->direction = (enum SimExternStepperMotorDirection) byte;

			return (len == 5);
		}
#endif

#if !defined(SIM_EXTERN_OMIT_HOST_TO_CLIENT) & !defined(SIM_EXTERN_OMIT_BUTTON_ARRAY)
		case SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_PRESSED:
		case SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_RELEASED: {
			struct SimExternButtonArrayChangeMessage* change_message = (struct SimExternButtonArrayChangeMessage*) message;

			si_circular_peek(circular_data, 1, &change_message->base.group_id);
			si_circular_peek(circular_data, 2, &change_message->row);
			si_circular_peek(circular_data, 3, &change_message->column);			

			return (len == 4);
		}
#endif

		case SIM_EXTERN_MESSAGE_TYPE_SET_CHANNEL: {
			struct SimExternSetChannelMessage* change_message = (struct SimExternSetChannelMessage*) message;
			
			si_circular_peek(circular_data, 1, &change_message->channel);

			return (len == 2);
		}
		default:
			break;
	}

	return SI_ERROR;
}

enum SiResult sim_extern_shared_push_message(const struct SimExternMessageBase* message, const struct SiOutputBuffer* output_buffer) {
	uint8_t buffer[SIM_EXTERN_MAX_MESSAGE_SIZE];
	uint8_t len = 0;

	// First byte is always the message type
	buffer[0] = (uint8_t) message->type;

	switch (message->type) {
#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_REQUEST_DEVICE_INFORMATION: {
		len = 1;
		break;
	}
#endif

	case SIM_EXTERN_MESSAGE_TYPE_DEVICE_INFORMATION: {
		struct SimExternDeviceInformationMessage* device_information = (struct SimExternDeviceInformationMessage*) message;

		buffer[1] = device_information->api_version;
		buffer[2] = device_information->channel;
		buffer[3] = (uint8_t) device_information->type;
		buffer[4] = device_information->nr_pins;
		buffer[5] = device_information->nr_digital_pins;
		buffer[6] = device_information->nr_analog_pins;
		buffer[7] = device_information->nr_groups;
		buffer[8] = (uint8_t) device_information->mode;
		len = 9;
		break;
	}

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_PUSH_PIN_CONFIG: {

		// Every pin config message has it's pin id and pin type
		struct SimExternPinConfigMessage* config_message = (struct SimExternPinConfigMessage*) message;
		buffer[1] = config_message->base.pin_id;
		buffer[2] = (uint8_t)config_message->type;

		switch (config_message->type) {
		case SIM_EXTERN_PIN_TYPE_DISABLED:
			len = 3;
			break;

		case SIM_EXTERN_PIN_TYPE_INPUT:
			buffer[3] = (uint8_t)config_message->config.input.pull_state;
			len = 4;
			break;

		case SIM_EXTERN_PIN_TYPE_OUTPUT:
			buffer[3] = (uint8_t) config_message->config.output.state;
			len = 4;
			break;

		case SIM_EXTERN_PIN_TYPE_LED:
			buffer[3] = config_message->config.led.brightness;
			len = 4;
			break;

		case SIM_EXTERN_PIN_TYPE_ROTARY:
			buffer[3] = (uint8_t)config_message->config.rotary.pull_state;
            buffer[4] = (uint8_t)config_message->config.rotary.debounce_ms;
			len = 5;
			break;

		case SIM_EXTERN_PIN_TYPE_BUTTON:
			buffer[3] = (uint8_t)config_message->config.button.pull_state;
            buffer[4] = config_message->config.button.debounce_ms;
			len = 5;
			break;

#ifndef SIM_EXTERN_OMIT_DAC
		case SIM_EXTERN_PIN_TYPE_ADC:
            buffer[4] = (uint8_t)config_message->config.adc.hysteresis % 256;
            config_message->config.adc.hysteresis /= 256;
            buffer[3] = (uint8_t)config_message->config.adc.hysteresis % 256;

			len = 5;
			break;
#endif

		case SIM_EXTERN_PIN_TYPE_PWM:
			buffer[6] = (uint8_t) (config_message->config.pwm.frequency % 256);
			config_message->config.pwm.frequency /= 256;
			buffer[5] = (uint8_t) (config_message->config.pwm.frequency % 256);
			config_message->config.pwm.frequency /= 256;
			buffer[4] = (uint8_t) (config_message->config.pwm.frequency % 256);
			config_message->config.pwm.frequency /= 256;
			buffer[3] = (uint8_t) (config_message->config.pwm.frequency % 256);

			buffer[8] = (uint8_t) config_message->config.pwm.duty_cycle % 256;
			config_message->config.pwm.duty_cycle /= 256;
			buffer[7] = (uint8_t) config_message->config.pwm.duty_cycle % 256;

			len = 9;
			break;

#ifndef SIM_EXTERN_OMIT_DAC
        case SIM_EXTERN_PIN_TYPE_DAC:
            buffer[4] = (uint8_t)config_message->config.dac.value % 256;
            config_message->config.dac.value /= 256;
            buffer[3] = (uint8_t)config_message->config.dac.value % 256;

            len = 5;
            break;
#endif

		case SIM_EXTERN_PIN_TYPE_TX:
		case SIM_EXTERN_PIN_TYPE_RX:
		case SIM_EXTERN_PIN_TYPE_CHR_DISPLAY:
		case SIM_EXTERN_PIN_TYPE_STEPPER_MOTOR:
			len = 3;
			break;

#ifndef SIM_EXTERN_OMIT_BUTTON_ARRAY
		case SIM_EXTERN_PIN_TYPE_BUTTON_ARRAY:
		    buffer[3] = config_message->config.button_array.is_row;
			buffer[4] = (uint8_t) config_message->config.button_array.pull_state;
		    len = 5;
		    break;
#endif

		default:
			return SI_ERROR;
		}

		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_PIN_CAPABILITY
	case SIM_EXTERN_MESSAGE_TYPE_REQUEST_PIN_CAPABILITY: {
		struct SimExternPinCapabilityRequestMesssage* pin_message = (struct SimExternPinCapabilityRequestMesssage*) message;
		buffer[1] = pin_message->base.pin_id;

		len = 2;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_PIN_CAPABILITY: {
		struct SimExternPinCapabilityMesssage* pin_message = (struct SimExternPinCapabilityMesssage*) message;

		buffer[1] = pin_message->base.pin_id;

		buffer[5] = (uint8_t) (pin_message->capability_map % 256);
		pin_message->capability_map /= 256;
		buffer[4] = (uint8_t) (pin_message->capability_map % 256);
		pin_message->capability_map /= 256;
		buffer[3] = (uint8_t) (pin_message->capability_map % 256);
		pin_message->capability_map /= 256;
		buffer[2] = (uint8_t) (pin_message->capability_map % 256);

		len = 6;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_PUSH_GROUP_CONFIG: {

		// Every pin config message has it's pin id and pin type
		struct SimExternGroupConfigMessage* config_message = (struct SimExternGroupConfigMessage*) message;
		buffer[1] = config_message->base.group_id;
		buffer[2] = (uint8_t)config_message->type;

		buffer[3] = config_message->pin_count;

		for (uint8_t i = 0; i < config_message->pin_count; i++) {
			buffer[4 + i] = config_message->pin_ids[i];
		}

		switch (config_message->type) {
        case SIM_EXTERN_GROUP_TYPE_DISABLED:
            len = 4 + config_message->pin_count;
            break;

		case SIM_EXTERN_GROUP_TYPE_ROTARY_ENCODER:
            buffer[4 + config_message->pin_count] = (uint8_t) config_message->config.rotary_encoder.type;
			len = 5 + config_message->pin_count;
			break;

#ifndef SIM_EXTERN_OMIT_CHR_DISPLAY
        case SIM_EXTERN_GROUP_TYPE_CHR_DISPLAY:
            buffer[4 + config_message->pin_count] = config_message->config.chr_display.type;
            
			switch (config_message->config.chr_display.type) {
				case SIM_EXTERN_CHR_DISPLAY_TYPE_MAX7219:
					buffer[5 + config_message->pin_count] = config_message->config.chr_display.max7219.nr_displays;
					for (uint8_t i = 0; i < config_message->config.chr_display.max7219.nr_displays; i++) {
						buffer[6 + config_message->pin_count + i] = config_message->config.chr_display.max7219.brightness[i];
					}
					len = 6 + config_message->pin_count + config_message->config.chr_display.max7219.nr_displays;
					break;

				case SIM_EXTERN_CHR_DISPLAY_TYPE_8SEGMENT:
                    buffer[5 + config_message->pin_count] = (uint8_t) config_message->config.chr_display._8segment.mode;
					buffer[6 + config_message->pin_count] = config_message->config.chr_display._8segment.nr_characters;
					buffer[7 + config_message->pin_count] = config_message->config.chr_display._8segment.brightness;
					len = 8 + config_message->pin_count;
					break;

				case SIM_EXTERN_CHR_DISPLAY_TYPE_HD44780:
					buffer[5 + config_message->pin_count] = config_message->config.chr_display.hd44780.nr_lines;
					buffer[6 + config_message->pin_count] = config_message->config.chr_display.hd44780.nr_columns;
					buffer[7 + config_message->pin_count] = config_message->config.chr_display.hd44780.brightness;
					len = 8 + config_message->pin_count;
					break;

				case SIM_EXTERN_CHR_DISPLAY_TYPE_TM1637:
					buffer[5 + config_message->pin_count] = config_message->config.chr_display.tm1637.nr_characters;
					buffer[6 + config_message->pin_count] = config_message->config.chr_display.tm1637.brightness;
					len = 7 + config_message->pin_count;
					break;
					
				case SIM_EXTERN_CHR_DISPLAY_TYPE_TM1638:
					buffer[5 + config_message->pin_count] = config_message->config.chr_display.tm1638.nr_characters;
					buffer[6 + config_message->pin_count] = config_message->config.chr_display.tm1638.brightness;
					len = 7 + config_message->pin_count;				
					break;

				default:
					break;
			}
            break;
#endif			

#ifndef SIM_EXTERN_OMIT_STEPPER_MOTOR
		case SIM_EXTERN_GROUP_TYPE_STEPPER_MOTOR:
			buffer[4 + config_message->pin_count] = config_message->config.stepper_motor.type;

			switch(config_message->config.stepper_motor.type) {
				case SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_4_STEP:
				case SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_6_STEP:
                case SIM_EXTERN_STEPPER_MOTOR_TYPE_4_WIRE_8_STEP:
                case SIM_EXTERN_STEPPER_MOTOR_TYPE_VID66_06:
					buffer[5 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.nr_steps >> 8) & 0xFF);
					buffer[6 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.nr_steps >> 0) & 0xFF);

					buffer[7 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.waveform_delay_ms >> 8) & 0xFF);
					buffer[8 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.waveform_delay_ms >> 0) & 0xFF);

					buffer[9 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.position >> 8) & 0xFF);
					buffer[10 + config_message->pin_count] = (uint8_t)((config_message->config.stepper_motor.position >> 0) & 0xFF);

					buffer[11 + config_message->pin_count] = (uint8_t)config_message->config.stepper_motor.direction;

					buffer[12 + config_message->pin_count] = (uint8_t)config_message->config.stepper_motor.circular;

					len = 13 + config_message->pin_count;
					break;
#endif

				default:
					break;
			}

			break;

#ifndef SIM_EXTERN_OMIT_BUTTON_ARRAY
		case SIM_EXTERN_GROUP_TYPE_BUTTON_ARRAY:
		    buffer[4 + config_message->pin_count] = config_message->config.button_array.nr_rows;
			buffer[5 + config_message->pin_count] = config_message->config.button_array.nr_columns;		    
			len = 6 + config_message->pin_count;
			break;
#endif			

		default:
			return SI_ERROR;
		}

		break;
	}
#endif

	case SIM_EXTERN_MESSAGE_TYPE_INPUT_UP:
	case SIM_EXTERN_MESSAGE_TYPE_INPUT_DOWN:
	case SIM_EXTERN_MESSAGE_TYPE_OUTPUT_UP:
	case SIM_EXTERN_MESSAGE_TYPE_OUTPUT_DOWN:
	case SIM_EXTERN_MESSAGE_TYPE_BUTTON_PRESSED:
	case SIM_EXTERN_MESSAGE_TYPE_BUTTON_RELEASED: {
		struct SimExternMessagePinBase* pin_message = (struct SimExternMessagePinBase*) message;
		buffer[1] = pin_message->pin_id;

		len = 2;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CW:
	case SIM_EXTERN_MESSAGE_TYPE_ROTARY_ENCODER_CCW: {
		struct SimExternMessageGroupBase* group_message = (struct SimExternMessageGroupBase*) message;
		buffer[1] =group_message->group_id;

		len = 2;
		break;
	}

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_LED_SET: {
		struct SimExternLEDSetMessage* led_message = (struct SimExternLEDSetMessage*) message;
		buffer[1] = led_message->base.pin_id;
		buffer[2] = led_message->brightness;

		len = 3;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_PWM_SET_DUTY_CYCLE: {
		struct SimExternPWMSetDutyCycleMessage* pwm_set_duty_cycle_message = (struct SimExternPWMSetDutyCycleMessage*) message;
		buffer[1] = pwm_set_duty_cycle_message->base.pin_id;

		buffer[3] = pwm_set_duty_cycle_message->duty_cycle % 256;
		pwm_set_duty_cycle_message->duty_cycle /= 256;
		buffer[2] = pwm_set_duty_cycle_message->duty_cycle % 256;

		len = 4;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_DAC
	case SIM_EXTERN_MESSAGE_TYPE_ADC_VALUE_CHANGED: {
		struct SimExternADCValueChangedMessage* adc_message = (struct SimExternADCValueChangedMessage*) message;
		buffer[1] = adc_message->base.pin_id;

		buffer[2] = (uint8_t)((adc_message->value >> 8) & 0xFF);
		buffer[3] = (uint8_t)((adc_message->value >> 0) & 0xFF);

		len = 4;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_HOST_TO_CLIENT
	case SIM_EXTERN_MESSAGE_TYPE_PUSH_HOST_TO_DEVICE_DATA: 
	case SIM_EXTERN_MESSAGE_TYPE_PUSH_DEVICE_TO_HOST_DATA: {
		struct SimExternMessageDataBase* data_message = (struct SimExternMessageDataBase*) message;

		buffer[1] = data_message->len;
		for(uint8_t i = 0; i < data_message->len; i++) {
			buffer[2+i] = data_message->buffer[i];
		}

		len = 2 + data_message->len;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_MESSAGE_PORT
    case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_EMPTY:
    case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_EMPTY: {
        struct SimExternPortMessageBase* message_message = (struct SimExternPortMessageBase*) message;

        buffer[1] = (uint8_t)((message_message->message_id >> 8) & 0xFF);
        buffer[2] = (uint8_t)((message_message->message_id >> 0) & 0xFF);

        len = 3;
        break;
    }

	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_BYTES:
	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_BYTES: {
		struct SimExternPortMessageBytesBase* message_message = (struct SimExternPortMessageBytesBase*) message;

		buffer[1] = (uint8_t)((message_message->base.message_id >> 8) & 0xFF);
		buffer[2] = (uint8_t)((message_message->base.message_id >> 0) & 0xFF);

		buffer[3] = message_message->len;
		for(uint8_t i = 0; i < message_message->len; i++) {
			buffer[4+i] = message_message->buffer[i];
		}

		len = 4 + message_message->len;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_STRING:
	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_STRING: {
		struct SimExternPortMessageStringBase* message_message = (struct SimExternPortMessageStringBase*) message;

		buffer[1] = (uint8_t)((message_message->base.message_id >> 8) & 0xFF);
		buffer[2] = (uint8_t)((message_message->base.message_id >> 0) & 0xFF);

		buffer[3] = message_message->len;
		for(uint8_t i = 0; i < message_message->len; i++) {
			buffer[4+i] = message_message->buffer[i];
		}

		len = 4 + message_message->len;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_INTEGERS:
	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_INTEGERS: {
		struct SimExternPortMessageIntegersBase* message_message = (struct SimExternPortMessageIntegersBase*) message;

		buffer[1] = (uint8_t)((message_message->base.message_id >> 8) & 0xFF);
		buffer[2] = (uint8_t)((message_message->base.message_id >> 0) & 0xFF);

		buffer[3] = message_message->len;
		for(uint8_t i = 0; i < message_message->len; i++) {
			int32_t* int32_buffer = (int32_t*) (buffer + (4 + (i*4)));
			*int32_buffer = SI_NETWORK_INT32_TO_NETWORK(message_message->buffer[i]);
		}

		len = 4 + (message_message->len * 4);
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_HOST_TO_DEVICE_FLOATS:
	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_DEVICE_TO_HOST_FLOATS: {
		struct SimExternPortMessageFloatsBase* message_message = (struct SimExternPortMessageFloatsBase*) message;

		buffer[1] = (uint8_t)((message_message->base.message_id >> 8) & 0xFF);
		buffer[2] = (uint8_t)((message_message->base.message_id >> 0) & 0xFF);

		buffer[3] = message_message->len;
		for(uint8_t i = 0; i < message_message->len; i++) {
			float* float_buffer = (float*) (buffer + (4 + (i*4)));
            *float_buffer = si_network_swap_float(message_message->buffer[i]);
		}

		len = 4 + (message_message->len * 4);
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_MESSAGE_PORT_PRINT: {
		struct SimExternMessagePortPrintMessage* print_message = (struct SimExternMessagePortPrintMessage*) message;

		buffer[1] = (uint8_t)print_message->level;
		buffer[2] = print_message->len;
		for(uint8_t i = 0; i < print_message->len; i++) {
			buffer[3+i] = (uint8_t) print_message->buffer[i];
		}

		len = 3 + print_message->len;
		break;
	}
#endif

#if !defined(SIM_EXTERN_OMIT_HOST_TO_CLIENT) & !defined(SIM_EXTERN_OMIT_CHR_DISPLAY)
    case SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_PUSH_TEXT: {
        struct SimExternChrDisplayPushTextMessage* text_message = (struct SimExternChrDisplayPushTextMessage*) message;
        buffer[1] = text_message->base.group_id;
        buffer[2] = text_message->display_offset;
        buffer[3] = text_message->line_offset;
        buffer[4] = text_message->text_offset;
        buffer[5] = text_message->text_len;
        buffer[6] = (uint8_t)((text_message->dot_buffer >> 24) & 0xFF);
        buffer[7] = (uint8_t)((text_message->dot_buffer >> 16) & 0xFF);
        buffer[8] = (uint8_t)((text_message->dot_buffer >> 8 ) & 0xFF);
        buffer[9] = (uint8_t)((text_message->dot_buffer >> 0 ) & 0xFF);

        for (uint8_t i = 0; i < text_message->text_len; i++) {
            buffer[10 + i] = (uint8_t)text_message->text[i];
        }
        len = 10 + text_message->text_len;
        break;
    }
#endif

#if !defined(SIM_EXTERN_OMIT_HOST_TO_CLIENT) & !defined(SIM_EXTERN_OMIT_CHR_DISPLAY)
    case SIM_EXTERN_MESSAGE_TYPE_CHR_DISPLAY_SET_BRIGHTNESS: {
        struct SimExternChrDisplaySetBrightnessMessage* brightness_message = (struct SimExternChrDisplaySetBrightnessMessage*) message;
        buffer[1] = brightness_message->base.group_id;
        buffer[2] = brightness_message->display_offset;
        buffer[3] = brightness_message->brightness;
        len = 4;
        break;
    }
#endif

#if !defined(SIM_EXTERN_OMIT_HOST_TO_CLIENT) & !defined(SIM_EXTERN_OMIT_STEPPER_MOTOR)
	case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_CALIBRATE: {
		struct SimExternStepperMotorCalibrateMessage* calibrate_message = (struct SimExternStepperMotorCalibrateMessage*) message;
		buffer[1] = calibrate_message->base.group_id;
		buffer[2] = (uint8_t)((calibrate_message->position >> 8) & 0xFF);
		buffer[3] = (uint8_t)((calibrate_message->position >> 0) & 0xFF);
		len = 4;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_SPEED: {
		struct SimExternStepperMotorSpeedMessage* speed_message = (struct SimExternStepperMotorSpeedMessage*) message;
		buffer[1] = speed_message->base.group_id;
		buffer[2] = (uint8_t)((speed_message->waveform_delay >> 8) & 0xFF);
		buffer[3] = (uint8_t)((speed_message->waveform_delay >> 0) & 0xFF);
		len = 4;
		break;
	}

	case SIM_EXTERN_MESSAGE_TYPE_STEPPER_MOTOR_POSITION: {
		struct SimExternStepperMotorPositionMessage* position_message = (struct SimExternStepperMotorPositionMessage*) message;
		buffer[1] = position_message->base.group_id;
		buffer[2] = (uint8_t)((position_message->position >> 8) & 0xFF);
		buffer[3] = (uint8_t)((position_message->position >> 0) & 0xFF);
		buffer[4] = (uint8_t)position_message->direction;
		len = 5;
		break;
	}
#endif

	case SIM_EXTERN_MESSAGE_TYPE_PING:
	case SIM_EXTERN_MESSAGE_TYPE_PONG:
	case SIM_EXTERN_MESSAGE_TYPE_DISABLE_ALL:
		len = 1;
		break;

#if !defined(SIM_EXTERN_OMIT_HOST_TO_CLIENT) & !defined(SIM_EXTERN_OMIT_DAC)
	case SIM_EXTERN_MESSAGE_TYPE_DAC_VALUE_SET: {
		struct SimExternDACValueSetMessage* dac_message = (struct SimExternDACValueSetMessage*) message;
		buffer[1] = dac_message->base.pin_id;

		buffer[2] = (uint8_t)((dac_message->value >> 8) & 0xFF);
		buffer[3] = (uint8_t)((dac_message->value >> 0) & 0xFF);

		len = 4;
		break;
	}
#endif

#ifndef SIM_EXTERN_OMIT_BUTTON_ARRAY
	case SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_PRESSED:
	case SIM_EXTERN_MESSAGE_TYPE_BUTTON_ARRAY_RELEASED: {
		struct SimExternButtonArrayChangeMessage* change_message = (struct SimExternButtonArrayChangeMessage*) message;
		buffer[1] = change_message->base.group_id;
		buffer[2] = change_message->row;
		buffer[3] = change_message->column;
		
		len = 4;
		break;
	}
#endif

	case SIM_EXTERN_MESSAGE_TYPE_SET_CHANNEL: {
		struct SimExternSetChannelMessage* change_message = (struct SimExternSetChannelMessage*) message;
		buffer[1] = change_message->channel;
		
		len = 2;
		break;
	}

	default:
		return SI_ERROR;
	}

	// Try to push this message into the output buffer
	return si_output_buffer_mode2_push_data(output_buffer, buffer, len);
}
