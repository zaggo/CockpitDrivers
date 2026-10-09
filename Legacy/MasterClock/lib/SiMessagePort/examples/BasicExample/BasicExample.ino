/*
	Sim Innovations Message Port example:
	
	In this example we can communicate with Air Manager or Air Player over a standard USB cable.
	This is done using the Message Port library.
	
	See the code below on how to implement this library.
	
	More information on how to implement the Air Manager or Air Player side can be found here:
	https://siminnovations.com/wiki/index.php?title=Hw_message_port_add
	
	
	NOTE: 
	The Message Port library communicates with the PC using Serial Port 0 of the Arduino.
	Do not use Serial port 0 yourself!
*/

#include <si_message_port.hpp>

SiMessagePort* messagePort;

static void new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
	// Do something with a message from Air Manager or Air Player

	// The arguments are only valid within this function!
	// Make a clone if you want to store it

	if (payload == NULL) {
		messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received without payload");
	}
	else {
		switch(payload->type) {
			case SI_MESSAGE_PORT_DATA_TYPE_BYTE:
				messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received " + payload->len + " bytes: " + payload->data_byte[0]);
				break;
			case SI_MESSAGE_PORT_DATA_TYPE_STRING:
				messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received string: " + payload->data_string);
				break;
			case SI_MESSAGE_PORT_DATA_TYPE_INTEGER:
				messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received " + payload->len + " integers" + payload->data_int[0]);
				break;
			case SI_MESSAGE_PORT_DATA_TYPE_FLOAT:
				messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received " + payload->len + " floats" + payload->data_float[0]);
				break;
		}
	}
}

void setup() {
	// Init library on channel A and Arduino type MEGA 2560
	messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560, SI_MESSAGE_PORT_CHANNEL_A, new_message_callback);
}

void loop() {
	// Make sure this function is called regularly
	messagePort->Tick();

	// You can send your own messages to Air Manager or Air Player
	//messagePort->SendMessage(123);
	//messagePort->SendMessage(123, "hello");
	//messagePort->SendMessage(123, (int32_t)1000);
	//messagePort->SendMessage(123, 2.5f);
	//messagePort->SendMessage(123, (uint8_t) 0xAA);
}