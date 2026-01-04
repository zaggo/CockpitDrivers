#ifndef SI_NETWORK_H
#define SI_NETWORK_H

#include "si_base.h"

#ifndef SI_EMBEDDED
#include "si_transaction_response.h"
#include "si_data.h"
#include "si_rect.h"
#include "si_data_packet.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

enum SiNetworkEndianess {
	SI_NETWORK_ENDIANESS_BIG,
	SI_NETWORK_ENDIANESS_LITTLE
};

#ifdef IOS
#include <arpa/inet.h>
#endif

#ifdef MAC
#include <arpa/inet.h>
#endif

#ifdef LIN
#include <arpa/inet.h>
#endif

#ifdef ESP_PLATFORM
#endif

#ifdef SI_PLATFORM_SAM54
#include <lwip/def.h>
#endif

#ifdef ANDROID
#include <arpa/inet.h>
#endif

#ifdef WASM
#include <arpa/inet.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


#if defined(SI_EMBEDDED) | defined(ESP_PLATFORM)

// We expect all embedded systems to be little endian
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#else
// On desktop systems, use OS implementation of conversion functions
#include "si_socket.h"
#endif

enum SiNetworkEndianess si_network_get_endianess(void);

int16_t si_network_int16_to_network(int16_t value);
int16_t si_network_int16_to_host(int16_t value);

int32_t si_network_int32_to_network(int32_t value);
int32_t si_network_int32_to_host(int32_t value);

float si_network_float_to_network(float value);
float si_network_float_to_host(float value);

double si_network_double_to_network(double value);
double si_network_double_to_host(double value);

float si_network_float(uint8_t* x);
float si_network_swap_float(float value);

#define SI_NETWORK_CHAR_TO_HOST(network_value) (network_value)
#define SI_NETWORK_CHAR_TO_NETWORK(host_value)  (host_value)

#define SI_NETWORK_UINT16_TO_HOST(network_value) (uint16_t) ntohs(network_value)
#define SI_NETWORK_UINT16_TO_NETWORK(host_value) (uint16_t) htons(host_value)

#define SI_NETWORK_UINT32_TO_HOST(network_value) (uint32_t) ntohl(network_value)
#define SI_NETWORK_UINT32_TO_NETWORK(host_value) (uint32_t) htonl(host_value)

#define SI_NETWORK_INT16_TO_HOST(network_value) ( si_network_int16_to_host(network_value) )
#define SI_NETWORK_INT16_TO_NETWORK(host_value) ( si_network_int16_to_network(host_value) )

#define SI_NETWORK_INT32_TO_HOST(network_value) ( si_network_int32_to_host(network_value) )
#define SI_NETWORK_INT32_TO_NETWORK(host_value) ( si_network_int32_to_network(host_value) )

#define SI_NETWORK_FLOAT_TO_HOST(network_value) (si_network_float_to_host(network_value))
#define SI_NETWORK_FLOAT_TO_NETWORK(host_value) (si_network_float_to_network(host_value))

#define SI_NETWORK_DOUBLE_TO_HOST(network_value)  ( si_network_double_to_host(network_value)  )
#define SI_NETWORK_DOUBLE_TO_NETWORK(host_value)  ( si_network_double_to_network(host_value)  )

size_t si_network_stream_uint64_size(void);
uint8_t* si_network_stream_uint64_to_host(uint8_t* data, uint64_t* integer);
uint8_t* si_network_stream_uint64_to_network(uint8_t* data, uint64_t integer);

size_t si_network_stream_uint32_size(void);
uint8_t* si_network_stream_uint32_to_host(uint8_t* data, uint32_t* integer);
uint8_t* si_network_stream_uint32_to_network(uint8_t* data, uint32_t integer);

size_t si_network_stream_bool_size(void);
uint8_t* si_network_stream_bool_to_host(uint8_t* data, SI_BOOL* value);
uint8_t* si_network_stream_bool_to_network(uint8_t* data, SI_BOOL value);

size_t si_network_stream_int64_size(void);
uint8_t* si_network_stream_int64_to_host(uint8_t* data, int64_t* integer);
uint8_t* si_network_stream_int64_to_network(uint8_t* data, int64_t integer);

size_t si_network_stream_int32_size(void);
uint8_t* si_network_stream_int32_to_host(uint8_t* data, int32_t* integer);
uint8_t* si_network_stream_int32_to_network(uint8_t* data, int32_t integer);

size_t si_network_stream_float_size(void);
uint8_t* si_network_stream_float_to_host(uint8_t* data, float* value);
uint8_t* si_network_stream_float_to_network(uint8_t* data, float value);

size_t si_network_stream_double_size(void);
uint8_t* si_network_stream_double_to_host(uint8_t* data, double* value);
uint8_t* si_network_stream_double_to_network(uint8_t* data, double value);

#ifndef SI_EMBEDDED
size_t si_network_stream_string_size(const char* string);
uint8_t* si_network_stream_string_to_host(uint8_t* data, const char** string);
uint8_t* si_network_stream_string_to_network(uint8_t* data, const char* string);

size_t si_network_stream_data_size(struct SiData* data);
uint8_t* si_network_stream_data_to_host(uint8_t* data, struct SiData** data_blob);
uint8_t* si_network_stream_data_to_network(uint8_t* data, struct SiData* data_blob);

size_t si_network_stream_data_packet_size(struct SiDataPacket* data);
uint8_t* si_network_stream_data_packet_to_host(uint8_t* data, struct SiDataPacket** data_blob);
uint8_t* si_network_stream_data_packet_to_network(uint8_t* data, struct SiDataPacket* data_blob);

size_t si_network_stream_transaction_response_size(struct SiTransactionResponse* response);
uint8_t* si_network_stream_transaction_response_to_host(uint8_t* data, struct SiTransactionResponse** response);
uint8_t* si_network_stream_transaction_response_to_network(uint8_t* data, struct SiTransactionResponse* response);

size_t si_network_stream_rect_float_size(void);
uint8_t* si_network_stream_rect_float_to_host(uint8_t* data, struct SiRectFloat* rect);
uint8_t* si_network_stream_rect_float_to_network(uint8_t* data, struct SiRectFloat rect);

size_t si_network_stream_raw_data_size(size_t data_len);
uint8_t* si_network_stream_raw_data_to_host(uint8_t* data, uint8_t** input_data, size_t* len);
uint8_t* si_network_stream_raw_data_to_network(uint8_t* data, uint8_t* input_data, size_t len);
#endif

#ifdef __cplusplus
}
#endif


#endif
