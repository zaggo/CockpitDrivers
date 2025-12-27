#include "si_network.h"

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)
#define FP_CONVERT(A) ((((uint64_t)(A) & 0xffffffff00000000LL) >> 32) | (((uint64_t)(A) & 0x00000000ffffffffLL) << 32))

int16_t si_network_int16_to_network(int16_t value) {
	uint16_t temp = htons(*((uint16_t*)&value));
	return *(int16_t*)&temp;
}

int16_t si_network_int16_to_host(int16_t value) {
	uint16_t temp = ntohs(*((uint16_t*)&value));
	return *(int16_t*)&temp;
}

int32_t si_network_int32_to_network(int32_t value) {
	uint32_t temp = htonl(*((uint32_t*)&value));
	return *(int32_t*)&temp;
}

int32_t si_network_int32_to_host(int32_t value) {
	uint32_t temp = ntohl(*((uint32_t*)&value));
	return *(int32_t*)&temp;
}

float si_network_float_to_network(float value) {
	float temp = (float)htonl((uint32_t)value);
	return temp;
}

float si_network_float_to_host(float value) {
	float temp = (float)ntohl((uint32_t)value);
	return temp;
}

double si_network_double_to_network(double value) {
#ifdef WIN
	uint64_t network_val = htond(value);
	return *(double*)&network_val;
#else

	union {
		uint8_t  byte[8];
		uint64_t integer;
		double   floating;
		uint32_t integer32[2];
	} ret;
	
	ret.floating = value;

#ifdef IS_BIG_ENDIAN
	uint64_t tmp;
	tmp = FP_CONVERT(ret.integer);
	ret.integer = tmp;

	ret.integer32[0] = ntohl(ret.integer32[0]);
	ret.integer32[1] = ntohl(ret.integer32[1]);
#endif

	return ret.floating;
#endif
}


double si_network_double_to_host(double value) {
#ifdef WIN
	uint64_t temp = *((uint64_t*)&value);
	return ntohd(temp);
#else
	union {
		uint8_t  byte[8];
		uint64_t integer;
		double   floating;
		uint32_t integer32[2];
	} ret;

    ret.floating = value;

#ifdef IS_BIG_ENDIAN
	uint64_t tmp;
	tmp = FP_CONVERT(ret.integer);
	ret.integer = tmp;

	ret.integer32[0] = ntohl(ret.integer32[0]);
	ret.integer32[1] = ntohl(ret.integer32[1]);
#endif

	return ret.floating;
#endif
}

float si_network_float(uint8_t* x) {
    union {
        uint8_t  byte[4];
        uint32_t integer;
        float   floating;
    } ret;

    memcpy(ret.byte, x, 4);

#ifdef IS_BIG_ENDIAN
    ret.integer = ntohl(ret.integer);
#endif

    return ret.floating;
}


float si_network_swap_float(float value) {
    union v {
        float       f;
        uint32_t    i;
    };

    union v val;

    val.f = value;
    val.i = htonl(val.i);

    return val.f;
}

size_t si_network_stream_uint64_size(void) {
    return 8;
}

uint8_t* si_network_stream_uint64_to_host(uint8_t* data, uint64_t* integer) {
    uint64_t in = *((uint64_t*)data);

#ifndef IS_BIG_ENDIAN
    *integer = (in && 0xff00000000000000ULL) >> 56
             | (in && 0x00ff000000000000ULL) >> 40
             | (in && 0x0000ff0000000000ULL) >> 24
             | (in && 0x000000ff00000000ULL) >> 8
             | (in && 0x00000000ff000000ULL) << 8
             | (in && 0x0000000000ff0000ULL) << 24
             | (in && 0x000000000000ff00ULL) << 40
             | (in && 0x00000000000000ffULL) << 56;
#else
    *integer = in;
#endif

    return data + 8;
}

uint8_t* si_network_stream_uint64_to_network(uint8_t* data, uint64_t integer) {
    uint64_t network_integer = integer;
#ifndef IS_BIG_ENDIAN
    network_integer = 
          (integer && 0xff00000000000000ULL) >> 56
        | (integer && 0x00ff000000000000ULL) >> 40
        | (integer && 0x0000ff0000000000ULL) >> 24
        | (integer && 0x000000ff00000000ULL) >> 8
        | (integer && 0x00000000ff000000ULL) << 8
        | (integer && 0x0000000000ff0000ULL) << 24
        | (integer && 0x000000000000ff00ULL) << 40
        | (integer && 0x00000000000000ffULL) << 56;
#endif

    memcpy(data, &network_integer, 8);
    data += 8;

    return data;
}


size_t si_network_stream_uint32_size(void) {
    return 4;
}
uint8_t* si_network_stream_uint32_to_host(uint8_t* data, uint32_t* integer) {
    *integer = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);
    return data + 4;
}
uint8_t* si_network_stream_uint32_to_network(uint8_t* data, uint32_t integer) {
    uint32_t network_integer = SI_NETWORK_UINT32_TO_NETWORK(integer);
    memcpy(data, &network_integer, 4);
    data += 4;

    return data;
}

size_t si_network_stream_bool_size(void) {
	return 4;
}
uint8_t* si_network_stream_bool_to_host(uint8_t* data, SI_BOOL* value) {
    *value = (SI_BOOL) SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);
    return data + 4;	
}
uint8_t* si_network_stream_bool_to_network(uint8_t* data, SI_BOOL value) {
	uint32_t val = (uint32_t)value;
    uint32_t network_integer = SI_NETWORK_UINT32_TO_NETWORK(val);
    memcpy(data, &network_integer, 4);
    data += 4;

    return data;	
}

size_t si_network_stream_int64_size(void) {
    return 8;
}

uint8_t* si_network_stream_int64_to_host(uint8_t* data, int64_t* integer) {
    int64_t in = *((int64_t*)data);

#ifndef IS_BIG_ENDIAN
    * integer = 
        (in && 0xff00000000000000ULL) >> 56
      | (in && 0x00ff000000000000ULL) >> 40
      | (in && 0x0000ff0000000000ULL) >> 24
      | (in && 0x000000ff00000000ULL) >> 8
      | (in && 0x00000000ff000000ULL) << 8
      | (in && 0x0000000000ff0000ULL) << 24
      | (in && 0x000000000000ff00ULL) << 40
      | (in && 0x00000000000000ffULL) << 56;
    #else
    * integer = in;
#endif

    return data + 8;
}

uint8_t* si_network_stream_int64_to_network(uint8_t* data, int64_t integer) {
    int64_t network_integer = integer;
#ifndef IS_BIG_ENDIAN
    network_integer =
        (integer && 0xff00000000000000ULL) >> 56
      | (integer && 0x00ff000000000000ULL) >> 40
      | (integer && 0x0000ff0000000000ULL) >> 24
      | (integer && 0x000000ff00000000ULL) >> 8
      | (integer && 0x00000000ff000000ULL) << 8
      | (integer && 0x0000000000ff0000ULL) << 24
      | (integer && 0x000000000000ff00ULL) << 40
      | (integer && 0x00000000000000ffULL) << 56;
#endif

    memcpy(data, &network_integer, 8);
    data += 8;

    return data;
}

size_t si_network_stream_int32_size(void) {
    return 4;
}
uint8_t* si_network_stream_int32_to_host(uint8_t* data, int32_t* integer) {
    *integer = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);
    return data + 4;
}
uint8_t* si_network_stream_int32_to_network(uint8_t* data, int32_t integer) {
    int32_t network_integer = SI_NETWORK_UINT32_TO_NETWORK(integer);
    memcpy(data, &network_integer, 4);
    data += 4;

    return data;
}

size_t si_network_stream_float_size(void) {
	return 4;
}

uint8_t* si_network_stream_float_to_host(uint8_t* data, float* value) {
	*value = SI_NETWORK_FLOAT_TO_HOST(*(float*)data);
	return data + 4;
}

uint8_t* si_network_stream_float_to_network(uint8_t* data, float value) {
	float network_float = SI_NETWORK_FLOAT_TO_NETWORK(value);
	memcpy(data, &network_float, 4);
	data += 4;

	return data;
}


size_t si_network_stream_double_size(void) {
    return 8;
}
uint8_t* si_network_stream_double_to_host(uint8_t* data, double* value) {
    *value = SI_NETWORK_DOUBLE_TO_HOST(*(double*)data);
    return data + 8;
}
uint8_t* si_network_stream_double_to_network(uint8_t* data, double value) {
    double network_double = SI_NETWORK_DOUBLE_TO_NETWORK(value);
    memcpy(data, &network_double, 8);
    data += 8;

    return data;
}

#ifndef SI_EMBEDDED
size_t si_network_stream_string_size(const char* string) {
    if (string == NULL) {
        return si_network_stream_uint32_size();
    }
    else {
        return si_network_stream_uint32_size() + strlen(string);
    }
}
uint8_t* si_network_stream_string_to_host(uint8_t* data, const char** string) {
    uint32_t len = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);

    if (len == 0xFFFFFFFF) {
        *string = NULL;
        return data + si_network_stream_uint32_size();
    }
    else {
        char* buffer = (char*)malloc(len + 1);
        memcpy(buffer, data + si_network_stream_uint32_size(), len);
        buffer[len] = '\0';

        *string = buffer;

        return data + (si_network_stream_uint32_size() + len);
    }
}
uint8_t* si_network_stream_string_to_network(uint8_t* data, const char* string) {
    if (string == NULL) {
        data = si_network_stream_uint32_to_network(data, 0xFFFFFFFF);
    }
    else {
        uint32_t len = (uint32_t)strlen(string);
        data = si_network_stream_uint32_to_network(data, len);

        memcpy(data, string, len);
        data += len;
    }
    return data;
}


size_t si_network_stream_data_size(struct SiData* data) {
    return si_network_stream_uint32_size() + (data != NULL ? data->len : 0);
}
uint8_t* si_network_stream_data_to_host(uint8_t* data, struct SiData** data_blob) {
    uint32_t len = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);
	if (len == UINT32_MAX) {
		*data_blob = NULL;
	}
	else {
		*data_blob = si_data_create(data + si_network_stream_uint32_size(), len);
	}
    return data + (si_network_stream_uint32_size() + len);
}

uint8_t* si_network_stream_data_to_network(uint8_t* data, struct SiData* data_blob) {
    data = si_network_stream_uint32_to_network(data, data_blob != NULL ? (uint32_t) data_blob->len : UINT32_MAX);
	if (data_blob != NULL) {
		memcpy(data, data_blob->buffer, data_blob->len);
	}
    return data + (data_blob != NULL ? data_blob->len : 0);
}

size_t si_network_stream_data_packet_size(struct SiDataPacket* packet) {
    return packet == NULL ? 0 : si_data_packet_byte_size(packet);
}

uint8_t* si_network_stream_data_packet_to_host(uint8_t* data, struct SiDataPacket** packet) {
    *packet = si_data_packet_create();
    return si_data_packet_load_bytes(*packet, data);
}

uint8_t* si_network_stream_data_packet_to_network(uint8_t* data, struct SiDataPacket* packet) {
    return si_data_packet_save_bytes(packet, data);
}


size_t si_network_stream_transaction_response_size(struct SiTransactionResponse* response) {
	return response == NULL ? 0 : 
        si_network_stream_uint32_size() + // state
        si_network_stream_double_size() + // percentage
		si_network_stream_uint64_size() + // bytes_total
        si_network_stream_string_size(response->error) + // Error
		si_network_stream_string_size(response->result); // Result
}

uint8_t* si_network_stream_transaction_response_to_host(uint8_t* data, struct SiTransactionResponse** response) {
	*response = (struct SiTransactionResponse*)malloc(sizeof(struct SiTransactionResponse));

    uint32_t state_value;

	data = si_network_stream_uint32_to_host(data, &state_value);
	(*response)->state = (enum SiTransactionState)state_value;

	data = si_network_stream_double_to_host(data, &(*response)->percentage);
	data = si_network_stream_uint64_to_host(data, &(*response)->bytes_total);
	data = si_network_stream_string_to_host(data, &(*response)->error);
	data = si_network_stream_string_to_host(data, &(*response)->result);

    return data;
}

uint8_t* si_network_stream_transaction_response_to_network(uint8_t* data, struct SiTransactionResponse* response) {
	data = si_network_stream_uint32_to_network(data, (uint32_t)response->state);
	data = si_network_stream_double_to_network(data, response->percentage);
	data = si_network_stream_uint64_to_network(data, response->bytes_total);
	data = si_network_stream_string_to_network(data, response->error);
	data = si_network_stream_string_to_network(data, response->result);

    return data;
}


size_t si_network_stream_rect_float_size(void) {
	return 16;
}

uint8_t* si_network_stream_rect_float_to_host(uint8_t* data, struct SiRectFloat* rect) {
	data = si_network_stream_float_to_host(data, &rect->x);
	data = si_network_stream_float_to_host(data, &rect->y);
	data = si_network_stream_float_to_host(data, &rect->w);
	data = si_network_stream_float_to_host(data, &rect->h);

    return data;
}

uint8_t* si_network_stream_rect_float_to_network(uint8_t* data, struct SiRectFloat rect) {
	data = si_network_stream_float_to_network(data, rect.x);
	data = si_network_stream_float_to_network(data, rect.y);
	data = si_network_stream_float_to_network(data, rect.w);
	data = si_network_stream_float_to_network(data, rect.h);

    return data;
}


size_t si_network_stream_raw_data_size(size_t data_len) {
    return data_len + si_network_stream_uint32_size();
}

uint8_t* si_network_stream_raw_data_to_host(uint8_t* data, uint8_t** input_data, size_t* len) {
    *len = SI_NETWORK_UINT32_TO_HOST(*(uint32_t*)data);
    *input_data = (uint8_t*)malloc(*len);
    memcpy(*input_data, data + si_network_stream_uint32_size(), *len);

    return data + (si_network_stream_uint32_size() + *len);
}

uint8_t* si_network_stream_raw_data_to_network(uint8_t* data, uint8_t* input_data, size_t len) {
    data = si_network_stream_uint32_to_network(data, (uint32_t)len);
    memcpy(data, input_data, len);
    return data + len;
}
#endif
