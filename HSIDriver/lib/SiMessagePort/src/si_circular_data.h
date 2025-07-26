#ifndef SI_CIRCULAR_DATA
#define SI_CIRCULAR_DATA

#include "si_base.h"

#ifdef __cplusplus
extern "C"{
#endif

struct SiCircularData {
	uint8_t* buffer;
	uint8_t len;
	
	uint8_t read_offset;
	uint8_t write_offset;
	
	uint8_t available;
};

void si_circular_data_init(struct SiCircularData* data, uint8_t* buffer, const uint8_t len);

// Returns number of bytes of the buffer (fixed)
uint8_t si_circular_data_total_size(const struct SiCircularData* data);

// Returns number of bytes available in the buffer
uint8_t si_circular_data_available(const struct SiCircularData* data);

// Returns number of free bytes in the buffer
uint8_t si_circular_data_free(struct SiCircularData* data);

// Push another byte to the end
enum SiResult si_circular_push(struct SiCircularData* data, uint8_t byte);

// Gets the byte at the given offset
enum SiResult si_circular_peek(const struct SiCircularData* data, const uint8_t offset, uint8_t* byte);

// Gets and removes the byte at the given offset
enum SiResult si_circular_poll(struct SiCircularData* data, const uint8_t offset, uint8_t* byte);

// Forward x number of bytes forward. Used when required data has been read and can be free'd up again
enum SiResult si_circular_forward(struct SiCircularData* data, const uint8_t len);

// Copy bytes from one circular data to another
uint8_t si_circular_data_copy(struct SiCircularData* source, struct SiCircularData* destination);

#ifdef __cplusplus
}
#endif

#endif