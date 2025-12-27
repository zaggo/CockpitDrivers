#include "si_circular_data.h"

#include <string.h>

void si_circular_data_init(struct SiCircularData* data, uint8_t* buffer, const uint8_t len) {
    data->buffer = buffer;
    
    data->available = 0;
    data->len = len;
    
    data->read_offset = 0;
    data->write_offset = 0;
}

uint8_t si_circular_data_total_size(const struct SiCircularData* data) {
	return data->len;
}

uint8_t si_circular_data_available(const struct SiCircularData* data) {
	return data->available;
}

uint8_t si_circular_data_free(struct SiCircularData* data) {
	return data->len - data->available;
}

enum SiResult si_circular_push(struct SiCircularData* data, uint8_t byte) {
    if (data->available < data->len) {
        
        data->buffer[data->write_offset] = byte;
        data->write_offset++;
        
        if (data->write_offset >= data->len) {
            data->write_offset = 0;
        }
        
        data->available++;
        
        return SI_OK;
    }
    else {
        return SI_ERROR;
    }
}

enum SiResult si_circular_peek(const struct SiCircularData* data, const uint8_t offset, uint8_t* byte) {
    if (offset < data->available) {
		if (byte != NULL) {
			if ( (data->read_offset + offset) < data->len) {
				*byte = data->buffer[data->read_offset + offset];
			}
			else {
				*byte = data->buffer[data->read_offset + offset - data->len];
			}
		}

        return SI_OK;
    }
    else {
        return SI_ERROR;
    }
}

enum SiResult si_circular_poll(struct SiCircularData* data, const uint8_t offset, uint8_t* byte) {
    if (si_circular_peek(data, offset, byte) == SI_OK) {
	    
		if (offset == 0) {
			// Update read offset
			data->read_offset++;
			if (data->read_offset >= data->len) {
				data->read_offset = 0;
			}
		}
		else {
			if (data->write_offset >= data->read_offset) {
				// All data is nicely aligned in order
				// ----- R ======= W -----

				// Move all bytes one step back, so this polled byte gets removed
				uint8_t i;
				for (i = data->read_offset + offset; i < (data->available + data->read_offset - 1); i++) {
					data->buffer[i] = data->buffer[i + 1];
				}
			}
			else {
				// Data is wrapped
				// ==== W -------- R ====

				// Check if the offset is at the begin of the buffer or at the end
				if ((data->read_offset + offset) < data->len) {
					// Offset is at the end

					// First move the data at the end forward
					uint8_t i;
					for (i = data->read_offset + offset; i < (data->len - 1); i++) {
						data->buffer[i] = data->buffer[i + 1];
					}

					// Then move the first byte at the start of the buffer back to the end
					data->buffer[data->len - 1] = data->buffer[0];

					// Move data at the front of the buffer
					for (i = 0; i < (data->write_offset - 1); i++) {
						data->buffer[i] = data->buffer[i + 1];
					}
				}
				else {
					// Move data at the front of the buffer
					uint8_t i;
					for (i = (offset + data->read_offset - data->len); i < (data->write_offset - 1); i++) {
						data->buffer[i] = data->buffer[i + 1];
					}
				}
			}

			// Update write offset
			if (data->write_offset == 0) {
				data->write_offset = data->len - 1;
			}
			else {
				data->write_offset--;
			}
		}

		data->available--;
		
		return SI_OK;
    }
    else {
	    return SI_ERROR;
    }
}

enum SiResult si_circular_forward(struct SiCircularData* data, const uint8_t len) {
	if (len <= data->available) {
		if ((data->read_offset + len) < data->len) {
			data->available -= len;
			data->read_offset += len;
        }
        else {
			data->available -= len;
			data->read_offset = len - (data->len - data->read_offset);
        }
        
        return SI_OK;
    }
    else {
         return SI_ERROR;
    }
}

uint8_t si_circular_data_copy(struct SiCircularData* source, struct SiCircularData* destination) {
	uint8_t byte;
	uint8_t len = 0;
	while(si_circular_data_free(destination) > 0) {
		if (si_circular_poll(source, 0, &byte) == SI_OK) {
			si_circular_push(destination, byte);
			len++;
		}
		else {
			break;
		}
	}
	
	return len;
}
