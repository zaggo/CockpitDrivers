#ifndef DIGITSFONT_H
#define DIGITSFONT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <avr/pgmspace.h>

typedef struct
{    
  const uint8_t *table;
  uint16_t width;
  uint16_t height;
} Font;

extern Font digitsFont;

#ifdef __cplusplus
}
#endif
  
#endif /* DIGITSFONT_H */