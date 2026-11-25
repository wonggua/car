#ifndef CAR_PS2_H
#define CAR_PS2_H
#include <stdint.h>
typedef struct { uint32_t buttons, lx, ly, id; } PS2Data;
int PS2_Decode(const uint8_t *data, unsigned int length, PS2Data *out);
#endif
