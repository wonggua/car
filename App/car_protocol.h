#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H
#include <stdint.h>
void Protocol_Byte(unsigned int port, unsigned int byte, uint32_t now);
void Protocol_Poll(uint32_t now);
void Protocol_Reset(void);
#endif
