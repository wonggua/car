/* Host-only hardware substitutes. Never included in car.uvprojx. */
#include <stddef.h>
#include "car_board.h"
static unsigned int servo = 90, beep, messages;
void Board_Servo(unsigned int angle) { servo = angle; }
void Board_Beep(uint32_t ms) { beep = ms; }
void Board_Print(unsigned int port, const char *s) { (void)port; (void)s; ++messages; }
void Board_Status(void) { ++messages; }
unsigned int Test_Servo(void) { return servo; }
unsigned int Test_Beep(void) { return beep; }
void *memset(void *p, int c, size_t n) { unsigned char *b = p; while (n--) *b++ = (unsigned char)c; return p; }
void *memcpy(void *d, const void *s, size_t n) { unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
int strcmp(const char *a, const char *b) { while (*a && *a == *b) { ++a; ++b; } return (unsigned char)*a - (unsigned char)*b; }
int strncmp(const char *a, const char *b, size_t n) { while (n--) { if (*a != *b || !*a) return (unsigned char)*a - (unsigned char)*b; ++a; ++b; } return 0; }
