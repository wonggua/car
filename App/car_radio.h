#ifndef CAR_RADIO_H
#define CAR_RADIO_H
#include <stdint.h>
int Wifi_Enabled(void);
void Board_RadioBaud(unsigned int baud);
void Wifi_Config(const char *line_text, uint32_t now);
void Wifi_Reset(uint32_t now);
void Wifi_Poll(uint32_t now);
void Wifi_Byte(unsigned int byte, uint32_t now);
void Wifi_Print(const char *text);
void Board_WifiRaw(const char *text);
#endif
