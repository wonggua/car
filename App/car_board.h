#ifndef CAR_BOARD_H
#define CAR_BOARD_H
#include "car_control.h"
void Board_Init(void);
uint32_t Board_Millis(void);
void Board_Service(void);
int Board_PollPS2(uint32_t now); /* 1 when a new valid frame is available */
void Board_Read(CarInput *in);
void Board_Write(const CarOutput *out);
void Board_Servo(unsigned int angle);
void Board_Beep(uint32_t duration);
int Board_GetByte(unsigned int port);
int Board_KeyEvent(void); /* 1=短按，2=长按；按下即停止的事件为3。 */
int Board_IRCommand(void);
int Board_RxOverflow(void);
void Board_Print(unsigned int port, const char *text);
void Board_Status(void);
#endif
