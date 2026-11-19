#include "car_protocol.h"
#include "car_board.h"
#include "car_control.h"
#include <string.h>
typedef struct { char text[48]; unsigned int length, active; uint32_t since; } Parser;
static Parser parsers[2];
void Protocol_Reset(void)
{
    parsers[0].active = parsers[1].active = 0;
    parsers[0].length = parsers[1].length = 0;
}
static int number(const char *s, uint32_t maximum)
{
    uint32_t n = 0;
    if (!*s) return -1;
    while (*s) {
        if (*s < '0' || *s > '9') return -1;
        n = n * 10U + (uint32_t)(*s++ - '0');
        if (n > maximum) return -1;
    }
    return (int)n;
}
static int frame(const char *s, uint32_t now)
{
    int value;
    if (strcmp(s, "STOP") == 0) Control_Run(0, now);
    else if (strcmp(s, "RUN") == 0) { Board_Servo(90); Control_Run(1, now); }
    else if (strcmp(s, "STATUS") == 0) Board_Status();
    else if (strncmp(s, "MODE=", 5) == 0) {
        value = number(s + 5, MODE_COUNT - 1U);
        if (value < 0) return 0;
        Board_Servo(90); Control_SetMode((uint32_t)value, now);
    } else if (strncmp(s, "SPEED=", 6) == 0) {
        value = number(s + 6, 10000U); if (value < 0) return 0;
        Control_Speed((uint32_t)value);
    } else if (strncmp(s, "SERVO=", 6) == 0) {
        value = number(s + 6, 160U); if (value < 20) return 0;
        Control_Run(0, now); Control_SetServo((uint32_t)value); Board_Servo((unsigned int)value);
    } else if (strncmp(s, "MOVE=", 5) == 0) {
        value = number(s + 5, MOVE_ARC_RIGHT); if (value < 0) return 0;
        Control_Move((uint32_t)value, now);
    } else if (strncmp(s, "LIGHT=", 6) == 0) {
        value = number(s + 6, 1U); if (value < 0) return 0;
        Control_Lights((uint32_t)value);
    } else if (strcmp(s, "BEEP") == 0) Board_Beep(150);
    else return 0;
    return 1;
}
static void single(unsigned int c, uint32_t now)
{
    uint32_t mode = MODE_COUNT;
    switch (c) {
    case '1': case 1: case 'w': case 'W': Control_Move(MOVE_FORWARD, now); return;
    case '2': case 2: case 's': case 'S': Control_Move(MOVE_BACK, now); return;
    case '3': case 3: case 'a': case 'A': Control_Move(MOVE_LEFT, now); return;
    case '4': case 4: case 'd': case 'D': Control_Move(MOVE_RIGHT, now); return;
    case '5': case 5: Control_Move(MOVE_ARC_LEFT, now); return;
    case '6': case 6: Control_Move(MOVE_ARC_RIGHT, now); return;
    case '0': case 0: case 'x': case 'X': Control_Run(0, now); return;
    case 'P': mode = MODE_PS2; break;
    case 'M': mode = MODE_MANUAL; break;
    case 'T': mode = MODE_TRACK; break;
    case 'U': mode = MODE_ULTRASONIC; break;
    case 'I': mode = MODE_IR_AVOID; break;
    case 'C': mode = MODE_COMBINED; break;
    case 'F': mode = MODE_FOLLOW; break;
    case 'L': mode = MODE_LIGHT; break;
    case 'G': mode = MODE_RGB; break;
    case 'Z': mode = MODE_DISTANCE; break;
    case 'R': Board_Servo(90); Control_Run(1, now); return;
    case 'H': Board_Beep(150); return;
    case 'O': Control_Lights(0); return;
    case '?': Board_Status(); return;
    case '\r': case '\n': case ' ': return;
    default: Control_Run(0, now); return;
    }
    if (mode < MODE_COUNT) { Board_Servo(90); Control_SetMode(mode, now); }
}
void Protocol_Byte(unsigned int port, unsigned int byte, uint32_t now)
{
    Parser *p = &parsers[port];
    if (byte == '@') { p->active = 1; p->length = 0; p->since = now; return; }
    if (!p->active) { single(byte, now); return; }
    if (p->active == 2) { /* 溢出后整帧丢弃，避免尾部数字变成电机指令。 */
        if (byte == '\r' || byte == '\n') p->active = 0;
        return;
    }
    if (byte == '\r' || byte == '\n') {
        p->text[p->length] = 0;
        if (frame(p->text, now)) Board_Print(port, "OK\r\n");
        else { Control_Run(0, now); Board_Print(port, "ERR\r\n"); }
        p->active = 0;
    } else if (p->length >= sizeof(p->text) - 1U || byte < 32U || byte > 126U) {
        p->active = 2; Control_Run(0, now); Board_Print(port, "ERR length\r\n");
    } else p->text[p->length++] = (char)byte;
}
void Protocol_Poll(uint32_t now)
{
    unsigned int port;
    for (port = 0; port < 2; ++port) {
        if (parsers[port].active == 1 && now - parsers[port].since >= 1000U) {
            parsers[port].active = 2; Control_Run(0, now);
            Board_Print(port, "ERR timeout\r\n");
        }
    }
}
