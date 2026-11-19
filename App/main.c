#include "car_board.h"
#include "car_control.h"
#include "car_protocol.h"

int main(void)
{
    uint32_t last_control = 0, last_status = 0;
    unsigned int was_running = 0;
    CarInput input;
    CarOutput output;
    Control_Init(); Board_Init(); Protocol_Reset();
    Board_Print(0, "car ready: M/T/U/I/C/F/L/G/Z/P mode, R run, X stop, ? status\r\n");
    Board_Print(1, "car ready; @MODE=1 then @RUN; 1..6 move, 0 stop\r\n");
    for (;;) {
        unsigned int port, n;
        int byte, event, code;
        uint32_t now = Board_Millis();
        Board_Service();
        if (Board_RxOverflow()) { Protocol_Reset(); Control_Run(0, now); }
        for (port = 0; port < 2; ++port) {
            for (n = 0; n < 32U && (byte = Board_GetByte(port)) >= 0; ++n)
                Protocol_Byte(port, (unsigned int)byte, now);
        }
        Protocol_Poll(now);
        event = Board_KeyEvent();
        if (event == 3) {
            was_running = Control_State()->running;
            Control_Run(0, now); /* 按下先停，消抖不在中断内等待。 */
        } else if (event == 1) {
            Board_Servo(90); Control_Run(!was_running, now); Board_Beep(60);
        } else if (event == 2) {
            Control_SetMode((Control_State()->mode + 1U) % MODE_COUNT, now);
            Board_Servo(90); Board_Beep(180); Board_Status();
        }
        code = Board_IRCommand();
        Control_IR(code, now);
        if (now - last_control >= 5U) {
            last_control = now;
            Board_Read(&input); Control_Step(&input, &output); Board_Write(&output);
        }
        if (now - last_status >= 1000U) { last_status = now; Board_Status(); }
    }
}
