#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdint.h>
enum { MODE_MANUAL, MODE_TRACK, MODE_ULTRASONIC, MODE_IR_AVOID,
       MODE_COMBINED, MODE_FOLLOW, MODE_LIGHT, MODE_RGB, MODE_DISTANCE,
       MODE_PS2, MODE_COUNT };
enum { MOVE_STOP, MOVE_FORWARD, MOVE_BACK, MOVE_LEFT, MOVE_RIGHT,
       MOVE_ARC_LEFT, MOVE_ARC_RIGHT };
/* 所有字段为 32 位，便于调试器和主机测试读取。传感器输入已经归一化。 */
typedef struct {
    uint32_t now, line, obstacle_left, obstacle_right, light_left, light_right;
    uint32_t distance_mm, distance_valid, battery_mv, battery_valid, clock_ok;
    uint32_t ps2_valid, ps2_buttons, ps2_lx, ps2_ly;
    uint32_t sonar_valid, servo_angle;
} CarInput;
typedef struct {
    int32_t left, right;
    uint32_t rgb, beep, servo; /* servo=0 keeps the current target */
} CarOutput;
typedef struct {
    uint32_t mode, running, speed, command, last_command;
    uint32_t phase, phase_since, turn_right, line_seen, line_last;
    int32_t line_direction;
    uint32_t battery_alarm, fault, lights_enabled;
} CarState;
void Control_Init(void);
void Control_IR(int code, uint32_t now);
void Control_SetMode(uint32_t mode, uint32_t now);
void Control_Run(uint32_t enabled, uint32_t now);
void Control_Move(uint32_t command, uint32_t now);
void Control_Speed(uint32_t speed);
void Control_Lights(uint32_t enabled);
void Control_SetServo(uint32_t angle);
void Control_Step(const CarInput *in, CarOutput *out);
const CarState *Control_State(void);
#endif
