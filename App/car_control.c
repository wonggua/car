#include "car_control.h"
#include "car_config.h"
static CarState state;
static uint32_t ps2_ready, scan_left, scan_right, recenter_pending;
static uint32_t ir_horn_active, ir_horn_started;

const CarState *Control_State(void) { return &state; }
void Control_Init(void)
{
    CarState empty = {0};
    state = empty;
    state.lights_enabled = 1;
    ir_horn_active = ir_horn_started = 0;
    scan_left = scan_right = recenter_pending = 0;
    ps2_ready = 0;
    state.speed = CAR_DEFAULT_SPEED;
}
void Control_Run(uint32_t enabled, uint32_t now)
{
    if (state.phase >= 10U) recenter_pending = 90;
    state.running = enabled != 0;
    ps2_ready = 0;
    state.command = MOVE_STOP;
    state.last_command = now;
    state.phase = 0;
    state.line_seen = 0;
    state.line_last = now;
    state.line_direction = 0;
}
void Control_SetMode(uint32_t mode, uint32_t now)
{
    if (mode >= MODE_COUNT) return;
    Control_Run(0, now);
    state.mode = mode;
}
void Control_Move(uint32_t command, uint32_t now)
{
    if (command > MOVE_ARC_RIGHT) return;
    if (command == MOVE_STOP) { Control_Run(0, now); return; }
    if (state.mode != MODE_MANUAL) Control_SetMode(MODE_MANUAL, now);
    state.running = 1;
    state.command = command;
    state.last_command = now;
}
/* Example 24 NEC command bytes; unknown keys stop. */
void Control_IR(int code, uint32_t now)
{
    switch (code) {
    case 12: ir_horn_started = now; ir_horn_active = 1; break;
    case 94: Control_Lights(0); break;
    case 21: Control_Run(0, now); break; /* User-confirmed stop key. */
    case 64: Control_Move(MOVE_FORWARD, now); break;
    case 25: Control_Move(MOVE_BACK, now); break;
    case 7: Control_Move(MOVE_LEFT, now); break;
    case 9: Control_Move(MOVE_RIGHT, now); break;
    case 22: Control_Move(MOVE_ARC_LEFT, now); break;
    case 13: Control_Move(MOVE_ARC_RIGHT, now); break;
    case -1: break;
    default: Control_Run(0, now); break;
    }
}
void Control_SetServo(uint32_t angle)
{
    if (angle >= 20U && angle <= 160U) recenter_pending = angle;
}
void Control_Speed(uint32_t speed)
{
    if (speed <= 10000U) state.speed = speed;
}
static void motion(CarOutput *o, uint32_t command, int32_t speed)
{
    switch (command) {
    case MOVE_FORWARD: o->left = speed; o->right = speed; break;
    case MOVE_BACK: o->left = -speed; o->right = -speed; break;
    case MOVE_LEFT: o->left = -speed; o->right = speed; break;
    case MOVE_RIGHT: o->left = speed; o->right = -speed; break;
    case MOVE_ARC_LEFT: o->left = 0; o->right = speed; break;
    case MOVE_ARC_RIGHT: o->left = speed; o->right = 0; break;
    default: o->left = 0; o->right = 0; break;
    }
}
static void track(const CarInput *i, CarOutput *o)
{
    int32_t error = 0, count = 0, correction;
    uint32_t mask = i->line & 15U;
    int32_t speed = (int32_t)state.speed;
    /* 从左至右：循迹4/PB10、循迹3/PC15、循迹2/PC13、循迹1/PC14；1 表示检测到黑线。 */
    if (mask & 1U) { error -= 3; ++count; }
    if (mask & 2U) { error -= 1; ++count; }
    if (mask & 4U) { error += 1; ++count; }
    if (mask & 8U) { error += 3; ++count; }
    if (!count) {
        if (state.line_seen && i->now - state.line_last < 350U && state.line_direction)
            motion(o, state.line_direction < 0 ? MOVE_LEFT : MOVE_RIGHT, speed / 2);
        return; /* 初始无黑线或丢线超时，停车等待，不盲目前进。 */
    }
    state.line_seen = 1;
    state.line_last = i->now;
    if (error) state.line_direction = error;
    correction = error * speed / (4 * count);
    o->left = speed + correction;
    o->right = speed - correction;
}
static void avoid(const CarInput *i, CarOutput *o)
{
    uint32_t use_sonar = state.mode != MODE_IR_AVOID;
    uint32_t use_ir = state.mode != MODE_ULTRASONIC;
    uint32_t left = use_ir && i->obstacle_left;
    uint32_t right = use_ir && i->obstacle_right;
    uint32_t front = use_sonar && i->distance_mm < CAR_OBSTACLE_MM;
    int32_t speed = (int32_t)state.speed;
    if (use_sonar && !i->distance_valid) { state.phase = 0; return; }
    if (state.phase == 1 && i->now - state.phase_since >= 250U) {
        state.phase = 2; state.phase_since = i->now;
    }
    if (state.phase == 2 && i->now - state.phase_since >= 350U) state.phase = 0;
    if (!state.phase && (front || left || right)) {
        state.phase = 1; state.phase_since = i->now;
        if (left != right) state.turn_right = left;
        else state.turn_right = !state.turn_right;
    }
    if (state.phase == 1) motion(o, MOVE_BACK, speed * 2 / 3);
    else if (state.phase == 2) motion(o, state.turn_right ? MOVE_RIGHT : MOVE_LEFT, speed * 2 / 3);
    else motion(o, MOVE_FORWARD, speed);
}
static void scan_avoid(const CarInput *i, CarOutput *o)
{
    uint32_t age = i->now - state.phase_since;
    uint32_t blocked_left = state.mode == MODE_COMBINED && i->obstacle_left;
    uint32_t blocked_right = state.mode == MODE_COMBINED && i->obstacle_right;
    o->servo = 90;
    if (state.phase == 10U) {
        o->servo = CAR_SCAN_LEFT_ANGLE;
        if (i->servo_angle == CAR_SCAN_LEFT_ANGLE && i->sonar_valid && age >= 300U) {
            scan_left = i->distance_mm;
        } else if (age < CAR_SCAN_TIMEOUT_MS) return;
        state.phase = 11; state.phase_since = i->now;
        o->servo = CAR_SCAN_RIGHT_ANGLE; return;
    }
    if (state.phase == 11U) {
        o->servo = CAR_SCAN_RIGHT_ANGLE;
        if (i->servo_angle == CAR_SCAN_RIGHT_ANGLE && i->sonar_valid && age >= 300U) {
            scan_right = i->distance_mm;
        } else if (age < CAR_SCAN_TIMEOUT_MS) return;
        if (blocked_left) scan_left = 0;
        if (blocked_right) scan_right = 0;
        o->servo = 90;
        if (scan_left <= CAR_OBSTACLE_MM && scan_right <= CAR_OBSTACLE_MM) {
            Control_Run(0, i->now); return; /* No verified free side: stop. */
        }
        state.turn_right = scan_right > scan_left;
        state.phase = 12; state.phase_since = i->now; return;
    }
    if (state.phase == 12U) {
        if (i->servo_angle == 90U && i->distance_valid && age >= 300U) {
            state.phase = 13; state.phase_since = i->now;
        } else {
            if (age >= CAR_SCAN_TIMEOUT_MS) Control_Run(0, i->now);
            return;
        }
    }
    if (state.phase == 13U) {
        if ((state.turn_right && blocked_right) || (!state.turn_right && blocked_left)) {
            Control_Run(0, i->now); return;
        }
        if (i->now - state.phase_since < 350U) {
            motion(o, state.turn_right ? MOVE_RIGHT : MOVE_LEFT, (int32_t)state.speed);
        } else { state.phase = 0; }
        return;
    }
    if (!i->distance_valid || i->servo_angle != 90U) return;
    if (i->distance_mm < CAR_OBSTACLE_MM || blocked_left || blocked_right) {
        scan_left = scan_right = 0;
        state.phase = 10; state.phase_since = i->now;
        o->servo = CAR_SCAN_LEFT_ANGLE; return;
    }
    motion(o, MOVE_FORWARD, (int32_t)state.speed);
}
static void ps2_control(const CarInput *i, CarOutput *o)
{
    int32_t forward, turn, left, right, peak, value;
    uint32_t b = i->ps2_buttons;
    if (!i->ps2_valid || (b & ((1U << 14) | 1U))) {
        Control_Run(0, i->now); return; /* Cross/SELECT or lost receiver. */
    }
    forward = 128 - (int32_t)i->ps2_ly;
    turn = (int32_t)i->ps2_lx - 128;
    if (forward >= -24 && forward <= 24) forward = 0;
    if (turn >= -24 && turn <= 24) turn = 0;
    if (b & 0xf0U) { /* D-pad overrides analog stick. */
        forward = ((b & (1U << 4)) ? 127 : 0) - ((b & (1U << 6)) ? 127 : 0);
        turn = ((b & (1U << 5)) ? 127 : 0) - ((b & (1U << 7)) ? 127 : 0);
    }
    if (!ps2_ready) {
        if (!forward && !turn && !(b & 0xf0U)) ps2_ready = 1;
        return; /* Require neutral once after each explicit start. */
    }
    left = forward + turn; right = forward - turn;
    peak = left < 0 ? -left : left;
    value = right < 0 ? -right : right;
    if (value > peak) peak = value;
    if (peak < 127) peak = 127;
    o->left = left * (int32_t)state.speed / peak;
    o->right = right * (int32_t)state.speed / peak;
}
static void control_step(const CarInput *i, CarOutput *o)
{
    int32_t speed = (int32_t)state.speed;
    o->left = 0; o->right = 0; o->rgb = 0; o->beep = 0;
    o->servo = recenter_pending; recenter_pending = 0;
    if (i->battery_valid) {
        if (i->battery_mv < CAR_LOW_BATTERY_MV) state.battery_alarm = 1;
        else if (i->battery_mv >= CAR_RECOVER_BATTERY_MV) state.battery_alarm = 0;
    }
    /* Audible warning only, independent of the running state. */
    o->beep = state.battery_alarm && (i->now % 1000U < 200U);
    state.fault = !i->clock_ok ? 1U : (!i->battery_valid ? 2U : 0U);
    if (state.fault) {
        if (state.phase >= 10U) o->servo = 90;
        state.running = 0; state.command = MOVE_STOP; state.phase = 0;
        o->rgb = ((i->now / 200U) & 1U) ? 1U : 0U;
        return;
    }
    if (state.mode == MODE_RGB) {
        uint32_t stage = (i->now / 500U) % 6U;
        o->rgb = (stage & 1U) ? 0U : (1U << (stage / 2U));
        return;
    }
    o->rgb = state.running ? 2U : (((i->now / 500U) & 1U) ? 4U : 0U);
    if (!state.running) return;
    switch (state.mode) {
    case MODE_MANUAL:
        /* Keep the last direction until an explicit stop or mode change. */
        motion(o, state.command, speed);
        break;
    case MODE_PS2: ps2_control(i, o); break;
    case MODE_TRACK: track(i, o); break;
    case MODE_ULTRASONIC: case MODE_COMBINED: scan_avoid(i, o); break;
    case MODE_IR_AVOID: avoid(i, o); break;
    case MODE_FOLLOW:
        if (i->distance_valid && !i->obstacle_left && !i->obstacle_right) {
            if (i->distance_mm > 300U && i->distance_mm < 1500U) motion(o, MOVE_FORWARD, speed * 2 / 3);
            else if (i->distance_mm >= 80U && i->distance_mm < 180U) motion(o, MOVE_BACK, speed / 2);
        }
        break;
    case MODE_LIGHT:
        if (i->light_left && i->light_right) motion(o, MOVE_FORWARD, speed);
        else if (i->light_left) motion(o, MOVE_ARC_LEFT, speed);
        else if (i->light_right) motion(o, MOVE_ARC_RIGHT, speed);
        break;
    default: break; /* 测距/显示模式不驱动电机。 */
    }
    if (o->left > 10000) o->left = 10000;
    if (o->left < -10000) o->left = -10000;
    if (o->right > 10000) o->right = 10000;
    if (o->right < -10000) o->right = -10000;
}

void Control_Lights(uint32_t enabled) { state.lights_enabled = enabled != 0; }
void Control_Step(const CarInput *i, CarOutput *o)
{
    control_step(i, o);
    if (!state.lights_enabled) o->rgb = 0;
    if (ir_horn_active) {
        if (i->now - ir_horn_started < 150U) o->beep = 1;
        else ir_horn_active = 0;
    }
}
