#include "car_control.h"
#include "car_config.h"
static CarState state;
static uint32_t saved_track_speed, saved_other_speed;
static uint32_t track_recovery_move, track_last_seen;
static uint32_t track_inner_side, track_inner_since, track_search_side;
static void reset_tracking(void)
{
    track_recovery_move = MOVE_STOP;
    track_inner_side = MOVE_STOP;
    track_search_side = MOVE_LEFT;
}

static uint32_t ps2_ready, scan_left, scan_right, recenter_pending;
static uint32_t ps2_start_released, triangle_released, ps2_new_run;
static uint32_t ps2_had_link, ps2_epoch;
static int32_t center_x, center_y;
static uint32_t ir_horn_active, ir_horn_started;

const CarState *Control_State(void) { return &state; }
void Control_Init(void)
{
    CarState empty = {0};
    state = empty;
    reset_tracking();
    state.alarm_enabled = 0;
    triangle_released = ps2_new_run = 0;
    center_x = center_y = 128;
    ps2_start_released = 0;
    state.lights_enabled = 1;
    ir_horn_active = ir_horn_started = 0;
    scan_left = scan_right = recenter_pending = 0;
    ps2_ready = ps2_had_link = 0;
    state.speed = saved_other_speed = CAR_DEFAULT_SPEED;
    saved_track_speed = TRACK_DEFAULT_PWM;
}
void Control_Run(uint32_t enabled, uint32_t now)
{
    if (state.phase >= 10U) recenter_pending = 90;
    state.running = enabled != 0;
    reset_tracking();
    if (enabled || state.mode != MODE_PS2) {
        center_x = center_y = 128;
        ps2_new_run = state.running;
        state.pad_only = 0;
    }
    ps2_ready = ps2_had_link = 0;
    state.command = MOVE_STOP;
    state.phase = 0;
    state.phase_since = now;
}
void Control_SetMode(uint32_t mode, uint32_t now)
{
    if (mode >= MODE_COUNT) return;
    Control_Run(0, now);
    if (state.speed && (mode == MODE_TRACK) != (state.mode == MODE_TRACK))
        state.speed = mode == MODE_TRACK ? saved_track_speed : saved_other_speed;
    state.mode = mode;
    if (mode == MODE_PS2) Control_Run(1, now);
}
void Control_Move(uint32_t command, uint32_t now)
{
    if (command > MOVE_ARC_RIGHT) return;
    if (command == MOVE_STOP) { Control_Run(0, now); return; }
    if (state.mode != MODE_MANUAL) Control_SetMode(MODE_MANUAL, now);
    state.running = 1;
    state.command = command;
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
int Control_Speed(uint32_t speed)
{
    uint32_t maximum = state.mode == MODE_TRACK ? TRACK_MAX_PWM : CAR_MAX_SPEED;
    uint32_t minimum = state.mode == MODE_TRACK ? TRACK_MIN_PWM : CAR_MIN_SPEED;
    if (speed && (speed < minimum || speed > maximum)) return 0;
    if (speed && state.mode == MODE_TRACK) saved_track_speed = speed;
    else if (speed) saved_other_speed = speed;
    state.speed = speed;
    if (!speed) reset_tracking();
    return 1;
}
static void motion(CarOutput *o, uint32_t command, int32_t speed)
{
    switch (command) {
    case MOVE_FORWARD: o->left = speed; o->right = speed; break;
    case MOVE_BACK: o->left = -speed; o->right = -speed; break;
    case MOVE_LEFT: o->left = -speed; o->right = speed; break;
    case MOVE_RIGHT: o->left = speed; o->right = -speed; break;
    case MOVE_ARC_LEFT: o->left = -speed / 2; o->right = speed; break;
    case MOVE_ARC_RIGHT: o->left = speed; o->right = -speed / 2; break;
    default: o->left = 0; o->right = 0; break;
    }
}
static void track_turn(CarOutput *o, uint32_t side, int32_t outer_pwm)
{
    o->left = side == MOVE_RIGHT ? outer_pwm : 0;
    o->right = side == MOVE_LEFT ? outer_pwm : 0;
}

static void recover_line(uint32_t now, CarOutput *o)
{
    uint32_t age = now - track_last_seen;
    uint32_t move = track_recovery_move;
    track_inner_side = MOVE_STOP;
    if (move == MOVE_STOP) return;
    if (age >= TRACK_SEARCH_END_MS) {
        reset_tracking();
        return;
    }
    if (age >= TRACK_SEARCH_REVERSE_MS)
        move = track_search_side == MOVE_LEFT ? MOVE_RIGHT : MOVE_LEFT;
    else if (age >= TRACK_RECOVERY_MS)
        move = track_search_side;
    if (move == MOVE_FORWARD) motion(o, move, TRACK_MIN_PWM);
    else track_turn(o, move, TRACK_TURN_PWM);
}
static void track(const CarInput *i, CarOutput *o)
{
    int32_t error = 0, count = 0, outer_pwm;
    uint32_t mask = i->line & 15U;
    int32_t speed = (int32_t)state.speed;
    /* 从左至右：循迹4/PB10、循迹3/PC15、循迹2/PC13、循迹1/PC14；1 表示检测到黑线。 */
    if (mask & 1U) { error -= 3; ++count; }
    if (mask & 2U) { error -= 1; ++count; }
    if (mask & 4U) { error += 1; ++count; }
    if (mask & 8U) { error += 3; ++count; }
    if (!speed) return;
    if (!count) {
        recover_line(i->now, o);
        return;
    }
    track_last_seen = i->now;
    track_recovery_move = MOVE_FORWARD;
    if (mask == 2U || mask == 4U) {
        uint32_t side = mask == 2U ? MOVE_LEFT : MOVE_RIGHT;
        if (track_inner_side != side) {
            track_inner_side = side;
            track_inner_since = i->now;
        }
        if (i->now - track_inner_since < TRACK_INNER_CONFIRM_MS) {
            motion(o, MOVE_FORWARD, speed);
            return;
        }
        track_search_side = side;
        outer_pwm = speed + speed / 24;
    } else {
        track_inner_side = MOVE_STOP;
        if (!error) {
            motion(o, MOVE_FORWARD, speed);
            return;
        }
        track_search_side = error < 0 ? MOVE_LEFT : MOVE_RIGHT;
        track_recovery_move = track_search_side;
        if (error < 0) error = -error;
        if (error >= 2 * count) {
            track_turn(o, track_search_side, TRACK_TURN_PWM);
            return;
        }
        outer_pwm = speed + speed * error / (8 * count);
    }
    track_turn(o, track_search_side, outer_pwm > TRACK_MAX_PWM ? TRACK_MAX_PWM : outer_pwm);
}
static void avoid(const CarInput *i, CarOutput *o)
{
    int32_t speed = (int32_t)state.speed;
    uint32_t left = i->obstacle_left, right = i->obstacle_right;
    if (state.phase == 1 && i->now - state.phase_since >= 250U) {
        state.phase = 2; state.phase_since = i->now;
    }
    if (state.phase == 2 && i->now - state.phase_since >= 350U) state.phase = 0;
    if (!state.phase && (left || right)) {
        state.phase = 1; state.phase_since = i->now;
        if (left != right) state.turn_right = left;
        else state.turn_right = !state.turn_right;
    }
    if (state.phase == 1) motion(o, MOVE_BACK, speed);
    else if (state.phase == 2) motion(o, state.turn_right ? MOVE_RIGHT : MOVE_LEFT, speed);
    else motion(o, MOVE_FORWARD, speed);
}
static void scan_avoid(const CarInput *i, CarOutput *o)
{
    uint32_t age = i->now - state.phase_since;
    uint32_t blocked_left = state.mode == MODE_COMBINED && i->obstacle_left;
    uint32_t blocked_right = state.mode == MODE_COMBINED && i->obstacle_right;
    int32_t speed = (int32_t)state.speed;
    o->servo = 90;
    if (state.phase == 13U) {
        if ((state.turn_right && blocked_right) || (!state.turn_right && blocked_left))
            state.phase = 0;
        else if (age < 350U) {
            motion(o, state.turn_right ? MOVE_RIGHT : MOVE_LEFT, speed);
            return;
        } else state.phase = 0;
    }
    if (!state.phase) {
        if (i->servo_angle == 90U && i->distance_valid &&
            i->distance_mm >= CAR_OBSTACLE_MM && !blocked_left && !blocked_right) {
            motion(o, MOVE_FORWARD, speed);
            return;
        }
        scan_left = scan_right = 0;
        state.phase = 10;
        state.phase_since = i->now;
        age = 0;
    }
    /* Back away slowly while the servo settles and measures each side. */
    motion(o, MOVE_BACK, speed < CAR_MIN_SPEED ? speed : CAR_MIN_SPEED);
    if (state.phase == 10U) {
        o->servo = CAR_SCAN_LEFT_ANGLE;
        if (i->servo_angle == CAR_SCAN_LEFT_ANGLE && i->sonar_valid && age >= 300U)
            scan_left = i->distance_mm;
        else if (age < CAR_SCAN_TIMEOUT_MS) return;
        state.phase = 11;
        state.phase_since = i->now;
        o->servo = CAR_SCAN_RIGHT_ANGLE;
        return;
    }
    o->servo = CAR_SCAN_RIGHT_ANGLE;
    if (i->servo_angle == CAR_SCAN_RIGHT_ANGLE && i->sonar_valid && age >= 300U)
        scan_right = i->distance_mm;
    else if (age < CAR_SCAN_TIMEOUT_MS) return;
    if (blocked_left != blocked_right) state.turn_right = blocked_left;
    else if (scan_left != scan_right) state.turn_right = scan_right > scan_left;
    else state.turn_right = !state.turn_right;
    state.phase = 13;
    state.phase_since = i->now;
    o->servo = 90;
    motion(o, state.turn_right ? MOVE_RIGHT : MOVE_LEFT, speed);
}
static void ps2_control(const CarInput *i, CarOutput *o)
{
    int32_t forward, turn, left, right, peak, value;
    uint32_t b = i->ps2_buttons;
    if (ps2_had_link && (!i->ps2_valid || i->ps2_epoch != ps2_epoch)) {
        ++state.ps2_timeouts;
    }
    ps2_had_link = i->ps2_valid;
    ps2_epoch = i->ps2_epoch;
    if (!i->ps2_valid) return; /* Wheel outputs default to zero once the last frame expires. */
    if (b & ((1U << 14) | 1U)) {
        Control_Run(0, i->now); return; /* Cross/SELECT explicitly ends the session. */
    }
    forward = center_y - (int32_t)i->ps2_ly;
    turn = (int32_t)i->ps2_lx - center_x;
    if (forward >= -24 && forward <= 24) forward = 0;
    if (turn >= -24 && turn <= 24) turn = 0;
    if (ps2_new_run) {
        state.pad_only = forward != 0 || turn != 0;
        ps2_new_run = 0;
    }
    if (state.pad_only) forward = turn = 0;
    if (b & 0xf0U) { /* D-pad overrides analog stick. */
        forward = ((b & (1U << 4)) ? 127 : 0) - ((b & (1U << 6)) ? 127 : 0);
        turn = ((b & (1U << 5)) ? 127 : 0) - ((b & (1U << 7)) ? 127 : 0);
    }
    if (!ps2_ready) {
        if (!forward && !turn && !(b & 0xf0U))
            ps2_ready = 1;
        return;
    }
    if (forward || turn) state.running = 1;
    /* Increase steering authority on diagonals while retaining gentle curves. */
    if (forward) turn += turn / 2;
    left = forward + turn; right = forward - turn;
    peak = left < 0 ? -left : left;
    value = right < 0 ? -right : right;
    if (value > peak) peak = value;
    if (peak < 127) peak = 127;
    o->left = left * (int32_t)state.speed / peak;
    o->right = right * (int32_t)state.speed / peak;
}
static int32_t wheel_pwm(int32_t request, int32_t minimum, int32_t maximum)
{
    int32_t duty = request < 0 ? -request : request;
    if (!duty) return 0;
    if (duty < minimum) duty = minimum;
    if (duty > maximum) duty = maximum;
    return request < 0 ? -duty : duty;
}

static void control_step(const CarInput *i, CarOutput *o)
{
    int32_t speed = (int32_t)state.speed;
    uint32_t start = i->ps2_buttons & (1U << 3);
    uint32_t start_pressed = i->ps2_valid && start && ps2_start_released;
    /* Observe a released START on this link before accepting a new press. */
    ps2_start_released = i->ps2_valid && !start;
    if (i->ps2_valid && triangle_released && (i->ps2_buttons & (1U << 12)))
        Control_Alarm(!state.alarm_enabled);
    triangle_released = i->ps2_valid && !(i->ps2_buttons & (1U << 12));
    o->left = 0; o->right = 0; o->rgb = 0; o->beep = 0;
    o->servo = recenter_pending; recenter_pending = 0;
    if (i->battery_valid) {
        if (i->battery_mv < CAR_LOW_BATTERY_MV) state.battery_alarm = 1;
        else if (i->battery_mv >= CAR_RECOVER_BATTERY_MV) state.battery_alarm = 0;
    }
    /* Audible warning only, independent of the running state. */
    o->beep = state.alarm_enabled && state.battery_alarm && (i->now % 1000U < 200U);
    state.fault = !i->clock_ok ? 1U : (!i->battery_valid ? 2U : 0U);
    if (state.fault) {
        if (state.phase >= 10U) o->servo = 90;
        state.running = 0; state.command = MOVE_STOP; state.phase = 0;
        ps2_ready = ps2_had_link = 0;
        o->rgb = ((i->now / 200U) & 1U) ? 1U : 0U;
        return;
    }
    if (!state.running && start_pressed && !(i->ps2_buttons & ((1U << 14) | 1U))) {
        /* L1 + START explicitly samples the resting left stick for this run. */
        Control_SetMode(MODE_PS2, i->now);
        if ((i->ps2_buttons & (1U << 10)) && !(i->ps2_buttons & 0xf0U) &&
            i->ps2_lx >= 32 && i->ps2_lx <= 223 && i->ps2_ly >= 32 && i->ps2_ly <= 223) {
            center_x = (int32_t)i->ps2_lx;
            center_y = (int32_t)i->ps2_ly;
        }
    }
    if (state.mode == MODE_RGB) {
        uint32_t stage = (i->now / 500U) % 6U;
        o->rgb = (stage & 1U) ? 0U : (1U << (stage / 2U));
        return;
    }
    o->rgb = state.running ? 2U : (((i->now / 500U) & 1U) ? 4U : 0U);
    /* In PS2 mode, fresh input after neutral can resume from an explicit stop. */
    if (!state.running && state.mode != MODE_PS2) return;
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
            if (i->distance_mm > 300U && i->distance_mm < 1500U) motion(o, MOVE_FORWARD, speed);
            else if (i->distance_mm >= 80U && i->distance_mm < 180U) motion(o, MOVE_BACK, speed);
        }
        break;
    case MODE_LIGHT:
        if (i->light_left && i->light_right) motion(o, MOVE_FORWARD, speed);
        else if (i->light_left) motion(o, MOVE_ARC_LEFT, speed);
        else if (i->light_right) motion(o, MOVE_ARC_RIGHT, speed);
        break;
    default: break; /* 测距/显示模式不驱动电机。 */
    }
    {
        int32_t maximum = state.mode == MODE_TRACK ? TRACK_TURN_PWM : CAR_MAX_SPEED;
        int32_t minimum = state.mode == MODE_TRACK ? TRACK_MIN_PWM : CAR_MIN_SPEED;
        o->left = wheel_pwm(o->left, minimum, maximum);
        o->right = wheel_pwm(o->right, minimum, maximum);
    }
}

void Control_Alarm(uint32_t enabled) { state.alarm_enabled = enabled != 0; }
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
