#include "sysinc.h"
#include "car_ctrl.h"
#include "steer_sg90.h"
#include "car_board.h"
#include "car_config.h"
#include <stdio.h>
#include "car_ps2.h"

/* 4WD_STM32_V1.0: PA4=左避障2, PA6=右避障1; PA5=左寻光2, PB3=右寻光1。 */
static volatile uint32_t milliseconds;
typedef struct { volatile unsigned int read, write; unsigned char data[128]; } RxQueue;
typedef struct { unsigned int read, write; unsigned char data[512]; } TxQueue;
static RxQueue rx[2];
static TxQueue tx[2];
static volatile unsigned int overflow;
static volatile unsigned int sonar_state, sonar_width;
static volatile uint16_t sonar_rise;
static uint32_t sonar_started, sonar_last_trigger, sonar_last_good;
static uint32_t distance_mm, distance_ok, samples[3], sample_index, sample_count;
static unsigned int servo_angle = 90;
static uint32_t servo_changed, beep_until;
static uint32_t battery_mv, battery_valid, adc_last, adc_sum, adc_count;
static uint32_t key_changed, key_pressed;
static unsigned int key_raw = 1, key_stable = 1, key_long, key_event;
static volatile int ir_pending = -1;
static volatile uint32_t ir_bits, ir_last_time, ir_last_accepted;
static volatile uint16_t ir_edge;
static volatile unsigned int ir_count, ir_receiving, ir_last_code = 255;

static PS2Data ps2_data;
static uint32_t ps2_last_poll, ps2_last_good, ps2_seen;
/* Board schematic: DATA PB12, CMD PB13, CLOCK PB15, ATT PB14.
 * Source example uses CPOL=HIGH, first edge, LSB first. GPIO avoids SPI1 conflicts. */
static uint8_t ps2_transfer(uint8_t command)
{
    uint8_t value = 0;
    unsigned int bit;
    for (bit = 0; bit < 8; ++bit) {
        PBout(13) = (command >> bit) & 1U;
        SYSTICK_DelayUs(3);
        PBout(15) = 0;
        SYSTICK_DelayUs(3);
        if (PBin(12)) value |= (uint8_t)(1U << bit);
        PBout(15) = 1;
    }
    SYSTICK_DelayUs(10);
    return value;
}
static void ps2_service(uint32_t now)
{
    uint8_t data[21];
    unsigned int j, length;
    if (now < 500U && !ps2_seen) return; /* Receiver power-up, without blocking boot. */
    if (now - ps2_last_poll < 20U) return;
    ps2_last_poll = now;
    PBout(14) = 0; SYSTICK_DelayUs(10);
    data[0] = ps2_transfer(0x01);
    data[1] = ps2_transfer(0x42);
    data[2] = ps2_transfer(0x00);
    length = 3U + 2U * (data[1] & 15U);
    if (length > sizeof(data)) length = sizeof(data);
    for (j = 3; j < length; ++j) data[j] = ps2_transfer(0x00);
    PBout(14) = 1; PBout(13) = 1;
    if (PS2_Decode(data, length, &ps2_data)) {
        ps2_last_good = milliseconds; ps2_seen = 1;
    }
}
static void tick(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        ++milliseconds;
    }
}
uint32_t Board_Millis(void) { return milliseconds; }
static void receive(unsigned int port, uint16_t byte)
{
    unsigned int next = (rx[port].write + 1U) & 127U;
    if (next == rx[port].read) { overflow = 1; return; }
    rx[port].data[rx[port].write] = (unsigned char)byte;
    rx[port].write = next;
}
static void uart1_rx(uint16_t byte) { receive(0, byte); }
static void uart2_rx(uint16_t byte) { receive(1, byte); }
int Board_GetByte(unsigned int port)
{
    int result;
    if (rx[port].read == rx[port].write) return -1;
    result = rx[port].data[rx[port].read];
    rx[port].read = (rx[port].read + 1U) & 127U;
    return result;
}
int Board_RxOverflow(void)
{
    unsigned int result;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    result = overflow; overflow = 0;
    if (result) { rx[0].read = rx[0].write; rx[1].read = rx[1].write; }
    __set_PRIMASK(mask);
    return (int)result;
}
void Board_Print(unsigned int port, const char *text)
{
    unsigned int count = 0, free_space;
    while (text[count]) ++count;
    free_space = (tx[port].read - tx[port].write - 1U) & 511U;
    if (count > free_space) return; /* 丢弃整条状态，绝不阻塞控制。 */
    while (*text) {
        tx[port].data[tx[port].write] = (unsigned char)*text++;
        tx[port].write = (tx[port].write + 1U) & 511U;
    }
}
static void tx_service(unsigned int port, USART_TypeDef *uart)
{
    if (tx[port].read != tx[port].write && USART_GetFlagStatus(uart, USART_FLAG_TXE) != RESET) {
        USART_SendData(uart, tx[port].data[tx[port].read]);
        tx[port].read = (tx[port].read + 1U) & 511U;
    }
}
/* 保留原 HC-SR04 引脚/计时思路，改为双边沿捕获，主循环不等待回波。 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line12);
        if (PAin(12) && sonar_state == 1) {
            sonar_rise = (uint16_t)TIM3->CNT;
            sonar_state = 2;
        } else if (!PAin(12) && sonar_state == 2) {
            sonar_width = (uint16_t)((uint16_t)TIM3->CNT - sonar_rise);
            sonar_state = 3;
        }
    }
}
/* 实验24同一PA1/NEC码表，改成下降沿时间戳解码，避免中断内等待整帧。 */
void EXTI1_IRQHandler(void)
{
    uint16_t now, gap;
    uint32_t value;
    if (EXTI_GetITStatus(EXTI_Line1) == RESET) return;
    EXTI_ClearITPendingBit(EXTI_Line1);
    now = (uint16_t)TIM3->CNT;
    gap = (uint16_t)(now - ir_edge); ir_edge = now;
    if (milliseconds - ir_last_time > 150U) ir_receiving = 0;
    ir_last_time = milliseconds;
    if (gap >= 12500U && gap <= 14500U) {
        ir_bits = 0; ir_count = 0; ir_receiving = 1;
    } else if (gap >= 10500U && gap <= 12000U && ir_last_code != 255U && milliseconds - ir_last_accepted <= 180U) {
        ir_pending = (int)ir_last_code; ir_last_accepted = milliseconds; ir_receiving = 0;
    } else if (ir_receiving) {
        if (gap >= 1800U && gap <= 2600U) ir_bits |= 1UL << ir_count;
        else if (gap < 850U || gap > 1450U) { ir_receiving = 0; return; }
        if (++ir_count == 32U) {
            value = ir_bits;
            if ((((value >> 16) ^ (value >> 24)) & 255U) == 255U &&
                (((value >> 8) ^ value) & 255U) == 255U) {
                ir_last_code = (value >> 16) & 255U;
                ir_pending = (int)ir_last_code; ir_last_accepted = milliseconds;
            }
            ir_receiving = 0;
        }
    }
}
int Board_IRCommand(void)
{
    int result;
    uint32_t mask = __get_PRIMASK();
    __disable_irq(); result = ir_pending; ir_pending = -1; __set_PRIMASK(mask);
    return result;
}
static void sonar_service(uint32_t now)
{
    if (sonar_state == 3) {
        uint32_t mm = sonar_width * 343U / 2000U; /* 1us tick -> mm */
        sonar_state = 0;
        if (mm >= 20U && mm <= 4000U) {
            uint32_t a, b, c, t;
            samples[sample_index] = mm; sample_index = (sample_index + 1U) % 3U;
            if (sample_count < 3U) ++sample_count;
            a = samples[0]; b = samples[1]; c = samples[2];
            if (a > b) { t = a; a = b; b = t; }
            if (b > c) { t = b; b = c; c = t; }
            if (a > b) b = a;
            distance_mm = sample_count == 3U ? b : mm;
            /* 近障碍立即响应，不让中值滤波推迟停车。 */
            if (mm < distance_mm) distance_mm = mm;
            distance_ok = 1; sonar_last_good = now;
        } else { distance_ok = 0; sample_count = 0; }
    } else if (sonar_state && now - sonar_started >= 30U) {
        sonar_state = 0; distance_ok = 0; sample_count = 0;
    }
    if (!sonar_state && now - sonar_last_trigger >= 65U && now - servo_changed >= 300U) {
        sonar_last_trigger = now;
        if (PAin(12)) { distance_ok = 0; return; }
        EXTI_ClearITPendingBit(EXTI_Line12);
        sonar_started = now; sonar_state = 1;
        PAout(15) = 1; SYSTICK_DelayUs(15); PAout(15) = 0;
    }
}
void Board_Servo(unsigned int angle)
{
    if (angle < 20U || angle > 160U) return;
    if (angle != servo_angle) {
        servo_angle = angle; servo_changed = milliseconds;
        distance_ok = 0; sonar_state = 0; sample_count = 0;
    }
    steer_sg90_run_to_angle(TIM1, PWM_CH4, (u8)angle);
}
void Board_Beep(uint32_t duration) { beep_until = milliseconds + duration; }
int Board_KeyEvent(void) { int e = (int)key_event; key_event = 0; return e; }
static void key_service(uint32_t now)
{
    unsigned int value = PAin(8);
    if (value != key_raw) { key_raw = value; key_changed = now; }
    if (value != key_stable && now - key_changed >= 25U) {
        key_stable = value;
        if (!value) { key_pressed = now; key_long = 0; key_event = 3; }
        else if (!key_long) key_event = 1;
    }
    if (!key_stable && !key_long && now - key_pressed >= 1000U) {
        key_long = 1; key_event = 2;
    }
}
void Board_Service(void)
{
    uint32_t now = milliseconds;
    tx_service(0, USART1); tx_service(1, USART2);
    key_service(now); sonar_service(now); ps2_service(now);
    if (now - adc_last >= 20U) {
        uint16_t sample;
        adc_last = now; sample = Get_Adc(ADC_CH0);
        if (sample > 4095U) { battery_valid = 0; adc_sum = 0; adc_count = 0; }
        else {
            adc_sum += sample;
            if (++adc_count >= 16U) {
                battery_mv = (adc_sum / 16U) * 9900U / 4095U;
                battery_valid = 1; adc_sum = 0; adc_count = 0;
            }
        }
    }
}
void Board_Read(CarInput *i)
{
    i->now = milliseconds;
    /* Physical left to right: connector 4/PB10, 3/PC15, 2/PC13, 1/PC14. */
    i->line = (PBin(10) == TRACK_BLACK_LEVEL ? 1U : 0U) |
              (PCin(15) == TRACK_BLACK_LEVEL ? 2U : 0U) |
              (PCin(13) == TRACK_BLACK_LEVEL ? 4U : 0U) |
              (PCin(14) == TRACK_BLACK_LEVEL ? 8U : 0U);
    i->obstacle_left = PAin(4) == AVOID_ACTIVE_LEVEL;
    i->obstacle_right = PAin(6) == AVOID_ACTIVE_LEVEL;
    i->light_left = PAin(5) == LIGHT_ACTIVE_LEVEL;
    i->light_right = PBin(3) == LIGHT_ACTIVE_LEVEL;
    i->distance_mm = distance_mm;
    i->distance_valid = distance_ok && i->now - sonar_last_good < CAR_SONAR_STALE_MS && servo_angle == 90U;
    i->battery_mv = battery_mv; i->battery_valid = battery_valid;
    i->clock_ok = SystemCoreClock == 72000000U;
    i->sonar_valid = distance_ok && i->now - sonar_last_good < CAR_SONAR_STALE_MS;
    i->servo_angle = servo_angle;
    i->ps2_valid = ps2_seen && i->now - ps2_last_good <= 100U;
    i->ps2_buttons = ps2_data.buttons;
    i->ps2_lx = ps2_data.lx; i->ps2_ly = ps2_data.ly;
}
void Board_Write(const CarOutput *o)
{
    static int32_t old_left, old_right;
    int32_t left = o->left, right = o->right;
    /* 实验 car_ctrl 的 B 路是左轮、A 路是右轮。切换方向前先撤销PWM。 */
    if ((left < 0 && old_left > 0) || (left > 0 && old_left < 0) ||
        (right < 0 && old_right > 0) || (right > 0 && old_right < 0)) {
        car_set_motor_speed(0, 0);
        TIM_GenerateEvent(TIM4, TIM_EventSource_Update); /* CCR预装载立即生效 */
        car_brake(); SYSTICK_DelayUs(20);
    }
    MOTOR_B_L = left > 0; MOTOR_B_R = left < 0;
    MOTOR_A_L = right > 0; MOTOR_A_R = right < 0;
    car_set_motor_speed((u16)(right < 0 ? -right : right), (u16)(left < 0 ? -left : left));
    old_left = left; old_right = right;
    if (o->servo) Board_Servo(o->servo);
    PBout(1) = !(o->rgb & 1U); PBout(0) = !(o->rgb & 2U); PAout(7) = !(o->rgb & 4U);
    PBout(11) = !(o->beep || (int32_t)(beep_until - milliseconds) > 0);
}
void Board_Status(void)
{
    char text[256];
    CarInput i;
    const CarState *s = Control_State();
    Board_Read(&i);
    sprintf(text, "mode=%lu run=%lu speed=%lu mm=%lu valid=%lu mv=%lu bat=%lu fault=%lu line=%lu ir=%lu,%lu light=%lu,%lu servo=%u ps2=%lu pid=%lu keys=%lu lx=%lu ly=%lu rcode=%lu scan=%lu led=%lu\r\n",
        (unsigned long)s->mode, (unsigned long)s->running, (unsigned long)s->speed,
        (unsigned long)i.distance_mm, (unsigned long)i.distance_valid,
        (unsigned long)i.battery_mv, (unsigned long)s->battery_alarm, (unsigned long)s->fault,
        (unsigned long)i.line, (unsigned long)i.obstacle_left, (unsigned long)i.obstacle_right,
        (unsigned long)i.light_left, (unsigned long)i.light_right, servo_angle,
        (unsigned long)i.ps2_valid, (unsigned long)ps2_data.id,
        (unsigned long)i.ps2_buttons, (unsigned long)i.ps2_lx, (unsigned long)i.ps2_ly,
        (unsigned long)ir_last_code, (unsigned long)s->phase, (unsigned long)s->lights_enabled);
    Board_Print(0, text); Board_Print(1, text);
}
void Board_Init(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef irq;
    EXTI_InitTypeDef exti;
    SystemCoreClockUpdate(); NVIC_init(); SYSTICK_DelayInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
                          RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /* 释放PA15/PB3/PB4，保留SWD */
    GPIO_SetBits(GPIOA, GPIO_Pin_7); GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11);
    GPIO_QuickInit(HW_GPIOA, GPIO_Pin_7, GPIO_Mode_Out_PP);
    GPIO_QuickInit(HW_GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11, GPIO_Mode_Out_PP);
    GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_1, GPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOB, GPIO_Pin_3 | GPIO_Pin_10, GPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_Mode_IPU);
    /* PC13不再作指示灯；不启用LSE，PC14/PC15专用于循迹。 */
    GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_QuickInit(HW_GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_Mode_Out_PP);
    GPIO_QuickInit(HW_GPIOB, GPIO_Pin_12, GPIO_Mode_IPU);
    car_init(); car_brake(); car_set_motor_speed(0, 0);
    steer_sg90_init(TIM1, PWM_CH4); Board_Servo(90);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000U - 1U);
    timer.TIM_Period = 999;
    TIM_TimeBaseInit(TIM2, &timer);
    TIMER_CallbackInstall(HW_TIMER2, tick);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    irq.NVIC_IRQChannel = TIM2_IRQn; irq.NVIC_IRQChannelPreemptionPriority = 1;
    irq.NVIC_IRQChannelSubPriority = 0; irq.NVIC_IRQChannelCmd = ENABLE; NVIC_Init(&irq);
    TIM_Cmd(TIM2, ENABLE);
    timer.TIM_Period = 65535; TIM_TimeBaseInit(TIM3, &timer); TIM_Cmd(TIM3, ENABLE);
    GPIO_QuickInit(HW_GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP); PAout(15) = 0;
    GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPD);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
    EXTI_StructInit(&exti); exti.EXTI_Line = EXTI_Line12;
    exti.EXTI_Mode = EXTI_Mode_Interrupt; exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE; EXTI_Init(&exti); EXTI_ClearITPendingBit(EXTI_Line12);
    irq.NVIC_IRQChannel = EXTI15_10_IRQn; irq.NVIC_IRQChannelPreemptionPriority = 0; NVIC_Init(&irq);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    exti.EXTI_Line = EXTI_Line1; exti.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&exti); EXTI_ClearITPendingBit(EXTI_Line1);
    irq.NVIC_IRQChannel = EXTI1_IRQn; irq.NVIC_IRQChannelSubPriority = 1; NVIC_Init(&irq);
    UART_CallbackInstall(HW_UART1, uart1_rx); UART_CallbackInstall(HW_UART2, uart2_rx);
    UART_QuickInit(HW_UART1, CAR_CONSOLE_BAUD, 2, 0, ENABLE);
    UART_QuickInit(HW_UART2, CAR_BLUETOOTH_BAUD, 2, 1, ENABLE);
    ADC_init(ADC_CH0);
}
