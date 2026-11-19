#include "sysinc.h"
/* car integration: retain the example ADC API and PA0 divider conversion.
 * Replace CR1 |= ~(1<<8), which sets reserved/mode bits, with SPL setup.
 * All waits are bounded; 0xffff means invalid, never a valid voltage.
 */
static u8 adc_ready;
void ADC_init(u8 channel)
{
    ADC_InitTypeDef init;
    u32 timeout;
    adc_ready = 0;
    if (channel != ADC_CH0) return;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    GPIO_QuickInit(HW_GPIOA, GPIO_Pin_0, GPIO_Mode_AIN);
    ADC_DeInit(ADC1); ADC_StructInit(&init);
    init.ADC_Mode = ADC_Mode_Independent;
    init.ADC_ScanConvMode = DISABLE;
    init.ADC_ContinuousConvMode = DISABLE;
    init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    init.ADC_DataAlign = ADC_DataAlign_Right;
    init.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &init);
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE); SYSTICK_DelayUs(10);
    ADC_ResetCalibration(ADC1);
    timeout = 100000;
    while (ADC_GetResetCalibrationStatus(ADC1)) if (!--timeout) return;
    ADC_StartCalibration(ADC1); timeout = 100000;
    while (ADC_GetCalibrationStatus(ADC1)) if (!--timeout) return;
    adc_ready = 1;
}
u16 Get_Adc(u8 channel)
{
    u32 timeout = 100000;
    if (!adc_ready || channel != ADC_CH0) return 0xffff;
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
        if (!--timeout) return 0xffff;
    return ADC_GetConversionValue(ADC1);
}
u16 Get_Adc_Average(u8 channel, u8 times)
{
    u32 sum = 0; u8 i; u16 value;
    if (!times) return 0xffff;
    for (i = 0; i < times; ++i) {
        value = Get_Adc(channel); if (value == 0xffff) return value;
        sum += value;
    }
    return (u16)(sum / times);
}
