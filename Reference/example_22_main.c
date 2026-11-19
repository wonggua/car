#include "sysinc.h"


#define KEY_CTRL PAin(8)

#define BEEP PBout(11)
#define LED_RED PBout(0)

u8 flag_run = 0;
u8 flag_power_alarm = 0;

#define MIN_POWER 6.8

void key_int(){
		SYSTICK_DelayMs(5);
		if(KEY_CTRL == 0){
			flag_run = !flag_run;
			while(KEY_CTRL == 0);
		}
}

void power_alarm(){
			if(flag_run && flag_power_alarm){
				BEEP = ~BEEP;
				LED_RED = ~LED_RED;
			}else{
				BEEP = 1;
				LED_RED = 1;
			}
}

void timer2(){
	static int i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //清除TIMx更新中断标志 
		
		  if(++i >= 10) { PCout(13)= !PCout(13); 	i= 0;	}
			power_alarm();
			
	}
}

int main(void){
	u16 adcx;
	float power = 0;
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_us_Init(TIM2, 50000, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口

	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_11, GPIO_Mode_Out_PP);//初始化与蜂鸣器连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_0,  GPIO_Mode_Out_PP);//初始化与蜂鸣器连接的硬件接口

	ADC_init(ADC_CH0);
	
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3);
	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	

	BEEP = 1;
	LED_RED = 1;
	
	while(1){  

		adcx=Get_Adc_Average(ADC_CH0,10);
		printf("adc4 val: %d\r\t",adcx);
		power=(float)3*adcx*(3.3/4096);
		printf("adc val: %f\n",power);
		
		if( power < MIN_POWER){
			flag_power_alarm = 1;
		}else{
			flag_power_alarm = 0;
		}
		
	}
	
}
 

