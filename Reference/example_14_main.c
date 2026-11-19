#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"
#include "steer_sg90.h"

#define KEY_CTRL PAin(8)

#define DISTANCE_FOLLOW 1500
#define DISTANCE_DIF  500

void timer2(){
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  PCout(13)= !PCout(13);
	  	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

int main(void){
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_us_Init(TIM2, 500000, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	 	
	steer_sg90_init(TIM1, PWM_CH4);
	steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
	
	while(1){  
		steer_sg90_run_to_angle( TIM1, PWM_CH4, 20);
		SYSTICK_DelayMs(5000);

		steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
		SYSTICK_DelayMs(5000);

		steer_sg90_run_to_angle( TIM1, PWM_CH4, 160);
		SYSTICK_DelayMs(5000);
	}
	
}
 

