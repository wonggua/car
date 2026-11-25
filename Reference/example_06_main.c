#include "sysinc.h"
#include "car_ctrl.h"

#define KEY_CTRL PAin(8)

void timer2()
{	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  PCout(13)= !PCout(13);
	  	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

int main(void){
	 
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_ms_Init(TIM2, 500, TIM_IT_Update,2, ENABLE);	
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	 
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//初始化与模块相连的引脚
	
	car_init();
	PCout(13)  =1;
	
	while(1){
		 
		if(KEY_CTRL == 0){
			SYSTICK_DelayMs(5);
			if(KEY_CTRL == 0){
				car_set_motor_speed(3000, 10000);  //设置左右轮不同转速
				car_forward();
				while(KEY_CTRL == 0);
				car_brake();
			}
		}
		
	}
}
 

