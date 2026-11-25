#include "sysinc.h"

#define KEY_CTRL PAin(8)

void car_init(){
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP); 
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_5, GPIO_Mode_Out_PP);  
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_6, GPIO_Mode_Out_PP); 
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_7, GPIO_Mode_Out_PP); 
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_8, GPIO_Mode_Out_PP); 
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_9, GPIO_Mode_Out_PP); 

	PBout(4) = 0;
	PBout(5) = 0;
	PBout(6) = 0;
	PBout(7) = 0;
	PBout(8) = 0;
	PBout(9) = 0;
}

void car_forward(){
	PBout(6) = 1;
	PBout(5) = 1;
		PBout(4) = 0;

	PBout(7) = 1;
	PBout(8) = 1;
	PBout(9) = 0;
}

void car_brake(){
		PBout(4) = 0;

	PBout(5) = 0;
	PBout(6) = 0;
	PBout(7) = 0;
	PBout(8) = 0;
	PBout(9) = 0;
}

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
				car_forward();
				while(KEY_CTRL == 0);
				car_brake();
			}
		}
		
	}
}
 

