#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"

#define KEY_CTRL PAin(8)

#define DISTANCE_FOLLOW 1500
#define DISTANCE_DIF  500

void timer2(){
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		PAout(7) = !PAout(7);
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

u8 flag_run = 0;

void key_int(){
		SYSTICK_DelayMs(5);
		if(KEY_CTRL == 0){
			flag_run = !flag_run;
			while(KEY_CTRL == 0);
		}
}

int main(void){
	int LeftSensorValue1 = 0;
	int LeftSensorValue2 = 0;
	int RightSensorValue1 = 0;
	int RightSensorValue2 = 0;
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_ms_Init(TIM2, 500, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 	 
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//初始化与按键相连的引脚
	
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_14, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_10, GPIO_Mode_IPU);//初始化与模块相连的引脚
	
	
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_7, GPIO_Mode_Out_PP);//初始化与模块相连的引脚

	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3);
	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	
	
	car_init();          //小车驱动初始化
	car_set_motor_speed(5000, 5000);
	while(1){  

		//printf("ir is %d, %d, %d, %d \n",LeftSensorValue1, LeftSensorValue2, RightSensorValue1, RightSensorValue2 );

		if(flag_run){
			
				LeftSensorValue1  = PCin(14);
				RightSensorValue1 = PCin(15);

				LeftSensorValue2  = PCin(13);
				RightSensorValue2 = PBin(10);
				
				if (LeftSensorValue1 == 1 && RightSensorValue1 == 1 ) { 
					car_forward();        
				} 
				else if (LeftSensorValue1 == 1 && RightSensorValue1 == 0 ) {
					car_turn_right_place(); 
				}
				else if (LeftSensorValue1 == 0 && RightSensorValue1 == 1) {
					car_turn_left_place();  
				}
				else {
					car_forward(); 
				}
		
		}else{
			car_brake();
		}
	}
}
 

