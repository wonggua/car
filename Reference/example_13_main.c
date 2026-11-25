#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"

#define KEY_CTRL PAin(8)

#define DISTANCE_FOLLOW 1500
#define DISTANCE_DIF  500

void timer2(){
	static int time_c = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  if(++time_c >= 10 ) {
				PCout(13)= !PCout(13);
				time_c = 0;
			}
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
	int LeftSensorValue = 0;
	int RightSensorValue = 0;
	int distance = 0;

	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_ms_Init(TIM2, 50, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	 	
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_6, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_4, GPIO_Mode_IPU);//初始化与模块相连的引脚
	
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3);
	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	
	
	hc_sr04_init();       //超声波初始化
	
	car_init();          //小车驱动初始化
	car_set_motor_speed(5000, 5000);
	while(1){  

		SYSTICK_DelayMs(20);
					  distance = get_hc_sr04_value_by_median_filter();
			  printf("%d \n",distance );

		if(flag_run){
				

				LeftSensorValue  = PAin(4);
				RightSensorValue = PAin(6);
			
				
				if (distance > 10000 && LeftSensorValue == 1 && RightSensorValue == 1) { 
					car_set_motor_speed(6000, 6000);
					car_forward();        
				}
				else if ( distance >= 3000 && distance <= 10000 && LeftSensorValue == 1 && RightSensorValue == 0) {
					car_set_motor_speed(4000, 4000);
					car_back();
					SYSTICK_DelayMs(300);
					car_turn_left_place();  
					SYSTICK_DelayMs(300);
				}
				else if ( distance >= 3000 && distance <= 10000 && RightSensorValue == 1 && LeftSensorValue == 0) {
					car_set_motor_speed(4000, 4000);
					car_back();
					SYSTICK_DelayMs(300);
					car_turn_right_place();  
					SYSTICK_DelayMs(300);  
				}else if( distance >= 3000 && distance <= 10000 ){
						car_set_motor_speed(4000, 4000);
						car_forward(); 
				}
				else {
					car_set_motor_speed(4500, 4500);
					car_back();
					SYSTICK_DelayMs(300);
					car_turn_left_place();  
					SYSTICK_DelayMs(300 );
				}
		
		}else{
			car_brake();
		}
	}
}
 

