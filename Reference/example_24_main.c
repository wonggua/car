#include "sysinc.h"
#include "car_ctrl.h"
#include "ir_remote.h"

#define KEY_CTRL PAin(8)

#define BEEP PBout(11)

#define MIN_POWER 6.8

u8 txbuf[33]={1,7,3,4};
u8 rxbuf[33]={1,9,3,4};

u8 flag_power_alarm = 0;

void power_check(void);
void power_alarm(void);

void ir_car_ctrl(){
		if(ir_key_value < 99){
			printf("ir data is %d\n", ir_key_value );
			
			switch(ir_key_value){
				case 13:  car_forward(); break;
				case 12:  car_back(); break;
				case 14:  car_turn_left_place(); break;
				case 15:  car_turn_right_place(); break;
				case 0 :  car_turn_left(); break;
				case 10:  car_turn_right(); break;
			}
			
			ir_key_value = 99;
		}else{
			car_brake();
		}
}


//定时器2中断函数
void timer2(){
	static u16 i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //清除TIMx更新中断标志 
		
		  if(++i % 10 == 0) { PCout(13)= !PCout(13);	 power_check(); }
			power_alarm();
			
			if(i % 3 == 0) ir_car_ctrl();
			
			if(i >= 50000) { i=0; }
	}
}

//主函数
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_us_Init(TIM2, 50000, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口

	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_11, GPIO_Mode_Out_PP);//初始化与蜂鸣器连接的硬件接口
	
	ADC_init(ADC_CH0);

	BEEP = 1;
	
	IrInit();
	car_init();

	while(1);
	
}

//电量检测函数
void power_check(){
	  u16 adcx;
	  float power = 0;
		adcx=Get_Adc_Average(ADC_CH0,10);
		power=(float)3*adcx*(3.3/4096);
		//printf("adc val: %f\n",power);
		
		if( power+1 < MIN_POWER ){
			flag_power_alarm = 1;
		}else{
			flag_power_alarm = 0;
		}
}


//电量检测报警函数
void power_alarm(){
			static u8 flag_fir = 0;
			if(flag_power_alarm){
				BEEP = ~BEEP;
				flag_fir = 1;
			}else if(flag_fir == 1){
				BEEP = 1;
				flag_fir = 0;
			}
}



