#include "stm32f10x.h"
#include "usart.h"
#include "LD3320.h"
#include "stm32f10x_tim.h"
#include <stdio.h>

/*************端口信息********************
 * 接线说明
 LD3320接口     STM32F103ZET6接口
 * RST              PB15
 * IRQ              PB12
 * WR/SPIS          PB13
 * CS   	          PA4
 * P2/SDCK          PA5
 * P1/SDO           PA6
 * P0/SDI           PA7
*****************************************/

#define LED2__OFF GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define LED2__ON GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define LED2_REV GPIO_WriteBit(GPIOE, GPIO_Pin_5,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_5))))

#define LED3__OFF GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define LED3__ON GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define LED3__REV GPIO_WriteBit(GPIOB, GPIO_Pin_5,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5))))

int time_count;
uint8 start_flag_,stop_flag_,move_forward_flag_,move_back_flag_,turn_left_falg_;
uint8 count_forward=0,count_back=0,count_turn_left=0;

/*******************************自编函数部分，可分文件*****************************************************/
void LED_Init(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  //打开PB口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//打开PE口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	//PB5,PE5引脚设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//端口模式，此为输出推挽模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//初始化对应的端口
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void TIM1_ALL_CHANNAL_PWM_Init(u16 arr,u16 psc){ 
	
	//使用高级定时器输出PWM注意两点：1.RCC时钟使用RCC_APB2PeriphClockCmd初始化，且必须使能pwm模式TIM_CtrlPWMOutputs(TIM1,ENABLE);
	GPIO_InitTypeDef                   MYPWMstructure;
	TIM_TimeBaseInitTypeDef            MYTIM_PWMstructure;
	TIM_OCInitTypeDef                  MYOCPWMstructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //使能定时器3的时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能GPIOA时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	
	MYPWMstructure.GPIO_Mode=GPIO_Mode_AF_PP; //推挽复用输出
	MYPWMstructure.GPIO_Pin=GPIO_Pin_9 | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;   //选择引脚
	MYPWMstructure.GPIO_Speed=GPIO_Speed_50MHz; //速度
	GPIO_Init(GPIOE,&MYPWMstructure);      //初始 化GPIO
		
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //使能AFIO时钟
//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //开启重映射
		
	MYTIM_PWMstructure.TIM_Period=arr; //自动装载值
	MYTIM_PWMstructure.TIM_Prescaler=psc;//预分频系数
	MYTIM_PWMstructure.TIM_ClockDivision=0;//设置时钟分割:TDTS = Tck_tim
	MYTIM_PWMstructure.TIM_CounterMode=TIM_CounterMode_Up;//配置向上计数模式
	TIM_TimeBaseInit(TIM1,&MYTIM_PWMstructure);    //初始化定时器
	
	MYOCPWMstructure.TIM_OCMode=TIM_OCMode_PWM1; //配置PWM模式2
	MYOCPWMstructure.TIM_OCPolarity=TIM_OCPolarity_High; //有效电平为高电平
	MYOCPWMstructure.TIM_OutputState=TIM_OutputState_Enable; //使能
	
	MYOCPWMstructure.TIM_Pulse =0;
	
	TIM_OC1Init(TIM1, &MYOCPWMstructure);          //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能TIMx在CCR2上的预装载寄存器
	
	TIM_OC2Init(TIM1, &MYOCPWMstructure);          //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能TIMx在CCR2上的预装载寄存器
	
	TIM_OC3Init(TIM1, &MYOCPWMstructure);          //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能TIMx在CCR2上的预装载寄存器
	
	TIM_OC4Init(TIM1, &MYOCPWMstructure);          //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能TIMx在CCR2上的预装载寄存器	
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1,ENABLE); //使能定时器
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

void TIM5_Steer_PWM_Init(void){

	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);	//使能TIM2时钟，GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA时钟和复用功能时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM2二通道PWM波形输出端口PA1   //1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//定时器定时时间T计算公式：T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(1000*1440/72M)s=0.02s，即50hz频率
	TIM_TimeBaseStructure.TIM_Period = 1000-1;//自动重装载值，取值必须在0x0000~0xFFFF之间
	TIM_TimeBaseStructure.TIM_Prescaler =1440-1;//预分频值，+1为分频系数，取值必须在0x0000~0xFFFF之间							 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数模式	 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
		
	//  
	TIM_OCInitStructure.TIM_Pulse =100;               //设置待装入捕获比较寄存器的脉冲值,取值必须在0x0000~0xFFFF之间，占空比900/3600
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);          //根据TIM_OCInitStruct中指定的参数初始化外设TIMx  //oc2
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //使能TIMx在CCR2上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);               //使能TIMx在ARR上的预装载寄存器                     
	//TIM_Cmd(TIM2, DISABLE);	//禁止TIM2使能
  TIM_Cmd(TIM5, ENABLE);//使能定时器	
}

void TIM_IRQ_config(u16 arr,u16 psc){
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	TIM_DeInit(TIM6);//将TIM5定时器初始化位复位值
	TIM_InternalClockConfig(TIM6);//配置 TIM5 内部时钟
//定时器TIM5初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //初始化

  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );//使能更新中断
  TIM_ARRPreloadConfig(TIM6, ENABLE); 
	TIM_Cmd(TIM6, ENABLE);  //使能TIMx
//中断优先级NVIC设置
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
			
}

void pwm_out(int left_forward,int left_back,int right_forward,int right_back){

	TIM_SetCompare1(TIM1,left_forward);//e9左前pwm4
	TIM_SetCompare3(TIM1,right_back);//e13右后pwm2
	
  TIM_SetCompare2(TIM1,left_back);//e11左后pwm3
	TIM_SetCompare4(TIM1,right_forward);//E14右前pwm1
	
}

void steer_angle_SET(int16_t a){
	/**舵机pwm范围35-80-125**/
	TIM_SetCompare1(TIM5,a);		
	/****************************************************************************/	
}


void trans(){
	
  start_flag_=start_flag;
	stop_flag_=stop_flag;
	
	//参数传递一次
	if(move_forward_flag==1&&count_forward==0){
	move_forward_flag_=move_forward_flag;
  count_forward++;
	}
		
	if(move_back_flag==1&&count_back==0){
	move_back_flag_=move_back_flag;
  count_back++;
	}
			
	if(turn_left_falg==1&&count_turn_left==0){
	turn_left_falg_=turn_left_falg;
  count_turn_left++;
	}
	
}


/******************************************************************************************************/

void TIM6_IRQHandler(void){
	
//用TIM3进行测距	

	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
	
  /********写中断处理程序**********************************************/		
		
	/*******************************定义变量*****************************/	
		
	/****************************************处理程序**************************/	
  pwm_out(500,500,500,500);
	trans();	
  //HCSR04_read();
  if(start_flag_==1){
		pwm_out(500,0,500,0);
  }
	if(stop_flag_==1){
		pwm_out(0,0,0,0);
	}
	
	if(move_forward_flag_==1){
		steer_angle_SET(80);
		pwm_out(500,0,500,0);
		time_count++;
		if(time_count==100){
			pwm_out(0,0,0,0);
			time_count=0;
			move_forward_flag_=0;
		}
			
		//////////////
		count_back=0;
		count_turn_left=0;
		//////////////
	}
	
	if(move_back_flag_==1){
	
		steer_angle_SET(80);
		pwm_out(0,500,0,500);
		time_count++;
		if(time_count==100){
			pwm_out(0,0,0,0);
			time_count=0;
			move_back_flag_=0;
		}

		//////////////
		count_forward=0;
		count_turn_left=0;
		//////////////
		
	}
	
	if(turn_left_falg_==1){
	
		steer_angle_SET(40);
		pwm_out(500,0,500,0);
		time_count++;
		if(time_count==100){
			pwm_out(0,0,0,0);
			time_count=0;
			turn_left_falg_=0;
			steer_angle_SET(80);
		}

		//////////////
		count_back=0;
		count_forward=0;
		//////////////
		
	}			
	
	//清中断标志位
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

/******************************************************************************/

int main(void)
{
	USART_init();
	LED_Init();
	TIM5_Steer_PWM_Init();
	TIM1_ALL_CHANNAL_PWM_Init(1000-1,6-1);
  pwm_out(0,0,0,0);	
	steer_angle_SET(80);
	TIM_IRQ_config(1000-1,720-1);
	LD3320_main();				//LD3320执行函数	
	
	while(1)
	{
		
	}
}
/*********************************************END OF FILE**********************/
