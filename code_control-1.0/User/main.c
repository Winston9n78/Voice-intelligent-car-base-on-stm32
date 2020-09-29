#include "stm32f10x.h"
#include "usart.h"
#include "LD3320.h"
#include "stm32f10x_tim.h"
#include <stdio.h>

/*************�˿���Ϣ********************
 * ����˵��
 LD3320�ӿ�     STM32F103ZET6�ӿ�
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

/*******************************�Աຯ�����֣��ɷ��ļ�*****************************************************/
void LED_Init(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  //��PB��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//��PE��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	//PB5,PE5��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//�˿�ģʽ����Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void TIM1_ALL_CHANNAL_PWM_Init(u16 arr,u16 psc){ 
	
	//ʹ�ø߼���ʱ�����PWMע�����㣺1.RCCʱ��ʹ��RCC_APB2PeriphClockCmd��ʼ�����ұ���ʹ��pwmģʽTIM_CtrlPWMOutputs(TIM1,ENABLE);
	GPIO_InitTypeDef                   MYPWMstructure;
	TIM_TimeBaseInitTypeDef            MYTIM_PWMstructure;
	TIM_OCInitTypeDef                  MYOCPWMstructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //ʹ�ܶ�ʱ��3��ʱ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//ʹ��GPIOAʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	
	MYPWMstructure.GPIO_Mode=GPIO_Mode_AF_PP; //���츴�����
	MYPWMstructure.GPIO_Pin=GPIO_Pin_9 | GPIO_Pin_11 |GPIO_Pin_13 | GPIO_Pin_14;   //ѡ������
	MYPWMstructure.GPIO_Speed=GPIO_Speed_50MHz; //�ٶ�
	GPIO_Init(GPIOE,&MYPWMstructure);      //��ʼ ��GPIO
		
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //ʹ��AFIOʱ��
//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //������ӳ��
		
	MYTIM_PWMstructure.TIM_Period=arr; //�Զ�װ��ֵ
	MYTIM_PWMstructure.TIM_Prescaler=psc;//Ԥ��Ƶϵ��
	MYTIM_PWMstructure.TIM_ClockDivision=0;//����ʱ�ӷָ�:TDTS = Tck_tim
	MYTIM_PWMstructure.TIM_CounterMode=TIM_CounterMode_Up;//�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM1,&MYTIM_PWMstructure);    //��ʼ����ʱ��
	
	MYOCPWMstructure.TIM_OCMode=TIM_OCMode_PWM1; //����PWMģʽ2
	MYOCPWMstructure.TIM_OCPolarity=TIM_OCPolarity_High; //��Ч��ƽΪ�ߵ�ƽ
	MYOCPWMstructure.TIM_OutputState=TIM_OutputState_Enable; //ʹ��
	
	MYOCPWMstructure.TIM_Pulse =0;
	
	TIM_OC1Init(TIM1, &MYOCPWMstructure);          //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM1, &MYOCPWMstructure);          //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC3Init(TIM1, &MYOCPWMstructure);          //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC4Init(TIM1, &MYOCPWMstructure);          //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���	
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1,ENABLE); //ʹ�ܶ�ʱ��
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

void TIM5_Steer_PWM_Init(void){

	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);	//ʹ��TIM2ʱ�ӣ�GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ�Ӻ͸��ù���ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM2��ͨ��PWM��������˿�PA1   //1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//��ʱ����ʱʱ��T���㹫ʽ��T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(1000*1440/72M)s=0.02s����50hzƵ��
	TIM_TimeBaseStructure.TIM_Period = 1000-1;//�Զ���װ��ֵ��ȡֵ������0x0000~0xFFFF֮��
	TIM_TimeBaseStructure.TIM_Prescaler =1440-1;//Ԥ��Ƶֵ��+1Ϊ��Ƶϵ����ȡֵ������0x0000~0xFFFF֮��							 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ���ģʽ	 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
		
	//  
	TIM_OCInitStructure.TIM_Pulse =100;               //���ô�װ�벶��ȽϼĴ���������ֵ,ȡֵ������0x0000~0xFFFF֮�䣬ռ�ձ�900/3600
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);          //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx  //oc2
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);               //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���                     
	//TIM_Cmd(TIM2, DISABLE);	//��ֹTIM2ʹ��
  TIM_Cmd(TIM5, ENABLE);//ʹ�ܶ�ʱ��	
}

void TIM_IRQ_config(u16 arr,u16 psc){
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��
	
	TIM_DeInit(TIM6);//��TIM5��ʱ����ʼ��λ��λֵ
	TIM_InternalClockConfig(TIM6);//���� TIM5 �ڲ�ʱ��
//��ʱ��TIM5��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //��ʼ��

  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );//ʹ�ܸ����ж�
  TIM_ARRPreloadConfig(TIM6, ENABLE); 
	TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx
//�ж����ȼ�NVIC����
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
			
}

void pwm_out(int left_forward,int left_back,int right_forward,int right_back){

	TIM_SetCompare1(TIM1,left_forward);//e9��ǰpwm4
	TIM_SetCompare3(TIM1,right_back);//e13�Һ�pwm2
	
  TIM_SetCompare2(TIM1,left_back);//e11���pwm3
	TIM_SetCompare4(TIM1,right_forward);//E14��ǰpwm1
	
}

void steer_angle_SET(int16_t a){
	/**���pwm��Χ35-80-125**/
	TIM_SetCompare1(TIM5,a);		
	/****************************************************************************/	
}


void trans(){
	
  start_flag_=start_flag;
	stop_flag_=stop_flag;
	
	//��������һ��
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
	
//��TIM3���в��	

	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
	
  /********д�жϴ������**********************************************/		
		
	/*******************************�������*****************************/	
		
	/****************************************�������**************************/	
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
	
	//���жϱ�־λ
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
	LD3320_main();				//LD3320ִ�к���	
	
	while(1)
	{
		
	}
}
/*********************************************END OF FILE**********************/
