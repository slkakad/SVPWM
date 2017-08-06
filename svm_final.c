/*Slk_PEC
Programmer : Santosh L kakad , Ankit S basera (SVNIT surat M-TECH power electronics P15EL001 ,P15EL003
Email ID - > kakadsantosh24@gmail.com	  
Project : Space Vector Pulse Width modulation
OUTPUT Pin are PE 8,9,10,11,12,12
Dade TIME is 1us */
#include"stm32f4xx.h"
#include"arm_math.h"
#include<math.h>

void GPIO_config(void);
void TIM_config(void);
void DAC_config(void);
void bound_check(float *var);
uint8_t sec_idn(float angle);
void tim_cal(void);
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
DAC_InitTypeDef  DAC_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

float  PWM_F= 5000; // <- chang for PWM_F
float T_svm;
float theta1=0,theta2=120,theta3=240,Va,Vb,Vc,Val,Vbe,spc_angle,spc_mag; //<-- angle and signal declaration
float  MI=1.0f;
float clock=168000000;
float ARR_val;
uint8_t sig_flag=0,sector;
float T1,T2,T0,T1n,T2n,T0n;
float32_t CMP1, CMP2, CMP3;

//uint16_t sw1                                                                                                           
int main(void)
{
	T_svm= 0.0002;
	ARR_val =(clock/2)/PWM_F;
	GPIO_config();
	TIM_config();
	DAC_config();
		while(1)
		{
			if(sig_flag==1)
			{
				sig_flag=0;
				theta1++;
				bound_check(&theta1);
				theta2++;
				bound_check(&theta2);
				theta3++;
				bound_check(&theta3);
				Va =MI*sinf(theta1*PI/180)/2;
				Vb =MI*sinf(theta3*PI/180)/2;
				Vc =MI*sinf(theta2*PI/180)/2;		
				
				Val =2*(Va-0.5f*(Vb+Vc))/3;
				Vbe = (Vb-Vc)/sqrt(3);
				spc_angle = atan2(Vbe,Val);
				if(spc_angle <0)
				{
					spc_angle = 2*PI + spc_angle;
				}
				spc_mag= sqrt(pow(Val,2)+pow(Vbe,2));
				sector = sec_idn(spc_angle*180/PI);
				tim_cal();
				
				switch(sector)
				{
					  case 1:
										CMP1 = T1 + T2 + T0/2;
										CMP2 = T2 + T0/2;
										CMP3 = T0/2;
										break;
						case 2:
										CMP1 = T_svm - (T2 + T0/2);
										CMP2 = T1 + T2 + T0/2;
										CMP3 = T0/2;
										break;
						case 3:
										CMP1 = T0/2;
										CMP2 = T1 + T2 + T0/2;
										CMP3 = T2 + T0/2;
										break;
						case 4:
										CMP1 = T0/2;
									  CMP3 = T1 + T2 + T0/2;
									  CMP2 = T_svm-(T2 + T0/2);
									  break;
						case 5:								
									 CMP1 = T2 + T0/2;
									 CMP2 = T0/2;
									 CMP3 = T1 + T2 + T0/2;
									 break;
						case 6:
									 CMP1 = T1 + T2 + T0/2;
									 CMP2 = T0/2;
									 CMP3 = T_svm-(T2 + T0/2);
									 break;
				}
				DAC->DHR12R1= CMP1*4095/T_svm;
				DAC->DHR12R2= CMP2*4095/T_svm;
				CMP1 *= ARR_val/T_svm;
				CMP2 *= ARR_val/T_svm;
				CMP3 *= ARR_val/T_svm;
						TIM1->CCR1 =(uint16_t)CMP1;
						TIM1->CCR2 =(uint16_t)CMP2;
						TIM1->CCR3 =(uint16_t)CMP3;
			}
			
			//DAC->DHR12R1= (uint16_t)(spc_angle*180/PI)*4095/360;
      			//DAC->DHR12R2 = (uint16_t)(sector)*4095/6;	
			//DAC->DHR12R1= (uint16_t)T1;
			//DAC->DHR12R2= (uint16_t)T2;
		}
	
}
void GPIO_config()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
 	 GPIO_Init(GPIOE, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
}
void TIM_config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)ARR_val-1;
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
 	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM1, ENABLE);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
 	 TIM_OCInitStructure.TIM_Pulse = T1;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = T2;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = T0;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM1->BDTR  = 0xFF99;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period =4665;
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
 	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	 NVIC_Init(&NVIC_InitStructure);

}
void DAC_config()
{
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
}
void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	sig_flag =1;
	
}
void bound_check(float *var)
{
	if(*var ==360)
	{
		*var = 0;
	}
}
uint8_t sec_idn(float angle)
{
	uint8_t sec_sig;
	sec_sig = (angle/60 )+ 1;
	if(sec_sig == 7)
	{
		sec_sig =6;
	}
	return sec_sig;
}
void tim_cal(void)
{
	uint8_t ac_f =0;
	T1 = sqrt(3)*T_svm*spc_mag*sinf((sector*PI/3)-spc_angle);
	T2 = sqrt(3)*T_svm*spc_mag*sinf(spc_angle-(sector-1)*PI/3);
	T0 = T_svm -T1 -T2;
}

