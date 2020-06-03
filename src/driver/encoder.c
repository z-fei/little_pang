#include "stm32f10x.h"
#include "encoder.h"
#include "common.h"

void TIM4_Mode_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   	

	//PB6 ch1  A,PB7 ch2 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能TIM4时钟	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOB时钟	
	GPIO_StructInit(&GPIO_InitStructure);//将GPIO_InitStruct中的参数按缺省值输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PB6 PB7浮空输入	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                           

	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 65536-1;  //设定计数器重装值   TIMx_ARR = 359*4
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3时钟预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              
  //TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式1,上升沿都计数               
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
	TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入
	TIM_ICInitStructure.TIM_ICFilter = 6;  //选择输入比较滤波器 
	TIM_ICInit(TIM4, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM3
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM3的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//运行更新中断
	
	TIM4->CNT = 100;//Reset counter

	TIM_Cmd(TIM4, ENABLE);   //启动TIM4定时器

}
		
void TIM2_Mode_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   	

	//PA15 ch1  ,PB3 ch2 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能TIM2时钟	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA时钟	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA15 浮空输入	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOB时钟	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PB3浮空输入	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);          
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 65536-1;  //设定计数器重装值   TIMx_ARR = 359*4
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3时钟预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  	
  //TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式1,上升沿都计数               
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
	TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入
	TIM_ICInitStructure.TIM_ICFilter = 6;  //选择输入比较滤波器 
	TIM_ICInit(TIM2, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM3
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM3的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//运行更新中断
	
	TIM2->CNT = 100;//Reset counter

	TIM_Cmd(TIM2, ENABLE);   //启动TIM3定时器

}
void Encoders_Init(void)
{
	TIM2_Mode_Config();
  TIM4_Mode_Config();
}
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		 case 0:  Encoder_TIM= TIM2->CNT-100; TIM2->CNT=100;  break;	//CNT表示计数器
		 case 1:  Encoder_TIM= TIM4->CNT-100; TIM4->CNT=100;  break;	
		 default: Encoder_TIM= 0;                                 break;
	 }
		return Encoder_TIM;
}
