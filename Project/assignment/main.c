#include "stm32f10x_conf.h"
#include "et_stm32f_arm_kit.h" 
#include "et_stm32f_arm_kit_lcd.h" 
#include "stm32f10x_tim.h"
#include <math.h>
static __IO uint32_t TimingDelay;
__IO uint32_t TimeDisplay = 0;

/* Private functions ---------------------------------------------------------*/
void delay_ms(__IO uint32_t nTime);	 		//Delay mS
void TimingDelay_Decrement(void);
void USART_setup(void);
void USART_Pin_setup(int COM, USART_InitTypeDef* USART_InitStruct);
int putChar (int uart, int c);
void putsUART1(unsigned int *buffer);
void putsUART2(unsigned int *buffer);
int getChar (int uart);
void GPIO_setup(void);
void RCC_setup(void);
void delay(unsigned long);  
void	NVIC_setup(void); 
void	EXTI_setup(void);
void displayNumberUSART2(uint16_t n);
void ADC_setup(void);
void display16(uint16_t n);
void ledDisplay (uint16_t n);
void Systick_setup(void);
void TIM1_setup(void);
//----------ME Varibale----------
unsigned long sys_ms=0; // Keep time base from Systick
unsigned long ms=0;
uint16_t adc;
uint16_t adc_list[60];
int hh = 0;
int mm = 0;
int ss = 0;

//----------ME FN----------------
void display_adc(void){
	int i;
	uint8_t text[20]; 
	for(i = 0;i <20;i++){
		text[i] = ' ';
	}
	adc%=1000;
	text[6]='0'+adc/100;
	adc%=100;
	text[7]='0'+adc/10;
	adc%=10;
	text[8]='0'+adc;
	text[9]=' ';
	text[10]='C';	
	LCD_DisplayStringLine(Line3, text);
	
}
void display_clock(void){
	int i;
	uint8_t clock[20];
	for(i = 0;i <20;i++){
		clock[i] = ' ';
	}
	clock[6] = '0'+(hh/10);
	clock[7] = '0'+(hh%10);
	clock[8] = ':';
	clock[9] = '0'+(mm/10);
	clock[10] = '0'+(mm%10);
	clock[11] = ':';
	clock[12] = '0'+(ss/10);
	clock[13] = '0'+(ss%10);
	LCD_DisplayStringLine(Line1, clock);
}

void adc_save(void){
	if(ms%500 == 0){
		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); 
		adc =(ADC_GetConversionValue(ADC1)*100)/4096;		
	}
}

void displayProgress(uint16_t temp)
{
  uint8_t text[20] = "   OOOOOOOOOOOOOO   ";
	int i;
	int level = rintf(temp / 7.14);
	for(i=0;i<level;i++)
	{
	    text[i+3] = 'X';
	}
	LCD_DisplayStringLine(Line5,text);
}



int main()
{

 
	
	int i =0;
	RCC_setup(); 
	GPIO_setup();
	USART_setup(); 
	NVIC_setup(); 
	EXTI_setup(); 
	ADC_setup();
	LCD_Setup();
	TIM1_setup();
	Systick_setup();
	LCD_Clear(White); 
//	LCD_SetBackColor(Cyan); 
	LCD_SetTextColor(Red);


	while(1)     
	{ 
		adc_save();
		display_clock();
		display_adc();
		displayProgress(adc);
		displayNumberUSART2(adc);
	} 
}

/*******************************************************************************
* Function Name  : GPIO_setup
* Description    : Configures the GPIO. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void delay(unsigned long ms)  // delay 1 ms per count @ Crystal 8.0 MHz and PLL9x or SYSCLK = 72 MHz
{
	volatile unsigned long i,j;
	for (i = 0; i < ms; i++ )
		for (j = 0; j < 5525; j++ ); 
	}

void USART_setup(void)
{
	USART_InitTypeDef USART_InitStructure;
	/* USARTx configured as follow:
	- BaudRate = 115200 baud 
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Initial USART1
	USART_Pin_setup(1, &USART_InitStructure);
	//Initial USART2
	USART_Pin_setup(2, &USART_InitStructure);
}

void USART_Pin_setup(int COM, USART_InitTypeDef* USART_InitStruct)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if (COM == 1)
	{
		/* Enable GPIO clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		/* Enable the USART1 Pins Software Remapping */
		GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		/* Configure USART Tx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* Configure USART Rx as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* USART configuration */
		USART_Init(USART1, USART_InitStruct);
		/* Enable USART */
		USART_Cmd(USART1, ENABLE);
	}
	else if (COM == 2)
	{
		/* Enable GPIO clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
		/* Enable the USART2 Pins Software Remapping */
		GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		/* Configure USART Tx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		/* Configure USART Rx as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		/* USART configuration */
		USART_Init(USART2, USART_InitStruct);
		/* Enable USART */
		USART_Cmd(USART2, ENABLE);
	}
}

void ADC_setup(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	ADC_InitTypeDef ADC_InitStructure; 
	
  /* Enable ADC1 clock */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
  /* ADC Channel14 config --------------------------------------------------------*/ 
  /* Relative to STM3210C-EVAL Board   */ 
  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;        //PC4 = ADC 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  /* ADC1 Configuration ------------------------------------------------------*/ 
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
  ADC_InitStructure.ADC_NbrOfChannel = 1; 
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* ADC1 regular channel14 configuration */ 
  /* Sampling Time 13.5 cycle of ADC Clock */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_13Cycles5); 
  
  /* Enable ADC1 */ 
  ADC_Cmd(ADC1, ENABLE); 
  
   ADC_ResetCalibration(ADC1);   // Enable ADC1 reset calibaration register  
   while(ADC_GetResetCalibrationStatus(ADC1)); 
   
   ADC_StartCalibration(ADC1);   // Start ADC1 calibaration 
   
  /* Start ADC1 Software Conversion */ 
   ADC_SoftwareStartConvCmd(ADC1, ENABLE);  
} 


int putChar (int uart, int c) 
{
	if (uart == 1)
	{
			/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{}
			/* Place your implementation of fputc here */
			/* e.g. write a character to the USART */
		USART_SendData(USART1,c);
	} 
	else
	{
				/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
		{}
				/* Place your implementation of fputc here */
				/* e.g. write a character to the USART */
	USART_SendData(USART2,c);
	return c;
	}	
	return c;
}

void putsUART1(unsigned int *buffer)
{
	char * temp_ptr = (char *) buffer;
	int c;
	while(*temp_ptr != '\0')     // End of String
	{
		c = *temp_ptr++;
		if (c == '\n')        // \n = CR = Enter
		{ 
			putChar (1, 0x0D);    // Enter
		}
		else if(c=='\r')      // \r = LF = Line Feed
		{
			putChar (1, 0x0A);     // Line Feed
		}
		else
		{
			putChar (1, c);     // Character
		}
	}
}

void putsUART2(unsigned int *buffer)
{
	char * temp_ptr = (char *) buffer;
	int c;
	while(*temp_ptr != '\0')    // End of String
	{
		c = *temp_ptr++;
		if (c == '\n')        // \n = CR = Enter
		{
			putChar (2, 0x0D);    // Enter
		}
		else if(c=='\r')        // \r = LF = Line Feed
		{
			putChar (2, 0x0A);     // Line Feed 
		}
		else
		{
			putChar (2, c);     // Character
		}
	}
}

int getChar (int uart) 
{
	if (uart == 1)
	{
	/* Loop until the USARTz Receive Data Register is not empty */
		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){}
			return (USART_ReceiveData(USART1));
	}
	else
	{
		/* Loop until the USARTz Receive Data Register is not empty */
		while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET){}
			return (USART_ReceiveData(USART2));
	}
}
/*******************************************************************************
* Function Name  : RCC_setup
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_setup(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if (HSEStartUpStatus == SUCCESS )
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		/***************************************************/
		/* HSE=25MHz, HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz */
		/***************************************************/
		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK2 = HCLK */ 
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/* ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
		/* Configure PLLs */
		/* PPL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
		RCC_PREDIV2Config(RCC_PREDIV2_Div5);
		RCC_PLL2Config(RCC_PLL2Mul_8);
		/* Enable PLL2 */
		RCC_PLL2Cmd(ENABLE);
		/* Wait till PLL2 is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
		{}
		/* PPL1 configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */
	RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
	RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
		/* Enable PLL */
	RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{}
		/* Select PLL as system clock source */
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		/* Wait till PLL is used as system clock source */
while (RCC_GetSYSCLKSource() != 0x08)
{}
}
else
{ 
			/* If HSE fails to start-up, the application will have wrong clock configuration.
		User can add here some code to deal with this error */ 
		/* Go to infinite loop */
	while (1)
	{
	}
}
}

/*******************************************************************************
* Function Name  : GPIO_setup
* Description    : Configures the GPIO. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void GPIO_setup(){
	
	GPIO_InitTypeDef GPIO_InitStructure;   
   // Enable GPIOE clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC ,ENABLE);
	
   // Configure PE8 and PE15 as Output push-pull(connect with LED) 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 
	| GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	 // Enable GPIOA clock
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
  void delay_ms(__IO uint32_t nTime)
  {
  	TimingDelay = nTime;

  	while(TimingDelay != 0)
  	{
  	}
  }

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
  
void NVIC_setup(void) 
{ 
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //temper
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //wakeup
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn; // systick
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
} 	

void EXTI_setup() 
{ 
	EXTI_InitTypeDef EXTI_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line13; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_InitStructure); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line0; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_InitStructure); 
}  	

void EXTI0_IRQHandler(void)  
{  
	static uint16_t count = 0; 
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{ 
  // Toggle LED7 at PE15 pin  
		GPIO_WriteBit(GPIOE, GPIO_Pin_15,(BitAction)((1-GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_15)))); 
		putChar(2, 'W'); 
		count++;   
		
		displayNumberUSART2(count);
		delay(300); 
		
		EXTI_ClearITPendingBit(EXTI_Line0);   // Clear the EXTI line 0 pending bit  
	} 
} 		

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) 
	{ 
  // Toggle LED7 at PE15 pin  
		GPIO_WriteBit(GPIOE, GPIO_Pin_8,(BitAction)((1-GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_8)))); 
		putChar(2, 'T'); 
		delay(300);

	EXTI_ClearITPendingBit(EXTI_Line13);   // Clear the EXTI line 0 pending bit  
} 
}	

void TIM1_setup() 
{ 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	 // Enable TIM1 clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
	 // Configuration TIM1 Interval counter 1 ms 
	 // APB2 / ClockDivision / Prescaler / Period => Interval Time 
	 // 72 MHz / 1 / 72 / 1000 => 1 kHz ???? 1 ms 

	TIM_TimeBaseStructure.TIM_Prescaler = 71 ; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period = 999; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure); 
	 // TIM1 enable Update interrupt 
	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE); 
	 // TIM1 counter enable 
	TIM_Cmd(TIM1,ENABLE); 
} 

void TIM1_UP_IRQHandler(void) 
{ 
	ms++; // Increment millisec 1 time 
	
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update); // Clear TIM1 Update pending bit 
}

void Systick_setup(void){
	//Select source clock between AHB (MAX 72 MHz) and AHB/8 (MAX 72/8 = 9 MHz) by using
	//SysTick_CLKSource_HCLK and SysTick_CLKSource_HCLK_Div8 respectively
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	//Config counter
	if (SysTick_Config(72000-1))  // 72 MHz ? 72,000 = 1 kHz or 1 ms
	{ 
		/* Capture error */ 
		putsUART2((unsigned int *)"Systick Config Error");
		while (1);
	}
}
void SysTick_Handler(void)
{
	sys_ms++; // Increse millisec 1 time
	if(sys_ms%1000 == 0){
		sys_ms=0;
		ss++;
		if(ss >=60){
			ss =0;
			mm++;
		}
		if(mm >=60){
			mm =0;
			hh++;
		}
		if(hh>=24){
			hh=0;
		}
	}
	
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}

void displayNumberUSART2(uint16_t n){
	/* Loop until the end of transmission */
	int c=0,temp = n,mod = 1;
	int i;
	while(temp!=0){
		temp /=10;
		c++;
	}
	for(i =1;i < c;i++){
		mod *= 10;
	}
	while(c !=0){
		putChar(2,n/mod + 0x30);
		n %= mod;
		mod /= 10;
		c--;
	}
	putChar(2,' ');
}
