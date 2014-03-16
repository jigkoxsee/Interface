#include "stm32f10x_conf.h"
#include "et_stm32f_arm_kit.h" 
#include "et_stm32f_arm_kit_lcd.h" 
#include "stm32f10x_tim.h"
#include <math.h>
static __IO uint32_t TimingDelay;
__IO uint32_t TimeDisplay = 0;

/* Private functions ---------------------------------------------------------*/
void USART_setup(void);
void USART_Pin_setup(int COM, USART_InitTypeDef* USART_InitStruct);
void USART2_IRQHandler(void);
void putsUART2(unsigned int *buffer);
int putChar (int uart, int c);
void GPIO_setup(void);
void RCC_setup(void);
void delay(unsigned long);  
void NVIC_setup(void); 
void EXTI_setup(void);
void ADC_setup(void);
void Systick_setup(void);

//----------ME Varibale----------
int sys_ms=0;
int sys_ms2=0;
int ms_temp=0;
uint16_t temp;
uint16_t temp_list[60];
int hh = 0;
int mm = 0;
int ss = 0;
int empty = 0;
int isHalt = 0;
int countdown = 5;
int isSet=0;
int setSec=0;
int setMin=0;
int setHour=0;
int firstDigit=0;
int secDigit=0;
char msg[10];

//function for display adc value to LCD
void display_adc(void){
	int i;
	uint8_t text[20]; 
	for(i = 0;i <20;i++){
		text[i] = ' ';
	}
	if(temp>=100)
		text[8]='0'+temp/100;
	if(temp%100>=10)
		text[9]='0'+(temp%100)/10;
	text[10]='0'+temp%10;
	text[11]=' ';
	text[12]='C';
	LCD_SetTextColor(Magenta);
	LCD_DisplayStringLine(Line3, text);
}

//function for display clock to LCD
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
	LCD_SetTextColor(Black);
	LCD_DisplayStringLine(Line1, clock);
}

//Display progress bar use ascii 127 and 128
void displayProgress() 
{ 
	uint8_t text[20]; 	
	uint16_t i;
	uint16_t level;
	uint16_t d=7; 
	
	for(i=3;i<17;i++){ 
		text[i] = 128; 
	}
	level = temp/d; 
	for(i=0;i<level;i++){ 
		text[i+3] = 127; 
	} 
	LCD_SetTextColor(Green);
	if(temp>80)
		LCD_SetTextColor(Red);
	LCD_DisplayStringLine(Line5,text); 
}

//halt function 
void halt(void){
	int i;
	uint8_t haltText[20];
	for(i = 0;i < 20;i++){
		haltText[i] = ' ';
	}
	if(temp>80 && isHalt == 0){
		if(countdown >= 0){ 
			haltText[10] = '0'+countdown;
			countdown--;
		}
		if(countdown < 0){// if countdown to 0 set isHalt to 1 and loop infinite
			LCD_SetTextColor(Red);
			LCD_DisplayStringLine(Line8, haltText);
			isHalt = 1;
			while(1);
		}
	}else{// if in 5 sec temperature cooldown countdown reset to 5
		haltText[10] = ' ';
		countdown = 5;
	}
	LCD_SetTextColor(Red);
	LCD_DisplayStringLine(Line8, haltText);
}

//alert function 
void alert_fn(){
	int i;
	uint8_t alertText[20];
	for(i = 0;i < 20;i++){
		alertText[i] = ' ';
	}

	if(temp <= 80){ // if temp <= 80 show nothing
		isHalt = 0;
		for(i = 0;i < 20;i++){
			alertText[i] = ' ';
		}
		LCD_SetTextColor(Red);
		LCD_DisplayStringLine(Line7, alertText);
	}else{// if temp > 80 show red alert text
		putChar(2,7);
		if(empty == 1){
			for(i = 0;i < 11;i++){
				alertText[i] = ' ';
			}
		}else{
			alertText[6] = 'A';
			alertText[7] = 'L';
			alertText[8] = 'E';
			alertText[9] = 'R';
			alertText[10] = 'T';
		}
		LCD_SetTextColor(Red);
		LCD_DisplayStringLine(Line7, alertText);
	}
}

//save temparature to array
void temp_save(void){
	int i;
	if(empty == 0)
		empty =1;
	else
		empty =0;
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); 
	temp =(ADC_GetConversionValue(ADC1)+10)/41;
	alert_fn();
	for(i=59;i>0;i--){
		temp_list[i]=temp_list[i-1];
	}
	temp_list[0]=temp;
	display_adc();
	displayProgress();
}

//Calculate average temperature function
void temp_average(void){
	int i=0;
	int avg=0;
	
	for(i=0;i<60;i++){
		avg+=temp_list[i];
		if(i%6==0)
			putsUART2((unsigned int*)"\n\r");
		
		if(temp_list[i]>=100)
			putChar(2,'0'+temp_list[i]/100);
		
		if(temp_list[i]>=10)
			putChar(2,'0'+(temp_list[i]%100)/10);
		
		putChar(2,'0'+temp_list[i]%10);
		putChar(2,',');
	}
	avg/=60;
	putsUART2((unsigned int*)"Average Temperature = ");
//show average temp.
	if(avg>=100)
		putChar(2,'0'+avg/100);
	if(avg>=10)
		putChar(2,'0'+(avg%100)/10);
	putChar(2,'0'+avg%10);
	putsUART2((unsigned int*)"\n\r");
}





int main()
{
	int i =0;
	//start setup
	RCC_setup(); 
	GPIO_setup();
	USART_setup(); 
	NVIC_setup(); 
	EXTI_setup(); 
	ADC_setup();
	LCD_Setup();
	Systick_setup();
	
	LCD_Clear(Cyan);
	LCD_SetBackColor(Cyan);
	LCD_SetTextColor(Red);
	for(i=0;i<60;i++){
		temp_list[i]=0;			
	}
	
	putsUART2((unsigned int*)"\tWelcome\n\r");
	putsUART2((unsigned int*)"\t\t to\n\r");
	putsUART2((unsigned int*)"Computer Interfacing : Assignment\n\r\n\r");
	putsUART2((unsigned int*)"Press S to enter Clock setting\n\r");

	while(1);
}

/*******************************************************************************
* Function Name  : GPIO_setup
* Description    : Configures the GPIO. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void delay(unsigned long ms){// delay 1 ms per count @ Crystal 8.0 MHz and PLL9x or SYSCLK = 72 MHz
	volatile unsigned long i,j;
	for (i = 0; i < ms; i++ )
		for (j = 0; j < 5525; j++ ); 
}

void USART_setup(void){
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

	//Initial USART2
	USART_Pin_setup(2, &USART_InitStructure);
}

void USART_Pin_setup(int COM, USART_InitTypeDef* USART_InitStruct){
	GPIO_InitTypeDef GPIO_InitStructure;
	if (COM == 1){
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
		/* USART Interupt config */
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
		/* Enable USART */
		USART_Cmd(USART1, ENABLE);
	}else if (COM == 2){
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
		/* USART Interupt config */
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
		/* Enable USART */
		USART_Cmd(USART2, ENABLE);
	}
}

void ADC_setup(void){ 
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

int putChar (int uart, int c){
	if (uart == 1){
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USART1,c);
	}else{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART2,c);
		return c;
	}	
	return c;
}

void putsUART2(unsigned int *buffer){
	char * temp_ptr = (char *) buffer;
	int c;
	while(*temp_ptr != '\0'){    // End of String
		c = *temp_ptr++;
		if (c == '\n')        // \n = CR = Enter
			putChar (2, 0x0D);    // Enter
		else if(c=='\r')        // \r = LF = Line Feed
			putChar (2, 0x0A);     // Line Feed 
		else
			putChar (2, c);     // Character
	}
}

/*******************************************************************************
* Function Name  : RCC_setup
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_setup(void){
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if (HSEStartUpStatus == SUCCESS ){
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
		while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET);
		/* PPL1 configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */
		RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
		RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
		/* Enable PLL */
		RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08);
	}else{ 
		/* If HSE fails to start-up, the application will have wrong clock configuration.
		User can add here some code to deal with this error */ 
		/* Go to infinite loop */
		while (1);
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC ,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
  
void NVIC_setup(void){ 
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //wakeup
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)SysTick_IRQn; // systick
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //usart2
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
} 	

void EXTI_setup(){ 
	EXTI_InitTypeDef EXTI_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line0; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_InitStructure); 
}  	

void EXTI0_IRQHandler(void){  
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		if(isHalt == 0) //if program not halt do the interrupt
			temp_average();
		EXTI_ClearITPendingBit(EXTI_Line0);   // Clear the EXTI line 0 pending bit  
	} 
} 		

void Systick_setup(void){
	//Select source clock between AHB (MAX 72 MHz) and AHB/8 (MAX 72/8 = 9 MHz) by using
	//SysTick_CLKSource_HCLK and SysTick_CLKSource_HCLK_Div8 respectively
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	//Config counter
	if (SysTick_Config(72000-1)){  // 72 MHz ? 72,000 = 1 kHz or 1 ms
		/* Capture error */ 
		putsUART2((unsigned int *)"Systick Config Error");
		while (1);
	}
}

void SysTick_Handler(void){
	ms_temp++;
	if(!isSet)
		sys_ms++; // Increse millisec 1 time when not set the clock
	if(sys_ms%1000 == 0){ //Increase ss every 12 sec
		sys_ms=0;
		ss++;
		display_clock();
		if(ss >=60){ //increase 1 minute
			ss =0;
			mm++;
		}
		if(mm >=60){ //increase 1 hour
			mm =0;
			hh++;
		}
		if(hh>=24){//reset hour to 0
			hh=0;
		}	
	}
	if(ms_temp%500 == 0){//Increase ss every 12 sec
		temp_save();
	}
	sys_ms2++;
	if(sys_ms2%1000 == 0){ //check halt function every second
		halt();
	}
}

void USART2_IRQHandler(void){
	char in;
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET&&isHalt==0){
		in = (USART_ReceiveData(USART2));
		if(!isSet && in == 's'){ //start setting function
			isSet = 1;
			putsUART2((unsigned int*)"Set clock\n\r");
			putsUART2((unsigned int*)"Press h to set Hour\n\r");
			putsUART2((unsigned int*)"Press m to set Minute\n\r");
			putsUART2((unsigned int*)"Press c to set Second\n\r");
			putsUART2((unsigned int*)"Press Enter to exit\n\r");
			putChar(2,7);

		}else if(isSet){
			if(in=='c') { //input c to set second
				setSec=1;
				setMin=0;
				setHour=0;
				secDigit=1;
				putsUART2((unsigned int*)"Set Second\n\r");
				putsUART2((unsigned int*)"Second (SS) = ");
				
			}else if(in=='m') { //input m to set minute
				setSec=0;
				setMin=1;
				setHour=0;
				secDigit=1;
				putsUART2((unsigned int*)"Set Minute\n\r");
				putsUART2((unsigned int*)"Minute (MM) = ");
			}else if(in=='h') { //input h to set hour
				setSec=0;
				setMin=0;
				setHour=1;
				secDigit=1;
				putsUART2((unsigned int*)"Set Hour\n\r");
				putsUART2((unsigned int*)"Hour (HH) = ");

			}else if(setSec){ 
				if(secDigit){ //set second digit before first digit
					if(in>='0' && in<='5'){
						putChar(2,in);
						ss = ss%10+10*(in-'0');
						secDigit=0;
						firstDigit=1; //finish set second digit start set first digit
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}else if(firstDigit){ //first digit
					if(in>='0' && in<='9'){
						putChar(2,in);
						putsUART2((unsigned int*)"\n\r");
						ss = (ss/10)*10+in-'0';
						secDigit=0;
						firstDigit=0;
						setSec=0;
						putsUART2((unsigned int*)"Set Second finish\n\r");
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}
			}else if(setMin){
				if(secDigit){//set second digit before first digit
					if(in>='0' && in<='5'){
						putChar(2,in);
						mm = mm%10+10*(in-'0');
						secDigit=0;
						firstDigit=1;//finish set second digit start set first digit
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}else if(firstDigit){//first digit
					if(in>='0' && in<='9'){
						putChar(2,in);
						putsUART2((unsigned int*)"\n\r");
						mm = (mm/10)*10+in-'0';
						secDigit=0;
						firstDigit=0;
						setMin=0;
						putsUART2((unsigned int*)"Set Minute finish\n\r");
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}
			}else if(setHour){
				if(secDigit){//set second digit before first digit
					if(in>='0' && in<='2'){
						putChar(2,in);
						hh = hh%10+10*(in-'0');
						secDigit=0;
						firstDigit=1;//finish set second digit start set first digit
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}else if(firstDigit){//first digit
					if((in>='0' && in<='9' && (hh/10)<2) || (in>='0' && in<='3' && (hh/10)==2)){
						putChar(2,in);
						putsUART2((unsigned int*)"\n\r");
						hh = (hh/10)*10+in-'0';
						secDigit=0;
						firstDigit=0;
						setHour=0;
						putsUART2((unsigned int*)"Set Hour finish\n\r");
					}else{
						putsUART2((unsigned int*)"Wrong Input\n\r");
						putChar(2,7);
					}
				}
			}else if((in==10 || in==13)){ //input enter to exit setting
				putsUART2((unsigned int*)"Clock setting finish\n\r");
				isSet=0;
			}else{
				putsUART2((unsigned int*)"Press h to set Hour\n\r");
				putsUART2((unsigned int*)"Press m to set Minute\n\r");
				putsUART2((unsigned int*)"Press c to set Second\n\r");
				putsUART2((unsigned int*)"Press Enter to exit\n\r");
			}
		}	
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}
