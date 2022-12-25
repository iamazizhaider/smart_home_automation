/******************************************************************************
	Course: Microprocessor System (Lab)
	
	Project name: CEP (Complex Engineering Problem) 
	
	Description: A Smart Home Model is being designed by interfacing HC-05 Bluetooth module with TIVA C TM4C123GH6PM Launchpad. 
	An Android phone will be used as the transmitter to send different commands that will be received by the microcontroller via 
	UART communication. These commands will be used to turn on and off different device connected with the launchpad. We have used 
	four DC loads (2 fans and 2 LED lights) operating on a minimum of 7-9V. These loads are powered with L298N Motor Driver which 
	is electrically isolated with the microcontroller through the PC817 Optocoupler. To control the speed of the fan, LM35 Precision 
	Centigrade Temperature Sensor is used. When the temperature rises, the speed of the fan increases which is achieved by increasing
	the duty cycle of the input provided. While LDR Sensor Module adjusts the brightness of the room according to the intensity of 
	the light. Common Anode Seven Segment Display is used to display the temperature of the room as measured by the LM35 sensor. The 
	duty cycle of each load is displayed on the UART terminal along with the intensity of light and temperature as measured by LDR 
	Sensor Module and LM35 Precision Centigrade Temperature Sensors respectively.
	
	Team Members:  
	Aziz Haider			2020-EE-172C
	Arooj Fatima		2020-EE-152C
	Subhan Mansoor 	2020-EE-175C
******************************************************************************/


void SystemInit (void);

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//------------------------------------------------------------------------

# define SYSCTL_RCGC_GPIO_R 	*(( volatile unsigned long *) 0x400FE608 )

//GPIO Ports Base Addresses
#define PORT_A									0x40004000
#define PORT_B									0x40005000
#define PORT_C									0x40006000
#define PORT_D									0x40007000
#define PORT_E									0x40024000
#define PORT_F									0x40025000	

// GPIO Port E analog function configuration
#define GPIO_PORTE_DEN_R 				(*((volatile unsigned long*)(PORT_E + 0x51C)))
#define GPIO_PORTE_AMSEL_R 			(*((volatile unsigned long*)(PORT_E + 0x528)))

// GPIO Port C analog function configuration
#define GPIO_PORTC_AFSEL_R 			(*((volatile unsigned long*)(PORT_C + 0x420)))
#define	GPIO_PORTC_PCTL_R 			(*((volatile unsigned long*)(PORT_C + 0x52C)))
#define GPIO_PORTC_DEN_R 				(*((volatile unsigned long*)(PORT_C + 0x51C)))
#define GPIO_PORTC_AMSEL_R 			(*((volatile unsigned long*)(PORT_C + 0x528)))

/* GPIO registers for Port A*/
#define GPIO_PORTA_AFSEL_R 			(*((volatile unsigned long*)(PORT_A + 0x420)))
#define	GPIO_PORTA_AMSEL_R 			(*((volatile unsigned long*)(PORT_A + 0x528)))
#define	GPIO_PORTA_PCTL_R 			(*((volatile unsigned long*)(PORT_A + 0x52C)))
#define	GPIO_PORTA_DEN_R 				(*((volatile unsigned long*)(PORT_A + 0x51C)))
#define	GPIO_PORTA_DATA_R				(*((volatile unsigned long*)(PORT_A + 0x3FC)))
#define	GPIO_PORTA_DIR_R				(*((volatile unsigned long*)(PORT_A + 0x400)))

/* Register Definitions for Port B */
#define	GPIO_PORTB_DATA_R				(*((volatile unsigned long*)(PORT_B + 0x3FC)))
#define	GPIO_PORTB_DIR_R				(*((volatile unsigned long*)(PORT_B + 0x400)))
#define	GPIO_PORTB_DEN_R				(*((volatile unsigned long*)(PORT_B + 0x51C)))

/* GPIO registers for Port F*/
#define 	GPIO_PORTF_AFSEL_R		(*((volatile unsigned long*)(PORT_F + 0x420)))	//Setting Alternate Functionality
#define 	GPIO_PORTF_PCTL_R			(*((volatile unsigned long*)(PORT_F + 0x52C)))	//To select what alternate functionality to choose
#define 	GPIO_PORTF_DEN_R   		(*((volatile unsigned long*)(PORT_F + 0x51C)))
#define 	GPIO_PORTF_DIR_R   		(*((volatile unsigned long*)(PORT_F + 0x400)))
#define 	GPIO_PORTF_LOCK_R 		(*((volatile unsigned long*)(PORT_F + 0x520)))
#define 	GPIO_PORTF_CR_R 			(*((volatile unsigned long*)(PORT_F + 0x524)))

//------------------------------------------------------------------------

# define SYSCTL_RCGC_ADC_R 		*(( volatile unsigned long *) 0x400FE638 )

// ADC Module 0 base address
# define ADC0										0x40038000

// ADC Module Configuration Registers
# define ADC0_ACTIVE_SS_R 			(*( volatile long *) ( ADC0 + 0x000 ))
# define ADC0_INT_MASK_R 				(*( volatile long *) ( ADC0 + 0x008 ))
# define ADC0_TRIGGER_MUX_R 		(*( volatile long *) ( ADC0 + 0x014 ))
# define ADC0_PROC_INIT_SS_R 		(*( volatile long *) ( ADC0 + 0x028 ))
# define ADC0_PERI_CONFIG_R 		(*( volatile long *) ( ADC0 + 0xFC4 ))
# define ADC0_INT_STATUS_CLR_R 	(*( volatile long *) ( ADC0 + 0x00C ))
# define ADC0_SAMPLE_AVE_R 			(*( volatile long *) ( ADC0 + 0x030 ))

// ADC Sample Sequencer 3 configuration registers
# define ADC0_SS3_IN_MUX_R 			(*( volatile long *) ( ADC0 + 0x0A0 ))
# define ADC0_SS3_CONTROL_R 		(*( volatile long *) ( ADC0 + 0x0A4 ))
# define ADC0_SS3_FIFO_DATA_R 	(*( volatile long *) ( ADC0 + 0x0A8 ))

// ADC0 sequencer 2 configuration registers
# define ADC0_SS2_IN_MUX_R 			(*(volatile unsigned long*)( ADC0 + 0x080 ))
# define ADC0_SS2_CONTROL_R 		(*(volatile unsigned long*)( ADC0 + 0x084 ))
# define ADC0_SS2_FIFO_DATA_R 	(*(volatile unsigned long*)( ADC0 + 0x088 ))

//Interrupt Configuration for ADC Module
# define NVIC_EN0_R 						(*(( volatile unsigned long *) 0xE000E100 ))
//IRQ = 16 for ADC0 Sample Sequencer 2 
# define NVIC_EN0_INT16 				0x00010000 
//IRQ = 17 for ADC0 Sample Sequencer 3
# define NVIC_EN0_INT17 				0x00020000 

//------------------------------------------------------------------------

//Clock for UART
# define SYSCTL_RCGC_UART_R 		(*((volatile unsigned long*)0x400FE618))

#define UART_0									0x4000C000
#define UART_1									0x4000D000
#define UART_2									0x4000E000
#define UART_3									0x4000F000
#define UART_4									0x40010000
#define UART_5									0x40011000
#define UART_6									0x40012000	
#define UART_7									0x40013000	
	
//Register definitions for UART module
#define UART_0_DATA_R 						(*((volatile unsigned long*)(UART_0 + 0x000)))
#define UART_0_RX_STATUS_R 				(*((volatile unsigned long*)(UART_0 + 0x004)))
#define UART_0_ERR_CLEAR_R 				(*((volatile unsigned long*)(UART_0 + 0x004)))
#define UART_0_FLAG_R 						(*((volatile unsigned long*)(UART_0 + 0x018)))
#define UART_0_BAUD_INT_R 				(*((volatile unsigned long*)(UART_0 + 0x024)))
#define UART_0_BAUD_FRAC_R 				(*((volatile unsigned long*)(UART_0 + 0x028)))
#define UART_0_LINE_CONTROL_R 		(*((volatile unsigned long*)(UART_0 + 0x02C)))
#define UART_0_CONTROL_R 					(*((volatile unsigned long*)(UART_0 + 0x030)))

//Register definitions for UART module
#define UART_4_DATA_R 						(*((volatile unsigned long*)(UART_4 + 0x000)))
#define UART_4_RX_STATUS_R 				(*((volatile unsigned long*)(UART_4 + 0x004)))
#define UART_4_ERR_CLEAR_R 				(*((volatile unsigned long*)(UART_4 + 0x004)))
#define UART_4_FLAG_R 						(*((volatile unsigned long*)(UART_4 + 0x018)))
#define UART_4_BAUD_INT_R 				(*((volatile unsigned long*)(UART_4 + 0x024)))
#define UART_4_BAUD_FRAC_R 				(*((volatile unsigned long*)(UART_4 + 0x028)))
#define UART_4_LINE_CONTROL_R 		(*((volatile unsigned long*)(UART_4 + 0x02C)))
#define UART_4_CONTROL_R 					(*((volatile unsigned long*)(UART_4 + 0x030)))
#define UART_4_ICR_R 							(*((volatile unsigned long*)(UART_4 + 0x044)))
#define UART_4_IM_R 							(*((volatile unsigned long*)(UART_4 + 0x038)))
#define UART_4_IFLS_R 						(*((volatile unsigned long*)(UART_4 + 0x034)))

//Interrupt Configuration for UART4 Module
# define NVIC_EN1_R 						(*(( volatile unsigned long *) 0xE000E104 ))
//IRQ = 60 for UART4 
# define NVIC_EN1_INT28 				0x10000000 

#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_0_CTL_UARTEN         0x00000001  // UART Enable
#define UART_4_CTL_UARTEN       	0x301  // UART Enable
#define CR											13
#define BS											8
#define LF											10

//------------------------------------------------------------------------

//Peripheral clock enabling for timer
//page 338 of Data Sheet
#define		SYSCTL_RCGCTIMER_R			(*((volatile unsigned long*)0x400FE604))	

//page 726 of Data Sheet
#define 	CFG											0x000
#define 	MODE_A									0x004
#define 	MODE_B									0x008
#define 	CTL											0x00C
#define 	IL_A										0x028
#define 	IL_B										0x02C
#define 	MATCH_A									0x030
#define 	MATCH_B									0x034

#define 	TIMER0									0x40030000
#define 	TIMER1									0x40031000

//Registers for Timer 0
#define		TIMER_0_CFG_R						(*((volatile unsigned long*)(TIMER0 + CFG)))	
#define		TIMER_0_CTL_R						(*((volatile unsigned long*)(TIMER0 + CTL)))	
#define		TIMER_0A_MODE_R					(*((volatile unsigned long*)(TIMER0 + MODE_A)))
#define		TIMER_0B_MODE_R					(*((volatile unsigned long*)(TIMER0 + MODE_B)))
#define		TIMER_0A_ILR_R					(*((volatile unsigned long*)(TIMER0 + IL_A)))	
#define		TIMER_0B_ILR_R					(*((volatile unsigned long*)(TIMER0 + IL_B)))	
#define		TIMER_0A_MATCH_R				(*((volatile unsigned long*)(TIMER0 + MATCH_A)))
#define		TIMER_0B_MATCH_R				(*((volatile unsigned long*)(TIMER0 + MATCH_B)))

//Registers for Timer 1
#define		TIMER_1_CFG_R						(*((volatile unsigned long*)(TIMER1 + CFG)))	
#define		TIMER_1_CTL_R						(*((volatile unsigned long*)(TIMER1 + CTL)))	
#define		TIMER_1A_MODE_R					(*((volatile unsigned long*)(TIMER1 + MODE_A)))
#define		TIMER_1B_MODE_R					(*((volatile unsigned long*)(TIMER1 + MODE_B)))
#define		TIMER_1A_ILR_R					(*((volatile unsigned long*)(TIMER1 + IL_A)))	
#define		TIMER_1B_ILR_R					(*((volatile unsigned long*)(TIMER1 + IL_B)))	
#define		TIMER_1A_MATCH_R				(*((volatile unsigned long*)(TIMER1 + MATCH_A)))
#define		TIMER_1B_MATCH_R				(*((volatile unsigned long*)(TIMER1 + MATCH_B)))
#define 	Reload									16000

//------------------------------------------------------------------------

// Function headers
void GPIO_Init ( void );
void Timer_0_Init(void);
void Timer_1_Init(void);
void UART_0_Init ( void );
char UART_0_InChar(void);
void UART_0_InString(char *bufPt, uint16_t max);
void UART_0_OutChar(char data);
void UART_0_OutString(char *pt);
void OutCRLF(void);
void UART_4_Init ( void );
char UART_4_InChar(void);
void UART_4_InString(char *bufPt, uint16_t max);
void UART_4_OutChar(char data);
void UART_4_OutString(char *pt);
void UART4_Handler ( void );
void Seven_Segment ( void );
void ADC_0_Init ( void );
void LM35_Sample ( void );
void LDR_Sample ( void );
void ADC0Seq2_Handler (void);
void ADC0Seq3_Handler (void);
void Delay ( volatile unsigned int delay );
void delay ( volatile unsigned int delay );
void App_Display ( void );
void Set_MatchValue1(void);
void Set_MatchValue2(void);
int DutyCycle (int match);

// Global Variables 
int Temperature = 0;
int Intensity = 0;
unsigned char Lookup_7Seg_Disp [11] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90}; //Cathode --> Negative Anode --> Positive 
const char seg_select[3] = {0xFB,0xF7}; //PA2 & PA3	
int match_val1, match_val2 = 0;
int Load_A, Load_B, Load_C, Load_D = 0;	//These are flags to determine whether the load is on or off

//------------------------------------------------------------------------

int main ( void )
{
	char intro[20];
	char mesg[20];
	
	//Initialize all the modules
	ADC_0_Init() ; 
	UART_0_Init();
	UART_4_Init();	
	GPIO_Init();
	Timer_0_Init();
	Timer_1_Init();
	
	//Print a welcome message on UART0 (Putty Terminal)
	OutCRLF();	//Go to the beginning of the next line
	sprintf(intro,"Welcome to Smart Home Automation System\nDesigned by Arooj Fatima, Aziz Haider, and Subhan Mansoor\n");
	UART_0_OutString(intro);
	
	while (1) 
	{	
		LM35_Sample () ;		// Start sampling for LM35 Temperature Sensor
		delay (500) ;		
		
		Set_MatchValue1();	// Update match_val1 according to the new value of temperature 
		
		LDR_Sample () ; 		// Start sampling for LDR Sensor
		delay (500) ;		
		
		Set_MatchValue2();	// Update match_val1 according to the new value of intensity of light  
		
		if (Load_A == 1) TIMER_0A_MATCH_R = match_val1;
		if (Load_B == 1) TIMER_0B_MATCH_R = match_val1;
		if (Load_C == 1) TIMER_1A_MATCH_R = match_val2;
		if (Load_D == 1) TIMER_1B_MATCH_R = match_val2;

		//Use \33[2K to clear the current line written
		sprintf(mesg,"\33[2KTemperature: %d ; Intensity of Light: %d ; Duty Cycle of Load A: %d%% ; Duty Cycle of Load B: %d%% ; Duty Cycle of Load C: %d%% ; Duty Cycle of Load D: %d%%\r",Temperature, Intensity, DutyCycle(TIMER_0A_MATCH_R), DutyCycle(TIMER_0B_MATCH_R), DutyCycle(TIMER_1A_MATCH_R), DutyCycle(TIMER_1B_MATCH_R));
		UART_0_OutString(mesg);
	}
}


//------------------------------------------------------------------------

void GPIO_Init ( void )
{
	SYSCTL_RCGC_GPIO_R |= 0x23;
	/* Port B direction and digital enable*/
	GPIO_PORTB_DIR_R |= 0x7F;
	GPIO_PORTB_DEN_R |= 0x7F;
	
	/* Port A direction and digital enable*/
	GPIO_PORTA_DIR_R |= 0x0C;
	GPIO_PORTA_DEN_R |= 0x0C;

	//PORTS FOR TIMER IN PWM MODE
	//Unlock Port PF0
	GPIO_PORTF_LOCK_R = 0x4C4F434B;	//Page 684 of datasheet
	GPIO_PORTF_CR_R = 0x01;
	GPIO_PORTF_DEN_R |= 0x0F;
	GPIO_PORTF_AFSEL_R |= 0x0F;
	GPIO_PORTF_PCTL_R |= 0x00007777;  
	GPIO_PORTF_DIR_R |= 0x0F;	//Direction set as output 	
}

void UART_0_Init(void){
  SYSCTL_RCGC_UART_R |= 0x01;            		// activate UART0
  SYSCTL_RCGC_GPIO_R |= 0x01;			            		// activate port A
 
  UART_0_CONTROL_R &= ~UART_0_CTL_UARTEN;      		// disable UART
  UART_0_BAUD_INT_R = 8;                    		// IBRD = int(16,000,000 / (16 * 115,200)) = int(8.6805)
  UART_0_BAUD_FRAC_R = 44;                     	// FBRD = int(0.6805 * 64 + 0.5) = 44
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART_0_LINE_CONTROL_R = 0x70;
  UART_0_CONTROL_R |= UART_0_CTL_UARTEN;       		// enable UART
  
	GPIO_PORTA_AFSEL_R |= 0x03;           			// enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             			// enable digital I/O on PA1-0
																					// configure PA1-0 as UART
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}

void UART_0_OutChar(char data){
  while((UART_0_FLAG_R & UART_FR_TXFF) != 0);
  UART_0_DATA_R = data;
}

void UART_0_OutString(char *pt){
  while(*pt){
    UART_0_OutChar(*pt);
    pt++;
  }
}







char UART_0_InChar(void){
  while((UART_0_FLAG_R & UART_FR_RXFE) != 0);
  return((char)(UART_0_DATA_R & 0xFF));
}

void UART_0_InString(char *bufPt, uint16_t max) {
int length=0;
char character;
  character = UART_0_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART_0_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART_0_OutChar(character);
    }
    character = UART_0_InChar();
  }
  *bufPt = 0;
}

void OutCRLF(void){
  UART_0_OutChar(CR);
  UART_0_OutChar(LF);
}

void UART_4_Init(void){
  SYSCTL_RCGC_UART_R |= 0x10;            		// activate UART6
  SYSCTL_RCGC_GPIO_R |= 0x04;			            		// activate port C
	
  UART_4_CONTROL_R &= ~UART_4_CTL_UARTEN;      		// disable UART
  UART_4_BAUD_INT_R = 8;                    		// IBRD = int(16,000,000 / (16 * 115,200)) = int(8.6805)
  UART_4_BAUD_FRAC_R = 44;                     	// FBRD = int(0.6805 * 64 + 0.5) = 44
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART_4_LINE_CONTROL_R = 0x70;
  UART_4_CONTROL_R |= UART_4_CTL_UARTEN;       		// enable UART
  
	GPIO_PORTC_AFSEL_R |= 0x30;           			// enable alt function on PC4 & PC5 
  GPIO_PORTC_DEN_R |= 0x30;             			// enable digital I/O on PC4 & PC5 
																					
  GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0xFF00FFFF) + 0x00110000;   // configure PC4 & PC5 as UART
  GPIO_PORTC_AMSEL_R &= ~(0x30);          // disable analog functionality on PC4 & PC5 

	UART_4_ICR_R &= ~(0x010);
	UART_4_IM_R	= 0x0010;	//Setting RXIM
	NVIC_EN1_R = NVIC_EN1_INT28;
	UART_4_IFLS_R = 0x00;
}

void UART_4_OutChar(char data){
  while((UART_4_FLAG_R & UART_FR_TXFF) != 0);
  UART_4_DATA_R = data;
}

void UART_4_OutString(char *pt){
  while(*pt){
    UART_4_OutChar(*pt);
    pt++;
  }
}

char UART_4_InChar(void){
  while((UART_4_FLAG_R & UART_FR_RXFE) != 0);
  return((char)(UART_4_DATA_R & 0xFF));
}

void UART_4_InString(char *bufPt, uint16_t max) {
	int length=0;
	char character;
  character = UART_4_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART_4_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART_4_OutChar(character);
    }
    character = UART_4_InChar();
  }
  *bufPt = 0;
}

void UART4_Handler ( void )
{
	char c;
	
	UART_4_ICR_R &= ~(0x010);
	
	c = UART_4_DATA_R & 0xFF;
	
	if (c == 'A')
	{
		TIMER_0A_MATCH_R = match_val1;
		Load_A = 1;
	}
	
	if (c == 'B')
	{
		TIMER_0A_MATCH_R = 15999;
		Load_A = 0;
	}
	
	if (c == 'C')
	{
		TIMER_0B_MATCH_R = match_val1;
		Load_B = 1;
	}
	
	if (c == 'D')
	{
		TIMER_0B_MATCH_R = 15999;
		Load_B = 0;
	}
	
	if (c == 'E')
	{
		TIMER_1A_MATCH_R = match_val2;
		Load_C = 1;
	}
	
	if (c == 'F')
	{
		TIMER_1A_MATCH_R = 15999;
		Load_C = 0;
	}
	
	if (c == 'G')
	{
		TIMER_1B_MATCH_R = match_val2;
		Load_D = 1;
	}
	
	if (c == 'H')
	{
		TIMER_1B_MATCH_R = 15999;
		Load_D = 0;
	}	
}

// Intialization routine for setting up the required ports
void ADC_0_Init ( void )
{	
	// Enable the clock for ADC0
	SYSCTL_RCGC_ADC_R |= 0x01 ;
	
	// We will use PE2 & PE1 as analog inputs for ADC Module 
	SYSCTL_RCGC_GPIO_R |= 0x10;
	
	// Delay to stable the clock
	Delay (3) ;
	
	// GPIO Configurations
	GPIO_PORTE_DEN_R &= ~(0x06 );
	GPIO_PORTE_AMSEL_R |= 0x06 ;

	// Sampling Rate is set to 250 Ksps
	ADC0_PERI_CONFIG_R |= 0x03 ; 
	
	// Disable Sample Sequencer 2 & 3
	ADC0_ACTIVE_SS_R = ~(0x0000000C);
	
	// Select AN1 ( PE2 ) as the analog input
	ADC0_SS3_IN_MUX_R = 0x01 ;
	ADC0_SS2_IN_MUX_R = 0x02;

	//Enabling Interrupt for first sample
	//First sample is the last sample
	ADC0_SS3_CONTROL_R |= 0x06 ;
	ADC0_SS2_CONTROL_R = 0x06;
	
	// 16 x oversampling and then averaged
	ADC0_SAMPLE_AVE_R |= 0x04 ;
	
	// Unmask ADC0 sequence 3 interrupt
	ADC0_INT_MASK_R |= 0x0C;
	
	// Enable ADC0 sequencer 3 interrupt in NVIC
	NVIC_EN0_R = NVIC_EN0_INT17 ;
	NVIC_EN0_R = NVIC_EN0_INT16;

	// Enable Sample Sequencer 2 & 3
	ADC0_ACTIVE_SS_R = 0x0000000C ;
}

void LM35_Sample( void )
{
// Processor sample sequencer initiate for sequencer 3
ADC0_PROC_INIT_SS_R |= 0x08 ;
}

void LDR_Sample (void)
{
	// Processor sample sequencer initiate for sequencer 2 - ADC0
	ADC0_PROC_INIT_SS_R |= 0x04;
}	

// ADC 0 interrupt service routine for sequencer 2
void ADC0Seq2_Handler (void)
{
	unsigned int adc_data = 0;
	// Read the raw analog value from ADC FIFO
	adc_data = ( ADC0_SS2_FIFO_DATA_R & 0xFFF );
	
	Intensity = (adc_data * 0.8);
	// Clear the interrupt from ADC0 sequencer 2
	ADC0_INT_STATUS_CLR_R |= 0x04 ;
}

// ADC 0 interrupt service routine for sequencer 3
void ADC0Seq3_Handler ( void )
{
	unsigned int adc_data = 0;
	// Read the raw analog value from ADC FIFO
	adc_data = ( ADC0_SS3_FIFO_DATA_R & 0xFFF );
	// Convert the raw value to Celsius scale
	Temperature = (int) ((( adc_data * 3.3) /4096) * 100);

	// Clear the interrupt from ADC0 sequencer 3
	ADC0_INT_STATUS_CLR_R |= 0x08 ;
}

void Seven_Segment ( void )
{	
	GPIO_PORTA_DATA_R = 0xFF;

	GPIO_PORTB_DATA_R = Lookup_7Seg_Disp[Temperature/10];
	GPIO_PORTA_DATA_R = seg_select [0];
	Delay (1000);
	GPIO_PORTA_DATA_R = 0xFF;

	GPIO_PORTB_DATA_R = Lookup_7Seg_Disp[Temperature%10];
	GPIO_PORTA_DATA_R = seg_select [1];
	Delay (1000);

	GPIO_PORTA_DATA_R = 0xFF;
}

/* Timer 0 Configuration */
void Timer_0_Init(void)
{
	/* Enable clock for Timer */
	SYSCTL_RCGCTIMER_R |= 0x01;

	/* Disable Timer before setup */
	TIMER_0_CTL_R &= 0x00000000;	//Page 737 of Data Sheet
	
	/* Configure 16-bit timer mode */
	//Put 4 in GPTM_R for the 32 bit timer to be used as 16 bit timer
	TIMER_0_CFG_R |= 0x00000004; //Page 727 of Data Sheet									
	
	/* Configure periodic mode */
	//Setting it in periodic as well as in PWM mode & as a down counter
	TIMER_0A_MODE_R |= 0x0000000A;					

	TIMER_0B_MODE_R |= 0x0000000A;					

	/* Set initial load value or reload value*/
	//We want PWM frequency of 1kHz which means (1/1000)s of interval
	//Reload = (Interval)*(clk_frequency)
	//Reload = (1/1000)*(16MHz)
	TIMER_0A_ILR_R = Reload;																		

	TIMER_0B_ILR_R = Reload;																		

	//TIMER_0A_MATCH_R = (int)(Reload * ((100-match_val1)/100));
	TIMER_0A_MATCH_R = Reload-1;
	TIMER_0B_MATCH_R = Reload-1;

	//TIMER_0B_MATCH_R = (int)(Reload * ((100-match_val1)/100));
	
	/* Enable Timer after setup */
	TIMER_0_CTL_R |= 0x00000101;	//Page 737 of Data Sheet
}

/* Timer 2 Configuration */
void Timer_1_Init(void)
{
	/* Enable clock for Timer */
	SYSCTL_RCGCTIMER_R |= 0x02;

	/* Disable Timer before setup */
	TIMER_1_CTL_R &= 0x00000000;	//Page 737 of Data Sheet
	
	/* Configure 16-bit timer mode */
	//Put 4 in GPTM_R for the 32 bit timer to be used as 16 bit timer
	TIMER_1_CFG_R |= 0x00000004; //Page 727 of Data Sheet									
	
	/* Configure periodic mode */
	//Setting it in periodic as well as in PWM mode & as a down counter
	TIMER_1A_MODE_R |= 0x0000000A;					

	TIMER_1B_MODE_R |= 0x0000000A;					

	/* Set initial load value or reload value*/
	//We want PWM frequency of 1kHz which means (1/1000)s of interval
	//Reload = (Interval)*(clk_frequency)
	//Reload = (1/1000)*(16MHz)
	TIMER_1A_ILR_R = Reload;																		

	TIMER_1B_ILR_R = Reload;																		
	
	//TIMER_0A_MATCH_R = (int)(Reload * ((100-match_val1)/100));
	TIMER_1A_MATCH_R = Reload-1;
	TIMER_1B_MATCH_R = Reload-1;

	//TIMER_0B_MATCH_R = (int)(Reload * ((100-match_val1)/100));
	
	/* Enable Timer after setup */
	TIMER_1_CTL_R |= 0x00000101;	//Page 737 of Data Sheet
}

/* This function generates the delay . */
void delay ( volatile unsigned int delay )
{
long int i;
for (i = 0; i < delay ; i ++)
{
	Seven_Segment();
}
}

/* This function generates the delay . */
void Delay ( volatile unsigned int delay )
{
long int i;
for (i = 0; i < delay ; i ++);
}

/*	If need to use double or float values i.e. floating point unit is used,
	replace empty SystemInit function with this and also include the header file.*/

void App_Display ( void )
{
	char Temp[17];
	char Inten[17];

	sprintf(Temp,"%d",Temperature);
	sprintf(Inten,"%d",Intensity);
	
	UART_4_OutChar(CR);		
	UART_4_OutChar(LF);	
	UART_4_OutString("Current Temperature is ");
	UART_4_OutString(Temp);

	UART_4_OutChar(CR);		
	UART_4_OutChar(LF);		
	UART_4_OutString("The Intensity of Light is ");
	UART_4_OutString(Inten);

}

void Set_MatchValue1(void)
{	
	if (Temperature <= 10)
	{
		match_val1 = 15999;	//Fan doesn't work
	}
	
	if (Temperature > 10 && Temperature <= 13)
	{
		match_val1 = 14400;
	}

	if (Temperature > 13 && Temperature <= 16)
	{
		match_val1 = 12800;
	}

	if (Temperature > 16 && Temperature <= 19)
	{
		match_val1 = 11200;
	}

	if (Temperature > 19 && Temperature <= 22)
	{
		match_val1 = 9600;
	}
	
	if (Temperature > 22 && Temperature <= 25)
	{
		match_val1 = 8000;
	}

	if (Temperature > 25 && Temperature <= 30)
	{
		match_val1 = 6400;
	}

	if (Temperature > 30 && Temperature <= 35)
	{
		match_val1 = 4800;
	}

	if (Temperature > 35 && Temperature <= 40)
	{
		match_val1 = 3200;
	}

	if (Temperature > 40 && Temperature <= 45)
	{
		match_val1 = 1600;
	}

	if (Temperature > 45)
	{
		match_val1 = 0;	//Full Speed of Fan
	}
}

void Set_MatchValue2(void)
{	
	if (Intensity <= 250)
	{
		match_val2 = 15999;	//Light doesn't work
	}
	
	if (Intensity > 250 & Intensity <= 500)
	{
		match_val2 = 14400;
	}

	if (Intensity > 500 & Intensity <= 750)
	{
		match_val2 = 12800;
	}

	if (Intensity > 750 & Intensity <= 1000)
	{
		match_val2 = 11200;
	}

	if (Intensity > 1000 & Intensity <= 1250)
	{
		match_val2 = 9600;
	}

	if (Intensity > 1250 & Intensity <= 1500)
	{
		match_val2 = 8000;
	}

	if (Intensity > 1500 & Intensity <= 1750)
	{
		match_val2 = 6400;
	}

	if (Intensity > 1750 & Intensity <= 2000)
	{
		match_val2 = 4800;
	}

	if (Intensity > 2000 & Intensity <= 2250)
	{
		match_val2 = 3200;
	}

	if (Intensity > 2250 & Intensity <= 2500)
	{
		match_val2 = 1600;
	}
		
	if (Intensity > 2500)
	{
		match_val2 = 0;	//Full Brightness of LED
	}
}

int DutyCycle (int match)
{
	int duty;
	
	if (match == 15999) duty = 0;
	if (match == 14400)	duty = 10;
	if (match == 12800) duty = 20;
	if (match == 11200) duty = 30;
	if (match == 9600) 	duty = 40;
	if (match == 8000) 	duty = 50;
	if (match == 6400) 	duty = 60;
	if (match == 4800) 	duty = 70;
	if (match == 3200) 	duty = 80;	
	if (match == 1600) 	duty = 90;
	if (match == 0)			duty = 100;	
	
	return duty;
}
	
void SystemInit (void)
{	
	//FPU --> Floating Point Unit
	/* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}
