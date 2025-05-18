#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../Common/Include/stm32l051xx.h"
#include "../Common/Include/serial.h"
#include "adc.h"
#include "lcd.h"
#include "UART2.h"

#define F_CPU 32000000L
#define SYSCLK 32000000L
#define DEF_F 15000L
//#define TICK_FREQ 2000L

volatile int Count = 0;

void ToggleLED(void) 
{    
	GPIOA->ODR ^= BIT8; // Toggle PA8
}

void TIM21_Handler(void) 
{
	TIM21->SR &= ~BIT0; // clear update interrupt flag
	ToggleLED(); // toggle the state of the LED every half second
}

// LQFP32 pinout with the pins that can be analog inputs.  This code uses ADC_IN9.
//                 ----------
//           VDD -|1       32|- VSS
//          PC14 -|2       31|- BOOT0
//          PC15 -|3       30|- PB7
//          NRST -|4       29|- PB6
//          VDDA -|5       28|- PB5
// (ADC_IN0) PA0 -|6       27|- PB4
// (ADC_IN1) PA1 -|7       26|- PB3
// (ADC_IN2) PA2 -|8       25|- PA15 //connect to JDY40 tx
// (ADC_IN3) PA3 -|9       24|- PA14 //connect to JDY40 rx
// (ADC_IN4) PA4 -|10      23|- PA13 //connect to JDY40
// (ADC_IN5) PA5 -|11      22|- PA12 
// (ADC_IN6) PA6 -|12      21|- PA11
// (ADC_IN7) PA7 -|13      20|- PA10 (Reserved for RXD)
// (ADC_IN8) PB0 -|14      19|- PA9  (Reserved for TXD)
// (ADC_IN9) PB1 -|15      18|- PA8  (LED+1k)
//           VSS -|16      17|- VDD
//                 ----------

void Configure_Pins (void)
{
	RCC->IOPENR |= BIT0; // peripheral clock enable for port A
	
	// Make pins PA0 to PA5 outputs (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT0|BIT1)) | BIT0; // PA0
	GPIOA->OTYPER &= ~BIT0; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2; // PA1
	GPIOA->OTYPER &= ~BIT1; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4; // PA2
	GPIOA->OTYPER &= ~BIT2; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6; // PA3
	GPIOA->OTYPER &= ~BIT3; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT8|BIT9)) | BIT8; // PA4
	GPIOA->OTYPER &= ~BIT4; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT10|BIT11)) | BIT10; // PA5
	GPIOA->OTYPER &= ~BIT5; // Push-pull
	
	
   // GPIOA->MODER  = (GPIOA->MODER & ~(BIT17|BIT16) ) | BIT16; // Make pin PA8 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	
	// Configure the pin used for analog input: PB1 (pin 15)
	RCC->IOPENR  |= BIT1;         // peripheral clock enable for port B
	GPIOA->MODER |= (BIT14|BIT15);  // Select analog mode for PB1 (pin 15 of LQFP32 package)
	
	GPIOB->MODER |= (BIT0|BIT1); 
}

void Button_Init(void)
{
    RCC->IOPENR |= BIT1;  // Enable clock for Port B

    // Configure PB5 (pin28) as input:
    // For PB5, the mode bits are at positions (5 * 2) and (5 * 2 + 1), i.e., bits 10 and 11
    GPIOB->MODER &= ~(3 << (5 * 2));   // Clear mode bits for PB5 (set as input)
    // Configure PB5 with internal pull-up (set PUPDR bits to '01')
    GPIOB->PUPDR &= ~(3 << (5 * 2));
    GPIOB->PUPDR |= (1 << (5 * 2));

    // Configure PB3 (pin26) as input:
    // For PB3, the mode bits are at positions (3 * 2) and (3 * 2 + 1), i.e., bits 6 and 7
    GPIOB->MODER &= ~(3 << (3 * 2));   // Clear mode bits for PB3 (set as input)
    // Configure PB3 with internal pull-up (set PUPDR bits to '01')
    GPIOB->PUPDR &= ~(3 << (3 * 2));
    GPIOB->PUPDR |= (1 << (3 * 2));

    // Configure PB4 (assumed to be pin27) as input with internal pull-up
    // For PB4, the mode bits are located at positions (4 * 2) and (4 * 2 + 1), i.e., bits 8 and 9
    GPIOB->MODER &= ~(3 << (4 * 2));   // Clear mode bits for PB4 (set as input)
    GPIOB->PUPDR &= ~(3 << (4 * 2));   // Clear pull-up/pull-down configuration for PB4
    GPIOB->PUPDR |= (1 << (4 * 2));    // Set PB4 to pull-up
}




void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	RCC->IOPENR |= BIT0; // peripheral clock enable for port A

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

}

void Buzzer_Init(void)
{
	// Set up output port bit for blinking LED
	RCC->IOPENR |= 0x00000001; // peripheral clock enable for port A
    GPIOA->MODER  = (GPIOA->MODER & ~(BIT17|BIT16) ) | BIT16;
 	// Make pin PA0 output (page 172, two bits used to configure: bit0=1, bit1=0)
	
	// Set up timer
	RCC->APB2ENR |= BIT2;  // turn on clock for timer21 (UM: page 188)
	//TIM21->ARR = SYSCLK/TICK_FREQ;
	NVIC->ISER[0] |= BIT20; // enable timer 21 interrupts in the NVIC
	TIM21->CR1 |= BIT4;      // Downcounting    
	TIM21->CR1 |= BIT0;      // enable counting    
	TIM21->DIER |= BIT0;     // enable update event (reload event) interrupt  
	__enable_irq();
	TIM21->ARR = 0;
}

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	//SysTick->CTRL = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}



void delayms(int len)
{
	while(len--) wait_1ms();
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}
void ReceptionOff (void)
{
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2()>0) egetc2(); // Clear FIFO
}

void main(void)
{
	float a;	//analog input for x direction (joystick)
	float b;	//analog input for y direction (joystick)
	float d;	//analog input from battery
	int adcx;
	int adcy;
	int adcz;
	int int_a;	//change a to integer
	int int_b;	//change b to integer
	char buff[800];
	char lcd_b[17];
    int timeout_cnt=0;
    int cont1=0, cont2=100;
    int flag_w = 0;		//flag to check if it is in welcome state
	int flag_m = 0;		//flag to check the page of menu
	int flag_s = 0;		//flag for mode chosing
	int flag_a = 0; 	//flag to start auto picking
	int flag_b = 0;		//flag to check if the button of joystick has been pressed
	int flag_o = 0;
	int flag_s4 = 0;	
	int cnt = 0;
	char c;
	int buzz;
	int buzz_pre;
	int buzz_sound;
	int buzz_base = 0;
	int coin_count = 0;
	int strength = 0;
	int v_m = 0;
	float v_m_f = 0.0;

    Buzzer_Init();
	Configure_Pins();
	initADC();
	LCD_4BIT();
	Button_Init();
	Hardware_Init();
	initUART2(9600);

    delayms(500); // Give PuTTY time to start
	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printf("STM32L051 ADC Test.  Analog input is PB1 (pin 15).\r\n");

	
	waitms(1000); // Give putty some time to start.
	printf("\r\nJDY-40 Master test\r\n");

	

	ReceptionOff();
	//To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDCBCB\r\n");
	cnt=0;
while(1)
{	
	while(flag_w == 0)
    {         	 
        LCDprint("Welcome to      ", 1, 1);
        LCDprint("Project 2 demo  ", 2, 1);
        if(!(GPIOB->IDR & BIT5))
        {
            waitms(50);  // Debounce delay
            if(!(GPIOB->IDR & BIT5))
            {
                flag_w = 1;
            }
        }
		
    }

    // flag_w = 1, in mode choosing state
    while(flag_w == 1)
    {	
		//flag_m = 0 , in fisrt page of mode
        while(flag_m == 0)
        {
            LCDprint("Mode:           ", 1, 1);
            LCDprint("a.Auto picking  ", 2, 1);
            if(!(GPIOB->IDR & BIT5))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_m = 1;
                }
            }
			if(!(GPIOB->IDR & BIT3))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT3))
                {
                    flag_m = -1;
					flag_s = 1;
					flag_w = 2;
					flag_a = 0;
                }
            }
        }
		//flag_m = 1 , in second page of mode
        while(flag_m == 1)
        {	
			LCDprint("Mode:           ", 1, 1);
            LCDprint("b.Manual picking", 2, 1);
            if(!(GPIOB->IDR & BIT5))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_m = 2;
                }
            }
			if(!(GPIOB->IDR & BIT3))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT3))
                {
                    flag_m = -1;
					flag_s = 2;
					flag_w = 2;
                }
            }
        }
		//flag_m = 2 , in third page of mode
		while(flag_m == 2)
        {	
			LCDprint("Mode:           ", 1, 1);
            LCDprint("c.obstacle avoid", 2, 1);
            if(!(GPIOB->IDR & BIT5))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {	
					flag_m = 3; 
                }
            }
			if(!(GPIOB->IDR & BIT3))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT3))
                {
                    flag_m = -1;
					flag_s = 3;
					flag_w = 2;
					flag_o = 0;
                }
            }
        }
		//flag_m = 3, in fourth page of mode
		while(flag_m == 3)
        {	
			LCDprint("Mode:           ", 1, 1);
            LCDprint("d.Battery check ", 2, 1);
            if(!(GPIOB->IDR & BIT5))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {	
					flag_m = 0; 
                }
            }
			if(!(GPIOB->IDR & BIT3))
            {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT3))
                {
                    flag_m = -1;
					flag_s = 4;
					flag_w = 2;
					flag_o = 0;
					flag_s4 = 1;
                }
            }
        }
    }
    // Start the whole programe
	//flag_s = 1 for mode 1:aoto picking
	//flag_s = 2 for mode 2:manual picking
	//flag_s = 3 for mode 3:wait to add(bonus)

	//mode 1: auto picking
	while(flag_s == 1){
		buzz = 0;
		if(!(GPIOB->IDR & BIT4))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT4))
                {
                    flag_a = 1;
                }
        }
		if(!(GPIOB->IDR & BIT5))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_s = 0;
					flag_w = 1;
					flag_m = 0;
					flag_a = 3;
                }
        }
		if(flag_a == 0){
			LCDprint("Mode:Auto picking", 1, 1);
        	LCDprint("Press to start   ", 2, 1);
		}
		else if(flag_a == 1){

			LCDprint("Start!", 1, 1);
			waitms(200);
			LCDprint("                 ", 1, 1);
        	LCDprint("                 ", 2, 1);
			sprintf(buff, "%03d %03d\n", -20, -20); 
			eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
			// Wait a bit so the slave has a chance to get ready
			waitms(50); // This may need adjustment depending on how busy is the slave
			eputs2(buff); // Send the test message
			flag_a = 2;
			//}
			
				
		}
		else if(flag_a == 2){
			if(coin_count <= 20){
				sprintf(lcd_b, "COIN NUMBER:%d", coin_count);
				LCDprint(lcd_b, 1, 1);
				sprintf(lcd_b, "strength:%d", strength);
				LCDprint(lcd_b, 2, 1);
			}
			else{
				LCDprint("finish picking", 1, 1);
			}
			eputc2('@'); 
			waitms(100);
			timeout_cnt=0;
			while(1)
			{
				if(ReceivedBytes2()>5) break; // Something has arrived
				if(++timeout_cnt>30) break; // Wait up to 25ms for the repply
				Delay_us(100); // 100us*250=25ms
			}
			
			if(ReceivedBytes2()>5) // Something has arrived from the slave
			{
				egets2(buff, sizeof(buff)-1);
				if(strlen(buff)==6)
				{	
					printf("Slave says: %s  \r", buff);
					sscanf(buff, "%d", &buzz);
					if(buzz_pre < buzz){
						coin_count++;
					}
					else{
						coin_count = coin_count;
					}
					buzz_pre = buzz;
					strength = buzz;
					//printf("%d",buzz);
				}
				else
				{
					while (ReceivedBytes2()) egetc2(); // Clear FIFO
					printf("*** BAD MESSAGE ***: %s\r", buff);
				}
			}
			else // Timed out waiting for reply
			{
				while (ReceivedBytes2()) egetc2(); // Clear FIFO
				printf("NO RESPONSE\r\n", buff);
				buzz_pre = 0;	
			}
			///buzzer
			buzz_sound = buzz - 57400;
			if(buzz_sound >1100){
				TIM21->ARR = SYSCLK / (1000L);
			}
			else if(buzz_sound<= 1100 && buzz_sound>800){
				TIM21->ARR = SYSCLK / (1500L);
			}
			else if(buzz_sound<= 800 && buzz_sound>500){
				TIM21->ARR = SYSCLK / (2000L);
			}
			else if(buzz_sound<= 500 && buzz_sound>300){
				TIM21->ARR = SYSCLK / (2500L);
			}
			else{
				TIM21->ARR = 0;
			}
			//TIM21->ARR = buzz;
			
		}
		

	}

	//mode 2: manual picking
	while(flag_s == 2)
	{	
		if(!(GPIOB->IDR & BIT4))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT4))
                {
                    flag_b = 1;
                }
        }
		if(!(GPIOB->IDR & BIT5))
        {
                waitms(100);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_s = 0;
					flag_w = 1;
					flag_m = 0;
                }
        }

		adcx=readADC(ADC_CHSELR_CHSEL8);
		a=(adcx*3.3)/0x1000;		//pin9
		a = a*100;
		
		adcy = readADC(ADC_CHSELR_CHSEL7);
		b = (adcy*3.3)/0x1000;		//pin8
		b = b*100;
		
		int_a = (int)a;
		int_b = (int)b;

		
		sprintf(lcd_b, "x= %.3f", a);
		LCDprint(lcd_b, 1, 1);
		sprintf(lcd_b, "y= %.3f", b);
		LCDprint(lcd_b, 2, 1);
		
		//GPIOA->ODR ^= BIT8; // Complement PA8 (pin 18)
		//delayms(500);
		if(flag_b ==0){
			sprintf(buff, "%03d %03d\n", int_a, int_b); // Construct a test message
		}
    	else if(flag_b == 1){
			sprintf(buff, "%03d %03d\n", -10, -10); // Construct a test message, send -10 if the button for arm is pressed
			
			flag_b = 0;
		}
		
		eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
		// Wait a bit so the slave has a chance to get ready
		waitms(50); // This may need adjustment depending on how busy is the slave
		eputs2(buff); // Send the test message
		waitms(50);

		eputc2('@');
		
		//printf("Slave says: %s\r\n", buff);
		timeout_cnt=0;
		while(1)
		{
			if(ReceivedBytes2()>5) break; // Something has arrived
			if(++timeout_cnt>30) break; // Wait up to 25ms for the repply
			Delay_us(100); // 100us*250=25ms
		}
		
		if(ReceivedBytes2()>5) // Something has arrived from the slave
		{
			egets2(buff, sizeof(buff)-1);
			if(strlen(buff)==6)
			{
				printf("Slave says: %s  \r", buff);
			}
			else
			{
				while (ReceivedBytes2()) egetc2(); // Clear FIFO
				printf("*** BAD MESSAGE ***: %s\r", buff);
			}
		}
		else // Timed out waiting for reply
		{
			while (ReceivedBytes2()) egetc2(); // Clear FIFO
			printf("NO RESPONSE\r\n", buff);	
		}


		//Something has arrived
		//printf("V=%fV V=%fV\r", a, b);
		TIM21->ARR = 0;
		fflush(stdout);
		

	}
		
	while(flag_s == 3){
		if(!(GPIOB->IDR & BIT5))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_s = 0;
					flag_w = 1;
					flag_m = 0;
                }
        }
		if(!(GPIOB->IDR & BIT4))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT4))
                {
                    flag_o = 1;
                }
        }
		if(flag_o == 1){
			LCDprint("Start             ", 1, 1);
        	LCDprint("                  ", 2, 1);
			sprintf(buff, "%03d %03d\n", -30, -30); 
			printf("11111");
			eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
			// Wait a bit so the slave has a chance to get ready
			waitms(50); // This may need adjustment depending on how busy is the slave
			eputs2(buff); // Send the test message
			waitms(50);
			flag_o = 0;
		}
		else if(flag_o == 0){
			LCDprint("Auto: obstacle    ", 1, 1);
        	LCDprint("      avoidance   ", 2, 1);
		}				
	}			
	while(flag_s == 4){
		if(!(GPIOB->IDR & BIT5))
        {
                waitms(50);  // Debounce delay
                if(!(GPIOB->IDR & BIT5))
                {
                    flag_s = 0;
					flag_w = 1;
					flag_m = 0;
                }
        }
		if(flag_s4){
			flag_s4 = 0;
			sprintf(buff, "%03d %03d\n", -40, -40); 
			eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
			// Wait a bit so the slave has a chance to get ready
			waitms(50); // This may need adjustment depending on how busy is the slave
			eputs2(buff); // Send the test message
			

		}
		eputc2('@');
		waitms(50);
		//printf("Slave says: %s\r\n", buff);
		timeout_cnt=0;
		while(1)
		{
			if(ReceivedBytes2()>5) break; // Something has arrived
			if(++timeout_cnt>30) break; // Wait up to 25ms for the repply
			Delay_us(100); // 100us*250=25ms
		}
		
		if(ReceivedBytes2()>5) // Something has arrived from the slave
		{
			egets2(buff, sizeof(buff)-1);
			if(strlen(buff)==6)
			{
				printf("Slave says: %s  \r", buff);
				sscanf(buff, "%d", &v_m);
				v_m_f = (float)v_m;
				v_m_f = v_m_f/10000;

			}
			else
			{
				while (ReceivedBytes2()) egetc2(); // Clear FIFO
				printf("*** BAD MESSAGE ***: %s\r", buff);
			}
		}
		else // Timed out waiting for reply
		{
			while (ReceivedBytes2()) egetc2(); // Clear FIFO
			printf("NO RESPONSE\r\n", buff);	
		}

		adcz=readADC(ADC_CHSELR_CHSEL6);
		d =(adcz*3.3)/0x1000;		//pin9
		d = d / 0.983 *(0.983+2.96);
		sprintf(lcd_b, "controler= %.2f", d);
		LCDprint(lcd_b, 1, 1);
		sprintf(lcd_b, "car= %.2f", v_m_f);
		LCDprint(lcd_b, 2, 1);
		
	}
}

}