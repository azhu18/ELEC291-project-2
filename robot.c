#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

volatile unsigned int pwm_reload;
volatile unsigned char pwm_state=0;
volatile unsigned char count20ms;
volatile unsigned char motor_flag=0; 
volatile unsigned char servo_up=50, servo_down=150; 
volatile unsigned int servo_counter = 0, isr_count = 0;
volatile unsigned int action_flag = 1; 
volatile unsigned int threshold;
volatile unsigned int motor_counter = 0;
float pulse_width;
xdata float period = 0.0;
xdata float period_100 = 0.0;
xdata float period_sum = 0.0;
volatile unsigned int z;
unsigned int bluetooth_flag;

//xdata float pulse_width;
volatile unsigned int distance = 0;

#define PWMOUT_left P2_1
#define PWMOUT_left_inv P2_2
#define PWMOUT_right P2_5
#define PWMOUT_right_inv P2_6
//servo arm 
#define SERVO_up P1_5
#define SERVO_down P1_7
#define EMAGNET  P1_4
#define VDD 3.3035

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L
#define SARCLK 18000000L
//#define RELOAD_10MS (0x10000L-(SYSCLK/(12L*100L)))
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate
#define RELOAD_50us (0x10000L - (SYSCLK/(12L*20000L)))
#define SARCLK 18000000L
#define MOTOR_PWM_PERIOD_TICKS 20
//auto avd
#define TRIG_PIN 0x01 //1.0
#define ECHO_PIN 0x02 //1.1
#define DF_PLAY_PIN 0x08 //1.3
#define DISTANCE_THRESHOLD 5  // Trigger playback when within this distance (cm)
#define MAX_DISTANCE 50        // Reasonable maximum distance (cm)
#define MAX_TIMEOUT 30000      // Maximum timeout (microseconds)

//idata char buff[20];
xdata char buff[20]; 
unsigned char overflow_count = 0;

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;
	RSTSRC = 0x02;

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	// Configure the pins used for square output

	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	P2MDOUT |= 0X01;
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs	
	// P0SKIP |= 0b_0000_0001; 
	// P0MDOUT |= 0b_0000_0010;
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X10; // Enable T0 on P0.0
	XBR1     = 0X00; // Enable T0 on P0.0
	XBR2     = 0x40; // Enable crossbar and weak pull-ups
	XBR2     = 0x41;
	P2MDOUT |= 0x0F;  
	//auto avd
	P1MDOUT |= TRIG_PIN | DF_PLAY_PIN; // Set TRIG and DF_PLAY to push-pull output
	P1MDOUT &= ~ECHO_PIN; // Ensure ECHO is in input mode
	P1 |= DF_PLAY_PIN;
	 
	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  	
  	P2_0=1; // 'set' pin to 1 is normal operation mode.

	//return 0;

	P2MDOUT |= 0x0F;  
	P1MDOUT |= 0xC0; 

    
    PWMOUT_left = 1;
    PWMOUT_left_inv = 1;
    PWMOUT_right = 1;
    PWMOUT_right_inv = 1;

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;   // Stop Timer5; Clear TF5; WARNING: lives in SFR page 0x10
	//CKCON1|=0b_0000_0100; // Timer 5 uses the system clock
	pwm_reload=0x10000L-(SYSCLK*1.5e-3)/12.0; // 1.5 miliseconds pulse is the center of the servo
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	
	return 0;
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	
	 // Clear Timer5 interrupt flag
	// Since the maximum time we can achieve with this timer in the
	// configuration above is about 10ms, implement a simple state
	// machine to produce the required 20ms period.
	//servo_counter ++;
	if(action_flag==0){
		SFRPAGE=0x10;
		TF5H = 0;
		TMR5RL=RELOAD_10us;
		servo_counter++;
		if(servo_counter == 2000){
			servo_counter=0;
		}
		if(servo_up>=servo_counter){
			SERVO_up=1;
		}else{
			SERVO_up=0;
		}
		if(servo_down>=servo_counter){
			SERVO_down=1;
		}else{
			SERVO_down=0;
		}
	}

	else if(action_flag==1){
		SFRPAGE=0x10;
		TF5H = 0;
		TMR5RL=RELOAD_50us;
		motor_counter++;
        if (motor_counter >= MOTOR_PWM_PERIOD_TICKS) {
            motor_counter = 0;
        }
        threshold =  (unsigned int)(pulse_width * MOTOR_PWM_PERIOD_TICKS);
        if (motor_counter < threshold) {
            switch (motor_flag) {
                case 1: // forward
                    PWMOUT_left = 0;
                    PWMOUT_right = 0;
                    PWMOUT_left_inv = 1;
                    PWMOUT_right_inv = 1;
                    break;
                case 2: // back
                    PWMOUT_left = 1;
                    PWMOUT_right = 1;
                    PWMOUT_left_inv = 0;
                    PWMOUT_right_inv = 0;
                    break;
                case 3: // right
                    PWMOUT_left = 0;
                    PWMOUT_right = 1;
                    PWMOUT_left_inv = 1;
                    PWMOUT_right_inv = 0;
                    break;
                case 4: // left
                    PWMOUT_left = 1;
                    PWMOUT_right = 0;
                    PWMOUT_left_inv = 0;
                    PWMOUT_right_inv = 1;
                    break;
                case 5: // stop
					PWMOUT_left = 1;
					PWMOUT_right = 1;
					PWMOUT_left_inv = 1;
					PWMOUT_right_inv = 1;
					break;
				case 6: //right_forward
					PWMOUT_left = 0;
					PWMOUT_right = 0;
					PWMOUT_left_inv = 1;
					PWMOUT_right_inv = 0;
					break;
				case 7: //left_forward
					PWMOUT_left = 0;
					PWMOUT_right = 0;
					PWMOUT_left_inv = 0;
					PWMOUT_right_inv = 1;
					break;
				case 8: //left_back
					PWMOUT_left = 0;
					PWMOUT_right = 1;
					PWMOUT_left_inv = 0;
					PWMOUT_right_inv = 0;
					break;
				case 9: //right_back
					PWMOUT_left = 1;
					PWMOUT_right = 0;
					PWMOUT_left_inv = 0;
					PWMOUT_right_inv = 0;
					break;
                default:
                    PWMOUT_left = 1;
                    PWMOUT_right = 1;
                    PWMOUT_left_inv = 1;
                    PWMOUT_right_inv = 1;
                    break;
            }
        } else {
            PWMOUT_left = 1;
            PWMOUT_right = 1;
            PWMOUT_left_inv = 1;
            PWMOUT_right_inv = 1;
        }

	}
	if(isr_count  == 1000){
		isr_count = 0;
		period_sum = 0.0;
		for(z = 0; z < 10; z++){
			TL0=0;
			TH0=0;
			overflow_count=0;
			while(P0_2!=0); // Wait for the signal to be zero
			while(P0_2!=1); // Wait for the signal to be one
			TR0=1; // Start the timer
			while(P0_2!=0) // Wait for the signal to be zero
			{
				if(TF0==1) // Did the 16-bit timer overflow?
				{
					TF0=0;
					overflow_count++;
				}
			}
			while(P0_2!=1) // Wait for the signal to be one
			{
				if(TF0==1) // Did the 16-bit timer overflow?
				{
					TF0=0;
					overflow_count++;
				}
			}
			TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period!
			period=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);
			period_sum = period + period_sum;
		}
		period_100 = period_sum / 10.0;
	}
	isr_count++;
}
void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}
unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}
void TIMER0_Init(void)
{
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	TR0=0; // Stop Timer/Counter 0
}
// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}
//slave
void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s, unsigned char n)
{
	char c;
	unsigned char cnt;
	
	cnt=0;
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		
		if (cnt<n)
		{
			cnt++;
			*s=c;
			s++;
		}
		else
		{
			*s=0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff)-1);
	waitms(10);
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	P2_0=1; // 'set' pin to 1 is normal operation mode.
}
//auto avd
void play_sound_via_gpio(void)
{
    P1 &= ~DF_PLAY_PIN;
    waitms(200);  
    P1 |= DF_PLAY_PIN;
    //printf("* Playing sound *\n");
}
unsigned int measure_distance(void)
{
    unsigned int duration = 0;
    unsigned int distance;

    // Send trigger pulse
    P1 &= ~TRIG_PIN;    // Ensure TRIG is initially low
    Timer3us(2);        // Short delay
    P1 |= TRIG_PIN;     // Set TRIG high
    Timer3us(15);       // Pulse for at least 10 microseconds 3
    P1 &= ~TRIG_PIN;    // Set TRIG low

    // Wait for ECHO rising edge with timeout
    duration = 0;
    while ((P1 & ECHO_PIN) == 0)
    {
        Timer3us(10);
        duration += 10;
        if (duration > MAX_TIMEOUT)
        {
           // printf("Error: Rising edge timeout\n");
            return 0xFFFF;   // Return error value
        }
    }

    // Reset timer
    duration = 0;

    // Measure the duration of ECHO high pulse
    while ((P1 & ECHO_PIN) != 0)
    {
        Timer3us(10);
        duration += 10;
        if (duration > MAX_TIMEOUT)
        {
            //printf("Error: Falling edge timeout\n");
            return 0xFFFF;   // Return error value
        }
    }

    // Calculate distance (cm) - Speed of sound = 340 m/s, divide by 2 for round trip
    
    distance = duration / 59;

    // Filter out abnormal values
    if (distance > MAX_DISTANCE)
    {
        //printf("Error: Distance too large (%u cm)\n", distance);
        return 0xFFFF;  // Return error value
    }

    //printf("Raw distance: %u cm (pulse duration: %u us)\n", distance, duration);
    return distance;
}
unsigned int filtered_measure_distance(void)
{
#define FILTER_SIZE 3
    unsigned int readings[FILTER_SIZE];
    unsigned int sum = 0, valid_count = 0;
    unsigned int i;

    //printf("\n--- Starting new measurement ---\n");

    // Get multiple readings
    for (i = 0; i < FILTER_SIZE; i++)
    {
        //printf("Sample %u: ", i + 1);
        readings[i] = measure_distance();

        if (readings[i] != 0xFFFF && readings[i] <= MAX_DISTANCE)
        {
            sum += readings[i];
            valid_count++;
        }
        waitms(20);  // Wait 20ms before the next measurement
    }

    // If there are valid readings, return the average
    if (valid_count > 0)
    {
        unsigned int avg = sum / valid_count;
       // printf("Average distance: %u cm (from %u valid readings)\n", avg, valid_count);
        return avg;
    }
    else
    {
        //printf("No valid readings\n");
        return 0xFFFF;
    }
}

////////////
void main (void)
{
    //float pulse_width;
	xdata int get_voltagex = -1500, get_voltagey = -1500;
	int datachanged = 0; //data didn't change
	///arm
	unsigned char j, k;
    ////slave
	unsigned int cnt=0;
	xdata char auto_flag = 0;
	int skip_servo = 0;
	int coin_count = 0;
    char c;
	///detecter
	//float period;
	xdata float v[4];
	float vsum;
	int coinflag = 0;
    int perimeterflag = 0;
	int i;
	xdata float freq;
	xdata float  freq_buffer[10] = {0};
	int buf_index = 0;
	int freq_flag = 1;
	float xdata freq_base = 0;
	xdata float speed;
	xdata int send_message_count = 0;
	xdata long int freq_int;
	xdata float m_voltage = 0;
	long int m_voltage_int = 0;
	P1_4 = 0;
	//bluetooth 


	
	TIMER0_Init();
	InitPinADC(0, 6); // Configure P0.6 as analog input
	InitPinADC(0, 7); // Configure P0.7 as analog input
	InitPinADC(2, 4); // Configure P2.4 as analog input
	InitADC();
	
	UART1_Init(9600);
	ReceptionOff();
	// DFPlayer_Init(); //new way for dfplayer

	// To check configuration
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
	action_flag=0;
	servo_up=50;
	waitms(500);
	servo_down=150;
	waitms(500);
    count20ms = 0; // Count20ms is an atomic variable, so no problem sharing with timer 5 ISR
    // In a HS-422 servo a pulse width between 0.6 to 2.4 ms gives about 180 deg
    // of rotation range.
	motor_flag = 0;
	servo_counter=0;
	waitms(20);
	while(1)
	{
		servo_counter=0;
		//printf("while pass\n");
		if(RXU1()) // Something has arrived
		{
			c=getchar1();
			
			if(c=='!') // Master is sending message
			{
				getstr1(buff, sizeof(buff)-1);
				if(strlen(buff)==7)
				{
					printf("%s\r\n", buff);
					sscanf(buff, "%d %d", &get_voltagex, &get_voltagey) == 2;
				}
				else
				{
					printf("*** BAD MESSAGE ***: %s\r\n", buff);
				}				
			}
			else if(c=='@') // Master wants slave data
			{
				sprintf(buff, "%05u\n", cnt);
				//cnt++;
				waitms(5); // The radio seems to need this delay...
				//sendstr1(buff);
			}
		
		}

		//printf("dataflag= %d\n", datachanged);
		//printf("vx= %f, vy= %f\n", get_voltagex, get_voltagey);
		if(get_voltagex < -1000 || get_voltagey < -1000){
			auto_flag = 0;
			motor_flag = 5;

		}

		else if(get_voltagex == -20 && get_voltagey == -20 ){
			printf("autoflag = %d\n", auto_flag);
			 if(auto_flag == 1){
				printf("enter continue\n");
				continue;
			}
		//start auto  
			auto_flag = 1;
			freq_flag = 1;
			while(coin_count < 23)
			{
				printf("in while 20\n");
				// first detect the ADC output
				// Send the period to the serial port
				freq = 1/period_100;
				freq_int = (long int) freq;
				if (freq_flag)
				{
					freq_flag = 0;
					freq_base = freq;
				}
				printf("freq = %f, freq_base = %f", freq, freq_base);

				if(freq <= freq_base + 350){
					coinflag = 0;
				}
				else if(freq > freq_base + 350){
					coinflag = 1;
				}
				v[0] = Volts_at_Pin(QFP32_MUX_P0_6);
				v[2] = Volts_at_Pin(QFP32_MUX_P0_7);
				vsum = v[0] + v[2];
				
				if (vsum>=1.5)
				{
					printf ("Perimeter detected:v2.2=%10.8fV, v2.4=%10.8fV, flag = 1\n",v[0], v[2]);
					perimeterflag = 1;
				}
				else
				{
					// printf ("Perimeter not detected:v2.2=%10.8fV, v2.4=%10.8fV, flag = 0\n",v[0], v[2]);
					perimeterflag = 0;
				}
				if(perimeterflag == 0){
					action_flag = 1;
					motor_flag = 1;
					waitms(20);
					pulse_width=0.6;
				}
				else if(perimeterflag ==1){
					while (vsum>1.5)// keep turning right until it leaves perimeter
					{
						action_flag = 1;
						motor_flag = 3;
						waitms(20);
						pulse_width=0.5;
						waitms(1200);
						v[0] = Volts_at_Pin(QFP32_MUX_P0_6);
						v[2] = Volts_at_Pin(QFP32_MUX_P0_7);
						vsum = v[0] + v[2];
					}
					
				}
				if(coinflag == 1){
					while(send_message_count < 10){
						if(RXU1()) // Something has arrived
						{
							c=getchar1();
							
							if(c=='!') // Master is sending message
							{
								getstr1(buff, sizeof(buff)-1);
								if(strlen(buff)==7)
								{
									printf("%s\r\n", buff);
									sscanf(buff, "%d %d", &get_voltagex, &get_voltagey) == 2;
								}
								else
								{
									printf("*** BAD MESSAGE ***: %s\r\n", buff);
								}				
							}
							else if(c=='@') // Master wants slave data
							{
								sprintf(buff, "%ld\n", freq_int);
								printf("send = %ld\n", freq_int);
								waitms(20); // The radio seems to need this delay...
								sendstr1(buff);
								waitms(20);
								sendstr1(buff);
								waitms(20);
								sendstr1(buff);
								waitms(20);
								sendstr1(buff);
								waitms(20);
								sendstr1(buff);
							}
						
						}
						send_message_count++;
					}
					send_message_count = 0;
					coin_count++;
					
					waitms(20);
					action_flag = 1;


					motor_flag =2; //go backwards for .5 seconds
					waitms(10);
					pulse_width=0.7;
					waitms(330);
					motor_flag =5;
					waitms(50);
					pulse_width=0;
					waitms(50);
					action_flag=0;
					servo_counter=0;
					waitms(20);
					P1_4=0;

					servo_up=50;
					waitms(500);
					servo_down=150;
					
					waitms(500);
					servo_up=50;
					waitms(500);
					servo_down=150;
					waitms(2000);
			
					printf("Servo has been initialized.\n");
						

					for(j=50; j<=240; j+=5) 
					{
						servo_up = j;   
						waitms(20);
					}
					//servo_up=240;
					// waitms(100);
					P1_4=1;
					waitms(1800);
					
					for(j=150; j<195; j+=5) 
					{
						servo_down = j;   
						waitms(60);
					}
					waitms(500);
			
					for(j=195; j>170; j-=5) 
					{
						servo_down = j;   
						waitms(60);
					}
					waitms(1000);

					//up: 240-55  185  down: 170-95  75

					for(j=0;j<=37;j++){
						servo_up = (240-j*5); 
						if(j>=22){
							servo_down = (170-(j-21)*5); 
							waitms(5);
						}
						waitms(20);
					}

					
					// for(j=240; j>55; j-=7) 
					// {
					// 	servo_up = j;   
					// 	waitms(25);
					// }
					// waitms(1000);
			
					// for(j=170; j>95; j-=5) 
					// {
					// 	servo_down = j;   
					// 	waitms(20);
					// }
					waitms(1000);
					P1_4=0;
					waitms(500);
					printf("coin number = %d\n", coin_count);
					coinflag = 0;
				}
			}
			get_voltagex = -1500;
			get_voltagey = -1500;
			coin_count = 0;
			continue;
		}
		
		
		else if(get_voltagex == -10 && get_voltagey == -10) {
			if (skip_servo)
				continue;

			skip_servo = 1;
			
			printf("Asked to move servo, wheel has been stopped.\n");

			//printf("Servo has been initialized.\n");

			waitms(20);
			action_flag = 1;


					motor_flag =5;
					waitms(50);
					pulse_width=0;
					waitms(100);
					action_flag=0;
					servo_counter=0;
					waitms(20);
					P1_4=0;
											
					servo_up=50;
					waitms(500);
					servo_down=150;//
					waitms(500);
					for(j=150; j<110; j-=4) 
					{
						servo_down = j;   
						waitms(105);
					}


					servo_up=50;
					waitms(500);
					for(j=110; j<155; j+=3) 
					{
						servo_down = j;   
						waitms(100);
					}

					waitms(1000);
			
					printf("Servo has been initialized.\n");
						

					for(j=50; j<=240; j+=5) 
					{
						servo_up = j;   
						waitms(60);
					}
					//servo_up=240;
					// waitms(100);
					P1_4=1;
					waitms(1800);
					
					for(j=155; j<191; j+=4) 
					{
						servo_down = j;   
						waitms(80);
					}
					waitms(500);
			
					for(j=191; j>170; j-=2) 
					{
						servo_down = j;   
						waitms(80);
					}
					waitms(1000);

					//up: 240-55  185  down: 170-95  75

					for(j=0;j<=37;j++){
						servo_up = (240-j*5); 
						if(j>=22){
							servo_down = (170-(j-21)*5); 
							waitms(5);
						}
						waitms(20);
					}
					waitms(1000);
					P1_4=0;
					waitms(500);
				
		} 
	else if(get_voltagex ==  -30 && get_voltagey == -30)
	{	
		freq_flag = 1;
		printf("start auto avd\n");
		while(coin_count < 5)
		{
			action_flag = 1;
			motor_flag  = 1;
			pulse_width = 0.75;
			//send message to hc-05
			bluetooth_flag = 1;
			sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
			waitms(5); // This may need adjustment depending on how busy is the slave
			sendstr1(buff); // Send the test message

			distance = filtered_measure_distance();
			while(distance != 0xFFFF)
			{ 		//test if there is smth im front of the sensor
					//if smth is within the threshold distance 
				while (distance < DISTANCE_THRESHOLD)
				{
					printf("detected\n");
					play_sound_via_gpio();
					action_flag = 1;
					motor_flag = 5;
					waitms(50);
					pulse_width=0; //stop the wheel
					//send message to hc-05
					// bluetooth_flag = 0; //stop
					// sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
					// waitms(5); // This may need adjustment depending on how busy is the slave
					// sendstr1(buff); // Send the test message
					printf("wheel stops\n");

					waitms(800);
					action_flag = 1;
					motor_flag = 3;
					waitms(20);
					pulse_width=0.75;
					waitms(800); //spin
					printf("wheel spins");
					//send message to hc-05
					bluetooth_flag = 3; //spin
					sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
					waitms(5); // This may need adjustment depending on how busy is the slave
					sendstr1(buff); // Send the test message
					distance = filtered_measure_distance();
					
				}
				while (distance >= DISTANCE_THRESHOLD)
				{	
					
					printf("in while 20\n");
					// first detect the ADC output
					// Send the period to the serial port
					freq = 1/period_100;
					if (freq_flag)
					{
						freq_flag = 0;
						freq_base = freq;
					}
					printf("freq = %f, freq_base = %f", freq, freq_base);

					if(freq <= freq_base + 350){
						coinflag = 0;
					}
					else if(freq > freq_base + 350){
						coinflag = 1;
						
					}
					v[0] = Volts_at_Pin(QFP32_MUX_P0_6);
					v[2] = Volts_at_Pin(QFP32_MUX_P0_7);
					vsum = v[0] + v[2];
					
					if (vsum>=1.5)
					{
						printf ("Perimeter detected:v2.2=%10.8fV, v2.4=%10.8fV, flag = 1\n",v[0], v[2]);
						perimeterflag = 1;
					}
					else
					{
						// printf ("Perimeter not detected:v2.2=%10.8fV, v2.4=%10.8fV, flag = 0\n",v[0], v[2]);
						perimeterflag = 0;
					}
					if(perimeterflag == 0){
						action_flag = 1;
						motor_flag = 1;
						waitms(20);
						pulse_width=0.75;
						//send message to hc-05
						bluetooth_flag = 1;
						sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
						waitms(5); // This may need adjustment depending on how busy is the slave
						sendstr1(buff); // Send the test message
						
					}
					else if(perimeterflag ==1){
						while (vsum>1.5)// keep turning right until it leaves perimeter
						{
							action_flag = 1;
							motor_flag = 3;
							waitms(20);
							pulse_width=0.75;
							waitms(800);
							//send message to hc-05
							bluetooth_flag = 3;
							sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
							waitms(5); // This may need adjustment depending on how busy is the slave
							sendstr1(buff); // Send the test message
							v[0] = Volts_at_Pin(QFP32_MUX_P0_6);
							v[2] = Volts_at_Pin(QFP32_MUX_P0_7);
							vsum = v[0] + v[2];
						}
						
					}
					if(coinflag == 1){

						waitms(20);

						waitms(20);
						action_flag = 1;
						//send message to hc-05
						bluetooth_flag = 4;//pick up coins
						sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
						waitms(5); // This may need adjustment depending on how busy is the slave
						sendstr1(buff); // Send the test message

						motor_flag =2; //go backwards for .5 seconds
						waitms(10);
					pulse_width=0.7;
					waitms(260);
						//send message to hc-05
						bluetooth_flag = 2;//back
						sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
						waitms(5); // This may need adjustment depending on how busy is the slave
						sendstr1(buff); // Send the test message
						waitms(295);
						motor_flag =5;
						waitms(50);
						pulse_width=0;
						waitms(500);
						//send message to hc-05
						// bluetooth_flag = 0;//stop
						// sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
						// waitms(5); // This may need adjustment depending on how busy is the slave
						// sendstr1(buff); // Send the test message
						action_flag=0;
						servo_counter=0;
						waitms(20);
						P1_4=0;
						servo_up=50;
					waitms(500);
					servo_down=150;//
					waitms(500);
					for(j=150; j<110; j-=4) 
					{
						servo_down = j;   
						waitms(105);
					}


					servo_up=50;
					waitms(500);
					for(j=110; j<155; j+=3) 
					{
						servo_down = j;   
						waitms(100);
					}

					waitms(1000);
			
					printf("Servo has been initialized.\n");
						

					for(j=50; j<=240; j+=5) 
					{
						servo_up = j;   
						waitms(60);
					}
					//servo_up=240;
					// waitms(100);
					P1_4=1;
					waitms(1800);
					
					for(j=155; j<191; j+=4) 
					{
						servo_down = j;   
						waitms(80);
					}
					waitms(500);
			
					for(j=191; j>170; j-=2) 
					{
						servo_down = j;   
						waitms(80);
					}
					waitms(1000);

					//up: 240-55  185  down: 170-95  75

					for(j=0;j<=37;j++){
						servo_up = (240-j*5); 
						if(j>=22){
							servo_down = (170-(j-21)*5); 
							waitms(5);
						}
						waitms(20);
					}
					waitms(1000);
						P1_4=0;
						waitms(500);
						printf("coin number = %d\n", coin_count);
						coinflag = 0;
					}
						printf("go forward\n");
						action_flag = 1;
						motor_flag = 1;
						waitms(20);
						pulse_width=0.75;
						//send message to hc-05
						bluetooth_flag = 1;
						sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
						waitms(5); // This may need adjustment depending on how busy is the slave
						sendstr1(buff); // Send the test message
						distance = filtered_measure_distance();
				}
			}
		}
			coin_count = 0;
			//send message to hc-05
			bluetooth_flag = 0;//stop
			sprintf(buff, "%d\r\n", bluetooth_flag); // Construct a test message
			waitms(5); // This may need adjustment depending on how busy is the slave
			sendstr1(buff); // Send the test message
	}
	else if(get_voltagex ==  -40 && get_voltagey == -40){
		v[1] = Volts_at_Pin(QFP32_MUX_P2_4);
		m_voltage = v[1];
		m_voltage = m_voltage/0.986*(0.986+2.983);
		m_voltage = m_voltage * 10000;
		m_voltage_int = (long int) m_voltage;
		if(RXU1()) // Something has arrived
		{
			c=getchar1();
			
			if(c=='!') // Master is sending message
			{
				getstr1(buff, sizeof(buff)-1);
				if(strlen(buff)==7)
				{
					printf("%s\r\n", buff);
				}
				else
				{
					printf("*** BAD MESSAGE ***: %s\r\n", buff);
				}				
			}
			else if(c=='@') // Master wants slave data
			{
				sprintf(buff, "%ld\n", m_voltage_int);
				printf("send = %ld\n", m_voltage_int);
				waitms(20); // The radio seems to need this delay...
				sendstr1(buff);
				waitms(20);
				sendstr1(buff);
				waitms(20);
				sendstr1(buff);
				waitms(20);
				sendstr1(buff);
				waitms(20);
				sendstr1(buff);
			}
		
		}

		
	}
		else {
			//printf("Asked to move wheel, servo cannot be moved. \n");
			action_flag=1;
			if((get_voltagex > 150) &&(get_voltagex < 180) &&
			(get_voltagey > 150) && (get_voltagey < 180)){

				motor_flag = 5; //stop
				pulse_width=0;
				//printf("enter stop, flag:%d, pw:%.2f\n",motor_flag,pulse_width);
			}else if((get_voltagey > 180 && get_voltagey < 500)&& (get_voltagex > 150 && get_voltagex < 180)){
				speed=(float)(get_voltagey-180)/(350-180);
				motor_flag = 1; //forward
				pulse_width=0.75*speed;
				printf("enter forward, flag:%d, pw:%.2f\n",motor_flag,pulse_width);
			}else if((get_voltagey < 150 && get_voltagey > -1)&& (get_voltagex > 150 && get_voltagex < 180)){
				speed=(float)(170-get_voltagey)/(170);
				motor_flag = 2; //back
				pulse_width=0.75*speed;

				//printf("enter b, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}else if((get_voltagex < 500 && get_voltagex > 180) && (get_voltagey > 150 && get_voltagey < 180)){
				speed=(float)(get_voltagex-180)/(350-180);
				motor_flag = 3; //right
				pulse_width=0.75*speed;
				//printf("enter r, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}else if((get_voltagex < 180 && get_voltagex > -1) && (get_voltagey > 150 && get_voltagey < 180)){
				speed=(float)(170-get_voltagex)/(170);
				motor_flag = 4; //left
				pulse_width=0.75*speed;

				//printf("enter l, flag:%d, pw:%.2f\n",motor_flag,pulse_width);
				
			}else if((get_voltagex > 180 && get_voltagex < 500) && (get_voltagey > 180 && get_voltagey < 500)){
				speed=(float)(get_voltagey-180)/(350-180);
				motor_flag = 6;  //right forward
				pulse_width=0.75*speed;

				//printf("enter rf, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}else if((get_voltagex < 150 && get_voltagex > -1) && (get_voltagey > 180 && get_voltagey < 500)){
				speed=(float)(get_voltagey-180)/(350-180);
				motor_flag = 7; //left forward
				pulse_width=0.75*speed;
	
				//printf("enter lf, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}else if((get_voltagex > 180 && get_voltagex < 500)&& (get_voltagey < 150 && get_voltagey > -1)){
				speed=(float)(170-get_voltagey)/(170);
				motor_flag = 9; //right back
				pulse_width=0.75*speed;

				//printf("enter rb, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}else if((get_voltagex < 150 && get_voltagex > -1) && (get_voltagey < 150 && get_voltagey > -1)){
				speed=(float)(170-get_voltagey)/(170);	
				motor_flag = 8; //left back				
				pulse_width=0.75*speed;
				//printf("enter lb, flag:%d, pw:%.2f\n",motor_flag,pulse_width);

			}
				skip_servo = 0;
				auto_flag = 0;
			//printf("Motion of wheel has been set, waiting for next input. \n");
		}
		
		
	}
}
