//
//
//
//
// MAGNETIC SENSOR MICROCONTROLLER
//
//
//
//

#include <mega128.h>
#include <delay.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "index.h"  
#include "lcd_library.h"
       
// SERVO
#define TOP 0x7FFF	// 32767
#define CENTER 2870 
#define RIGHT CENTER-725 // 2075
#define LEFT CENTER+725 // 3600

#define CONVERT_NUM 7
#define CONVERT_DENOM 10

#define XTAL 16000000
#define BAUD 9600

#define TWI_ADDR 0x02
#define TWI_SPEED_ADDR 0x01

#define NUM_SENS 2
#define MAX_SAMP 32

#define CENTER_SENSOR_INDEX 2
#define LEFT_SENSOR_INDEX 3
#define RIGHT_SENSOR_INDEX 0                                                 

#define LEFT_CENTER 0
#define RIGHT_CENTER 1
                                     
// this details which adc port to read from
int sensor_loc[NUM_SENS] = {1,2};          
                                              
#define NUM_SEGMENTS 7
// these three arrays will get set after the calibration phase
// heigh details the height of each of the fuzzy membership areas
// slope details the up slope and down slop of each membership area (note that the two slopes will be different in most cases)
// range details where each membership area is
float calibrated_height[NUM_SEGMENTS-2];
float calibrated_slope[NUM_SEGMENTS-2][2]; 
float calibrated_max_mem[NUM_SEGMENTS-2];
int calibrated_range[NUM_SEGMENTS];

// these are used for when the calibration is getting the far_left value and the far_right value
int right_extreme = 0;
int left_extreme = 0;
      
char keypad;
int have_keypad_val = 0;
 
// flag telling us if we have gotten the speed index yet or not
int have_speed_index = 0;   

// this details which sensor value we have after we take the std of the input
float sensor_value[NUM_SENS] = {0,0};
   
// indexes for the PID constant tables     
int sensor_index;
int speed_index;                      
               
// these are the P and D constant values that we can choose from
// for each of the tables, the first index is the sensor index
// so the indexes go as follows:
// 	0: far left
// 	1: mid left
// 	2: center
// 	3: mid right
// 	4: far right
//
// the second index ins for the speed value and go as follows:
// 	0: very slow
// 	1: slow
// 	2: average speed
// 	3: somewhat fast
// 	4: very fast
//
// so if you need to, change the values in each matrix with those index settings
// in mind
// this means that p_table[4][4] means the car is far to the right and going very fast.
// the p,d, and speed value should reflect that
int p_table[5][5] =   {{12, 10, 9, 10, 12},
                       {10, 9, 8, 9, 10},
                       {7, 7, 7, 7, 7}, 
                       {9, 7, 5, 7, 9},
                       {5, 4, 3, 4, 5}};

int d_table[5][5] =   {{2, 4, 7, 4, 2},
                       {4, 5, 6, 5, 4},
                       {6, 7, 8, 7, 6},
                       {9, 7, 8, 7, 9},
                       {12, 10, 9, 10, 12}};

// for this table the minimum value is 0 and the max value is 9
char speed_table[5][5] = {{6, 7, 9, 7, 6},
                          {4, 6, 9, 6, 4},
                          {3, 5, 9, 5, 3},
                          {2, 4, 9, 4, 2},
                          {1, 3, 9, 3, 1}};

                       
// we read the adc values into this array, and then after processing, save the value in sensor_value
int adc[NUM_SENS][MAX_SAMP];
//****************** USART COMMUNICATION ***********************
#define RXB8 1
#define TXB8 0
#define UPE 2
#define OVR 3
#define FE 4
#define UDR0E 5
#define RXC 7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDR0E)
#define RX_COMPLETE (1<<RXC)

// ***************** I2C PROTOCOL **********************
char Data_in ; //the data you receive
char index_in; //switch statement
char state;
char flag;
char temp_data;
char temp_addr;
char temp_index;


//****************** USART COMMUNICATION ***********************

//****************** USART 0 ***********************       

// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 32
unsigned char tx_buffer0[TX_BUFFER_SIZE0];
unsigned char tx_wr_index0,tx_rd_index0,tx_counter0;

// USART0 Transmitter interrupt service routine

interrupt [USART0_TXC] void uart0_tx_isr(void)
{

if (tx_counter0)
   {
   --tx_counter0;
   UDR0=tx_buffer0[tx_rd_index0];
   if (++tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0=0;
   };

}



void putchar0(unsigned char c)
{
while (tx_counter0 == TX_BUFFER_SIZE0);
#asm("cli")
if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer0[tx_wr_index0]=c;
   if (++tx_wr_index0 == TX_BUFFER_SIZE0) tx_wr_index0=0;
   ++tx_counter0;
   }
else UDR0=c;
#asm("sei")
}



//****************** USART 1 *********************** 
// USART1 Transmitter buffer
#define TX_BUFFER_SIZE1 32
char tx_buffer1[TX_BUFFER_SIZE1];
unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;

// USART1 Transmitter interrupt service routine

interrupt [USART1_TXC] void uart1_tx_isr(void)
{

if (tx_counter1)
   {
   --tx_counter1;
   UDR1=tx_buffer1[tx_rd_index1];
   if (++tx_rd_index1 == TX_BUFFER_SIZE1) tx_rd_index1=0;
   };

} 
void putchar1(char c)
{
while (tx_counter1 == TX_BUFFER_SIZE1);
#asm("cli")
if (tx_counter1 || ((UCSR1A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer1[tx_wr_index1]=c;
   if (++tx_wr_index1 == TX_BUFFER_SIZE1) tx_wr_index1=0;
   ++tx_counter1;
   }
else UDR1=c;
#asm("sei")
}
// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 835
unsigned char rx_buffer1[RX_BUFFER_SIZE1];
unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
// This flag is set on USART1 Receiver buffer overflow
bit rx_buffer_overflow1;

// USART1 Receiver interrupt service routine

interrupt [USART1_RXC] void uart1_rx_isr(void)
{
char status,data;

status=UCSR1A;
data=UDR1;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer1[rx_wr_index1]=data;
   if (++rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
   if (++rx_counter1 == RX_BUFFER_SIZE1)
      {
      rx_counter1=0;
      rx_buffer_overflow1=1;
      };
   };

}

unsigned char getchar1(void)
{
unsigned char data;
while (rx_counter1==0);
data=rx_buffer1[rx_rd_index1];
if (++rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
#asm("cli")
--rx_counter1;
#asm("sei")
return data;
}

// ***************** I2C PROTOCOL *********************

void TWI_init(char local_add)
{
	//pass in desire self_address as desired numbers, automatically masking.
	// 2 Wire Bus initialization
	// Generate Acknowledge Pulse: On
	// General Call Recognition: Off
	// Prescalar = 0x00
	SREG=0x80;
	flag=0;
	state= 0;
	Data_in=0;
	TWSR=0x00;           // no prescalar
	TWBR=0x0C;           //Bit rate
	TWAR=(local_add<<1); //slave address without gernal call enable
	// TWCR=0x44;        // inititate the TWI no interupts
	TWCR=0x45;           // inititate the TWI with interupts
}

void recieveData()
{       //slave receier mode only
        // return the index   Data_in Global variable
        char WINT ;// interrupt flag
        char WEA ;  //ENABLE ack bit
        char WSTA ;//start condition bit
        char WSTO ;//stop bit
        char WEN  ; //Enable activates TWI
        char WIE ;
        char Index  ;
        int loop = 0;
       
        WINT = 7;// interrupt flag
        WEA = 6;  //ENABLE ack bit
        WSTA = 5;//start condition bit
        WSTO= 4 ;//stop bit
        WEN =  2; //Enable activates TWI
        WIE = 0;  //Bit 1 don't care,bit 3 TWwC read only
                              
        TWCR=0X44;

        if (TWSR==0x60)                         // refer to a atmega128 page 221 for status code explaination
	{TWCR=(TWCR | (1<<WINT)|(1<<WEA));
	//terminate the I2C reciver mode if no addressed
	 
	
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}loop = 0;                //Wait for TWINT pull down to 0 i.e. done
      
        if (TWSR==0x80)
	{	Index=TWDR;
	        //index_in=Index;
		TWCR=(TWCR | (1<<WINT)|(1<<WEA));}
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}loop = 0;       
	if (TWSR==0x80)
	{	Data_in=TWDR;
	//PORTA=Data_in;
		TWCR=(TWCR | (1<<WINT)|(1<<WEA));}
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}
	loop = 0;
	if (TWSR==0xA0)
		{
		TWCR=(TWCR | (1<<WINT)|(1<<WEA)|(1<<WIE));}
            index_in=Index;
            
        //         TWCR=0x45;          // inititate the TWI with interupts
           //return the Data from transmitter
         }

}

int ERROR()
{
      //PORTA=0X77;
      return 0;
}

int ERROR1()
{
      //PORTA=0X66;
      return 0;
}

int ERROR2()
{
      //PORTA=0X58;
      return 0;
}

void sendData(char Index, char Data , char addr)
{
       //Master transmitter mode only

       char START;
       char MT_ADDR_ACK;
       char MT_DATA_ACK;
       char ARBIT_LOST;
       char REP_START;
       char WINT ;// interrupt flag
       char WEA ;  //ENABLE ack bit
       char WSTA ;//start condition bit
       char WSTO ;//stop bit
       char WEN  ; //Enable activates TWI
       char WIE ;
       int loop = 0;
       START=0x08;// Start Condition /w prescalar =00
       REP_START=0x10;// Repeat start condition

       MT_ADDR_ACK=0x18 ;//ADDRESS transmit ack status
       MT_DATA_ACK =0x28;//data transmit ack status
       ARBIT_LOST=0x38;
       
       TWCR=0X44;
       
       WINT = 7;// interrupt flag
       WEA = 6;  //ENABLE ack bit
       WSTA = 5;//start condition bit
       WSTO= 4 ;//stop bit
       WEN =  2; //Enable activates TWI
       WIE = 0;  //Bit 1 don't care,bit 3 TWwC read only

       // test code remove this while loop later
       TWCR = ((1<<WINT) |(1<<WSTA)|(1<<WEN))  ;  //send start condition
       
       
       while(!(TWCR&(1<<WINT))){ delay_ms(10);loop++;if(loop == 50)return;} 
       loop = 0;
                          //waiting TWINT = 0 ; start initiated check if bus avialable
       
       if ((TWSR & 0xF8)!= START)              //check bits 7-3 bits only  status of start transmitted
               {
                 temp_index=Index;
                 temp_data = Data;
                 temp_addr= addr;
                 //flag=1;
                  return;

                }
       TWDR = (addr<<1);               //Load the target address into TWDR Write=0 read=1 LSB

       TWCR = ( (1<<WINT) |(1<<WEN));       //send address
       while(!(TWCR&(1<<WINT))){delay_ms(10); loop++;if(loop == 50)return;}
loop = 0;
       if ((TWSR & 0xF8)!= MT_ADDR_ACK)//(check bits 7-3bits only  status of start transmitted
                {ERROR();}

       TWDR = Index;                          //Load Index
       TWCR = ((1<<WINT) |(1<<WEN));
            while(!(TWCR&(1<<WINT))){ delay_ms(10);loop++;if(loop == 50)return;} 
                          loop = 0;
       if ((TWSR & 0xF8)!= MT_DATA_ACK)//(check bits 7-3 bits only  status of start transmitted
                { ERROR1();}   


              //repeat start send 2nd byte

       TWDR = Data;
       TWCR = ((1<<WINT) |(1<<WEN));
            while(!(TWCR&(1<<WINT))){ delay_ms(10);loop++;if(loop == 50)return;}
            loop = 0;
       if ((TWSR & 0xF8)!= MT_DATA_ACK)//(check bits 7-3 bits only  status of start transmitted
                { ERROR2();}


       TWCR = ((1<<WINT) | (1<<WEN)| (1<<WSTO) | (1<<WIE) | (1<<WEA) );//stop
       //flag=0;
}

// this isn't used any more so ignore it
//
//************** DIGIPOT CONTROL ***************
// PORTB.0 == T0 CLOCK
// PORTB.1 == KILL

// The things you need to be concerned about.
// PORTE.6 == DATA
// PORTE.7 == CLOCK
// PORTE.4 == RESET_DP2
// PORTE.5 == RESET_DP1
void setRes(char a, int dp) {
	int j = 0;

	switch (dp){
	case 0:
		PORTE.4 = 1;
		PORTE.5 = 1;
		break;
	case 1:	
		PORTE.4 = 1;
		PORTE.5 = 0;
		break;
	case 2:
		PORTE.4 = 0;
		PORTE.5 = 1;
		break;
	default:
		PORTE.4 = 0;
		PORTE.5 = 0;
		return;
	}

	delay_ms(1);
	
	PORTE.6 = 0;
	delay_ms(1);
	PORTE.7 = 1;
	delay_ms(1);
	PORTE.7 = 0;
	delay_ms(1);

	for(j=0; j<8; j++) {
		PORTE.6 = ((a>>7) & 0x01);
		delay_ms(1);
		PORTE.7 = 1;
		delay_ms(1);
		PORTE.7 = 0;
		delay_ms(1);
		a <<= 1;
	}

	PORTE.4 = 0;
	PORTE.5 = 0;
	PORTE = 0;
}

// resistor value array.
char res[4];
interrupt [TWI] void TWI_interface(void) {
	recieveData();
	switch(index_in){
	// we use V to get the speed index from the speed mcu
	case 'V': 
		have_speed_index = 1;
		speed_index = Data_in;
		break; 
	case 'P':
		have_keypad_val = 1;
		keypad = Data_in;
		break;
	default : 
		break;
	}
	index_in = 0;
}

// ***** TIMER PARAMETERS *****
unsigned char wait = 0;
interrupt [TIM0_OVF] void timer0_overflow(void) {
	if(wait < 255) wait++;
}


// function for getting a character from the keypad, it requests a value from the speed mcu
char get_keypad_val(void){     
	int i = 0;
	have_keypad_val = 0;
	sendData('P', 0, TWI_SPEED_ADDR);
	
	while (!have_keypad_val)
	 {i++;if(i == 500)
	 return 0;
	 } 
	return keypad;
	
	
	// hack! to get around the lack of twi working
	//char key;
	//key = getchar1();
	//return key;
}

// ******************* INTERFACE FUNCTIONS ********************
char *mnu = "MENU\n\r1. calibrate \n\r2. GO! \n\r3. manual calibrate all\n\r4. choose speed\n\r5. toggle RC car driving\n\r6. show stats\n\r^K. STOP!\n\r";
char *done = "\n\rDONE!\n\r";
char *errmsg = "\n\rERROR!\n\r";
char *spchoice = "\n\rPlease choose a number [0 - 9].\n\r0 = stopped\n\r9 = full speed\n\r";
void printStr(char *st) {
	char i;
	for(i = 0; st[i] != 0; i++) {
		putchar0(st[i]);
	}
}
void showMenu(void) {
	printStr(mnu);
	//delay_ms(10);
}
void showDone(void) {
	printStr(done);
	//delay_ms(10);
}
void showError(void) {
	printStr(errmsg);
	//delay_ms(10);
}
void doSpeedChoice(void){
	char tempc;
	printStr(spchoice);
	do {
		tempc = getchar1();
		if(tempc < '0' || tempc > '9') {
			showError();
			printStr(spchoice);
		}
	} while(tempc < '0' || tempc > '9');
	tempc = (tempc - '0') * 2;
	sendData('S', tempc, TWI_SPEED_ADDR);
	tempc = 0;
	Data_in = 0;
	while(Data_in == 0 && tempc < 22) {
		tempc++;
		delay_ms(10);
	}
	if(Data_in == 0)
		showError();
	else
		showDone();
	//delay_ms(10);
}
    


/*
char *adst1 = "\n\rADC1 = ";
char *adst2 = "ADC2 = ";
char *adst3 = "ADC3 = ";
char *adst4 = "ADC4 = ";
char *admin = "MIN_ADC = {";
char *kpst = "KP_STEER = ";
char *kist = "KI_STEER = ";
char *kdst = "KD_STEER = ";
char *mytx = "TICK = ";
char *kpsp = "KP_TICKS = ";
char *kisp = "KI_TICKS = ";
char *kdsp = "KD_TICKS = ";    
*/
void showStats(void) {   
/*
	char str[8];
	
	printStr(adst1);
	itoa(adc[0][0], str);
	printStr(str);
	putchar0('\t');
	itoa(res[2], str); // res[2]
	printStr(str);
	putchar0('\n'); putchar0('\r');
	
	printStr(adst2);
	itoa(adc[1][0], str);
	printStr(str);
	putchar0('\t');
	itoa(res[3], str); // res[3]
	printStr(str);              
	putchar0('\n'); putchar0('\r');
	
	printStr(adst3);
	itoa(adc[2][0], str);
	printStr(str);
	putchar0('\t');
	itoa(res[0], str);
	printStr(str);              
	putchar0('\n'); putchar0('\r');
	
	printStr(adst4);
	itoa(adc[3][0], str);
	printStr(str);
	putchar0('\t');
	itoa(res[1], str); // res[1]
	printStr(str);              
	putchar0('\n'); putchar0('\r');
	
	printStr(admin);
	itoa(MIN_ADC[0], str);
	printStr(str);
	putchar0(',');
	itoa(MIN_ADC[1], str);
	printStr(str);
	putchar0(',');
	itoa(MIN_ADC[2], str);
	printStr(str);
	putchar0(',');
	itoa(MIN_ADC[3], str);
	printStr(str);
	putchar0('}');
	putchar0('\n'); putchar0('\r');	
	
	printStr(kpst);
	itoa(STEER_P, str);
	printStr(str);               
	putchar0('\n'); putchar0('\r');
	
	printStr(kist);
	itoa(STEER_I, str);
	printStr(str);               
	putchar0('\n'); putchar0('\r');
	
	printStr(kdst);
	itoa(STEER_D, str);
	printStr(str);
	putchar0('\n'); putchar0('\r');
		
	printStr(mytx);               
	putchar0('\n'); putchar0('\r');

	printStr(kpsp);               
	putchar0('\n'); putchar0('\r');

	printStr(kisp);               
	putchar0('\n'); putchar0('\r');

	printStr(kdsp);
	putchar0('\n'); putchar0('\r');
	*/
	//delay_ms(10);
}

char *smv1 = "\n\rEnter values for digipots 1-4. [0 - 255]\n\r";
void setManualValues(void) {           
/*
	char tc[5], j, i, tempc;

	printStr(smv1);

	for(j = 0; j < NUM_SENS; j++) {
		for(i = 0; i < 5; i++) {
			tc[i] = getchar0(1);
			if(tc[i] == 0 || tc[i] == '\n' || tc[i] == '\r' || i == 4) {
				tc[i] = 0;
				break;
			}
		}
		if(j != NUM_SENS-1) getchar0(1);
		tempc = (char)atoi(tc);
		switch(j) {
		case 0 :res[2] = tempc;
			break;
		case 1 :res[3] = tempc;
			break;
		case 2 :res[0] = tempc;
			break;
		case 3 :res[1] = tempc;
			break;
		default:
			break;
		}
		//res[j] = tempc;
	}
	// Set digipot 1.
	//setRes(res[0], res[1], 0);
	// Set digipot 2.
	//setRes(res[2], res[3], 1);
	showDone();
	*/
}

char *killmsg = "\n\rKILLED!\n\r";
void showKilled() {
	printStr(killmsg);
}
 
  
int get_calibrated_readings(int sensor_num){
	int calibrated_adc[NUM_SENS][50];
	float sensor_value[NUM_SENS];
	int i,j;
	 
	for(j = 0; j<50; j++) {
		for(i = 0; i<NUM_SENS; i++) {
			// Using 2.56V internal reference
			ADMUX = (0xC0 | sensor_loc[i]);
			ADCSRA = 0xC4;	// 1100 0100	// Enable and start, division factor by 16.
			while((ADCSRA & 0x40) != 0x00);	// Wait for conversion.
			calibrated_adc[i][j] = ADCW;
			//dataCycles++;
		}
	} 
	
	for (i = 0; i < NUM_SENS; i++){
		float average = 0;
		float variance = 0;
		
		for (j = 0; j <MAX_SAMP;j++)
			average += calibrated_adc[i][j];
		
		average /= (float) MAX_SAMP;
		
		for (j = 0; j < MAX_SAMP; j++)
			variance += (calibrated_adc[i][j] - average)*(calibrated_adc[i][j] - average);
		
		variance /= (float) MAX_SAMP;
		
		sensor_value[i] = sqrt(variance);
	}
	
	// for the two extreme range values save the raw sensor value of the opposite sensor
	if (sensor_num == 0)
		right_extreme = sensor_value[RIGHT_CENTER];
	else if (sensor_num == NUM_SEGMENTS-1)
		left_extreme = sensor_value[LEFT_CENTER];

	return sensor_value[LEFT_CENTER] - sensor_value[RIGHT_CENTER];
}
		                         
char *far_left = "place to the far left\n\r";
char *mid_left = "place to the mid left\n\r";
char *close_left = "place to the near left\n\r";
char *center = "place in the center\n\r";
char *close_right = "place to the near right\n\r";
char *mid_right = "place to the mid right\n\r";
char *far_right = "place to the far right\n\r"; 
   
char *calibrate_str = "press 0 to calibrate\n\r";
void calibrate(void){ 
	// this is terrible code, but code vision is a shitty compilier
	char input; 
	   
	// first get the calibrated readings                                     
	printStr(far_left);
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[0] = get_calibrated_readings(0);
        
        printStr(mid_left);
        lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[1] = get_calibrated_readings(1);
	
	printStr(close_left);          
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[2] = get_calibrated_readings(2);
	
	printStr(center);              
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[3] = get_calibrated_readings(3);
	
	printStr(close_right);         
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[4] = get_calibrated_readings(4);
	
	printStr(mid_right);           
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);             
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[5] = get_calibrated_readings(5);
	
	printStr(far_right);           
	lcd_display_text(0,0,far_left);
	do {
		printStr(calibrate_str);
		lcd_display_text(0,1, calibrate_str);
		
		input = get_keypad_val();
	} while (input != '0');
	calibrated_range[6] = get_calibrated_readings(6);
	
	// now process them and get the calibrated slope, height, and max membership
	get_calibrated_settings(calibrated_range, calibrated_slope, calibrated_height, calibrated_max_mem);
	
}
	
void checkForUpdates(void) {
	char tempMC;
	tempMC = getchar1();
	switch(tempMC) {
	//case 'w' :
		// increase speed
	//	sendData('V', 'U', TWI_SPEED_ADDR);
	//	break;
	//case 's' :
		// decrease speed
	//	sendData('V', 'D', TWI_SPEED_ADDR);
	//	break;
	case '3' :
		setManualValues();
		break;
	case '5' :
		//rcdrive = 1 - rcdrive;
		//putchar0('\n'); putchar0('\r');
		//putchar0('r'); putchar0('='); putchar0('0'+rcdrive);
		//putchar0('\n'); putchar0('\r');
		//showDone();
		break;
	case '6' :
		showStats();
		break;
	case 'K' : //case '\r' :
		sendData('K', 'K', TWI_SPEED_ADDR);
		showKilled();
		break;
	default:
		;
	}
}     

void getSpeedIndex(void){
	sendData('V', 0, TWI_SPEED_ADDR);
}                       
     
char *lcd_option_1 = "1. Calibrate";
char *lcd_option_2 = "2. Go";
void show_LCD_menu(void){
       lcd_display_text(0,0, lcd_option_1);
       lcd_display_text(0,1, lcd_option_2);
       
}

void main(void)
{
	char tempMC, keypad_val;
	char new_speed; 
	int i,j;  
	int turn, old_turn = 0; 
	float distance; 
	int P, D;
	float error = 0, error1 = 0, error2 = 0;
	unsigned int UBR;
	// According to...
	//	setRes(res[0], res[1], 0);
	//	setRes(res[2], res[3], 1);
	// The resistors correspond to these sensors:
	//	res[0] == sensor 1
	//	res[1] == sensor 2
	//	res[2] == sensor 4
	//	res[3] == sensor 3
	res[0] = 200;
	res[1] = 200;
	res[2] = 200;
	res[3] = 200;
	DDRE = 0xF0; // 1111 0000
	
	DDRD = 0xFF;
	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: Off
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// USART0 Baud rate: 19.2k
	/*UCSR0A=0x00;
	UCSR0B=0xD8; // 1101 1000
	UCSR0C=0x06; // 0000 0110
	UBRR0H=0;
	UBRR0L=51;
	*/
	
        UBR = XTAL/(16*BAUD) -1; 
        // USART0 initialization
	UBRR0H=(unsigned char)(UBR >>8);                
	UBRR0L=(unsigned char)(UBR &0xFF); 
    	UCSR0A=0x00;              
	UCSR0B=0xD8;
	UCSR0C=0x06;
	
	// SERVO PARAMETERS
	DDRB = 0x20; // 0010 0000
	TCCR1A=0x82; // 100000 10
	TCCR1B=0x1A; // 000 11 010
	ICR1H = TOP>>8; // UPPER 8 bits
	ICR1L = TOP & 0xFF; // LOWER 8 bits
	OCR1A = CENTER;
	
	// TIMER
	TCCR0 = 0x07; // 0000 0111
	
	// INTERRUPT REGISTERS
	TIMSK = 0x01;
	
	// Set digipot 1.
	//setRes(res[0], res[1], 0);
	// Set digipot 2.
	//setRes(res[2], res[3], 1);
	
	TWI_init(TWI_ADDR);	
	#asm("sei");

	// SHOW MENU BEFORE ANYTHING STARTS
	do { 		
		showMenu();    
		//show_LCD_menu();
		//delay_ms(1000);
		//tempMC = getchar0(0); 
		//for now just run everything off the keypad
		keypad_val = get_keypad_val();
	       tempMC = keypad_val;		
		switch(tempMC) {
		case '1' : 
			calibrate();
			break;
		case '2' :  
			showDone(); showDone();
			break;
		case '3' :
			setManualValues();
			break;
		case '4' :
			doSpeedChoice();
			break;
		case '5' :
			//rcdrive = 1 - rcdrive;
			//p/utchar0('\n'); putchar0('\r');
			//putchar0('r'); putchar0('='); putchar0('0'+rcdrive);
			//putchar0('\n'); putchar0('\r');
			//showDone();
			break;
		case '6' :
			showStats();
			break;
		case 'K' : //case '\r' :
			sendData('K', 'K', TWI_SPEED_ADDR);
			showKilled();
			break;
		default:
			;
		}
	} while(tempMC != '7');
	
	//going = 1;
	
	while(1) {
		checkForUpdates();   
		    
		have_speed_index = 0;
		getSpeedIndex();
		
		// read MAX_SAMP sensor readings from the designated adc ports for each sensor
		for(j = 0; j<MAX_SAMP; j++) {
			for(i = 0; i<NUM_SENS; i++) {
				// Using 2.56V internal reference
				ADMUX = (0xC0 | sensor_loc[i]);
				ADCSRA = 0xC4;	// 1100 0100	// Enable and start, division factor by 16.
				while((ADCSRA & 0x40) != 0x00);	// Wait for conversion.
				adc[i][j] = ADCW;
				//dataCycles++;
			}
		}
		    
		// according to jon lau, we just take the standard deviation of the sensor values read to determine the actual value
		for (i = 0; i < NUM_SENS; i++){
			float average = 0;
			float variance = 0;
			
			for (j = 0; j <MAX_SAMP;j++)
				average += adc[i][j];
			
			average /= (float) MAX_SAMP;
			
			for (j = 0; j < MAX_SAMP; j++)
				variance += (adc[i][j] - average)*(adc[i][j] - average);
			
			variance /= (float) MAX_SAMP;
			
			sensor_value[i] = sqrt(variance);
		}	
	          
		// honestly, i have no idea what this is for?
		//if(rcdrive == 1) continue;
		
		// this tells us how far we are away from the line, positive means to the right, and negative means to the left		           
		distance = sensor_value[LEFT_CENTER] - sensor_value[RIGHT_CENTER];
		
		// determine the index for the new constants
	        sensor_index = getIndex(distance, calibrated_range, calibrated_slope, calibrated_height, calibrated_max_mem);
		     
		// we need to wait for the speed index value before proceeding
		while (!have_speed_index)
			;
		    
		// get the new constants we want
		P = p_table[sensor_index][speed_index];
		D = d_table[sensor_index][speed_index];

		// send the new speed value to the speed mcu
		new_speed = speed_table[sensor_index][speed_index];
		sendData('S', new_speed, TWI_SPEED_ADDR);
		                                
		// compute the error this is the center range value - the current distance
		error = calibrated_range[3] - distance;
		                    
		// get the new turning value for the servo
		turn = old_turn + (P + D)*error - (P + 2*D)*error1 + D*error2;
		                                              
		turn = (CONVERT_NUM * turn) / (P * CONVERT_DENOM);
		// save the new values for future computations
		old_turn = turn;
		error2 = error1;
		error1 = error;
		
		// set the new turn value for the pwm
		if((turn+CENTER) > LEFT) OCR1A = LEFT;
		else if((turn+CENTER) < RIGHT) OCR1A = RIGHT;
		else OCR1A = turn+CENTER;
		       
				
		while(wait < 1) checkForUpdates();
		wait = 0;
	};
}
