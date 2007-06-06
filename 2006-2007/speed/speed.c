#include <mega128.h>
#include <delay.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "keypad.h" 
#include "index.h"

#define XTAL 16000000
#define BAUD 9600
void processcamera(void);
void getstuff(void);
//**************** TIMER PARAMETERS ************************
#define Kp 5 //vary from 1-15?
#define H2 OCR1B  
#define H1 OCR1A 
#define PWMFREQ 2000  
  
#define TOP XTAL/PWMFREQ
//pwm freq = xtal / top  = 2khz
//duty cycle = ocr1/ top
#define MED 1250
#define MIN 1000
#define SPEEDCAP 3000    //>2500
// WHEEL DIAMETER == 2.48"    
signed long int myticks;
signed long int ticks;
signed long int speed;
signed long int maxspeed = 10, topspeed;
unsigned char g_speed=3, g_dev = 0;

#define NUM_SEGMENTS 7
// these will be calibrated based on the range value in the main function
// don't worry about them
float speed_height[NUM_SEGMENTS-2];
float speed_slope[NUM_SEGMENTS-2][2];
float speed_max_mem[NUM_SEGMENTS-2];

// encoder tick range
int speed_range[] = {0,500,1000,1500,2000,2500,SPEEDCAP};
unsigned char testvar; 
char state = 0;

interrupt [TIM0_OVF] void timer0_overflow(void) {
//used to update the pwm signal
//counts from 0 to 255
//overflows
//counts from 255 to 0
//repeats

testvar++;

	ticks = TCNT3L;
	ticks |= (0xFFl & TCNT3H) << 8;
	ticks*=4;    
	if(ticks> SPEEDCAP ) 
        {   
           ticks = SPEEDCAP; 
        }
	TCNT3H = 0;
	TCNT3L = 0;
}
void updatespeed (void)
{
    myticks = (TOP/4)*g_speed;
 	topspeed = maxspeed*(TOP/10); 
 	if(myticks > topspeed)
		myticks = topspeed; 
		
        if(myticks == TOP && ticks > myticks)
        {
        	speed = 0;
        }
        else{
         	speed = (Kp*(myticks-ticks))/5; 
         }
         	if(speed > myticks)
		speed = myticks;
        
/*   
   
  
if(state == 0 &&maxspeed <=6)
state = 1;
else if(state == 1 &&maxspeed>= 8)
state = 0;
if(state == 0)
{
     topspeed = TOP;    
}
else if(state ==1)
{    
     topspeed = ((MED -MIN )/7)*maxspeed + MIN; 
}

     speed = (Kp*(topspeed-ticks))/5; 
*/



	// forward
	if(speed > 0) {
		if(speed > TOP) speed = TOP;  //ie high
		H1 = TOP-speed;
		H2 = 0xFFFF;         //low since will never count above top   
	}
	// reverse
	else {
		if(speed < -TOP) speed = -TOP;//ie high
       		H1= 0xFFFF;          //low since will never count above top
		H2 =  TOP+speed;    
	}
return;

}

//****************** USART COMMUNICATION ***********************
#define RXB8 1
#define TXB8 0
#define UPE 2
#define OVR 3
#define FE 4
#define UDRE 5
#define RXC 7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

//****************** USART 1 *********************** 
// USART1 Transmitter buffer
#define TX_BUFFER_SIZE1 32
char tx_buffer1[TX_BUFFER_SIZE1];
unsigned char tx_wr_index1 =0,tx_rd_index1=0,tx_counter1=0;

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
unsigned int rx_wr_index1=0,rx_rd_index1=0,rx_counter1=0;
// This flag is set on USART1 Receiver buffer overflow
char rx_buffer_overflow1;

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
if(rx_counter1==0)return 0;
data=rx_buffer1[rx_rd_index1];
if (++rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
#asm("cli")
--rx_counter1;
#asm("sei")
return data;
}

void printstr1(char* st)
{
	int i;
	for(i = 0; st[i]!= 0; i++)
	{
		putchar1(st[i]);
	}
}  

// ***************** I2C PROTOCOL **********************
#define TWI_ADDR 0x01
#define TWI_SENSOR_ADDR 0x02


char i2ckilled = 0;
char Data_in = 0; //the data you receive
char index_in = 0; //switchment
char state_question;
char flag;
char temp_data;
char temp_addr;
char temp_index;
char keypad = 0;

void TWI_init(char local_add)
{
	//pass in desire self_address as desired numbers, automatically masking.
	// 2 Wire Bus initialization
	// Generate Acknowledge Pulse: On
	// General Call Recognition: Off
	// Prescalar = 0x00
	SREG=0x80;
	flag=0;
	state_question= 0;
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
	 
	
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}loop = 0;              //Wait for TWINT pull down to 0 i.e. done
      
        if (TWSR==0x80)
	{	Index=TWDR;
	        //index_in=Index;
		TWCR=(TWCR | (1<<WINT)|(1<<WEA));}
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
	        
	if (TWSR==0x80)
	{	Data_in=TWDR;
	//PORTA=Data_in;
		TWCR=(TWCR | (1<<WINT)|(1<<WEA));}
	while(!(TWCR&(1<<WINT))){loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
	
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

       TWCR = ((1<<WINT) |(1<<WSTA)|(1<<WEN))  ;  //send start condition
       while(!(TWCR&(1<<WINT))) {loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
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
       while(!(TWCR&(1<<WINT))){ loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
       if ((TWSR & 0xF8)!= MT_ADDR_ACK)//(check bits 7-3bits only  status of start transmitted
                {ERROR();}

       TWDR = Index;                          //Load Index
       TWCR = ((1<<WINT) |(1<<WEN));
            while(!(TWCR&(1<<WINT))){ loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
       if ((TWSR & 0xF8)!= MT_DATA_ACK)//(check bits 7-3 bits only  status of start transmitted
                { ERROR1();}   


              //repeat start send 2nd byte

       TWDR = Data;
       TWCR = ((1<<WINT) |(1<<WEN));
            while(!(TWCR&(1<<WINT))){ loop++;delay_ms(10);if(loop == 50)return;}loop = 0;
       if ((TWSR & 0xF8)!= MT_DATA_ACK)//(check bits 7-3 bits only  status of start transmitted
                { ERROR2();}


       TWCR = ((1<<WINT) | (1<<WEN)| (1<<WSTO) | (1<<WIE) | (1<<WEA) );//stop
       //flag=0;
}
void getstuff(void)
{    
maxspeed = getchar1();
}
void getSpeedIndex(void){
	char speed_index;
	speed_index = getIndex(ticks, speed_range, speed_slope, speed_height, speed_max_mem);
        putchar1(speed_index);
	//sendData('V', speed_index, TWI_SENSOR_ADDR);
}

interrupt [TWI] void TWI_interface(void) {
	recieveData();
	switch(index_in) {
	case 'K' :
		i2ckilled = 1;
		index_in = 0;
		break;
	case 'S' :
	        if(Data_in > 10)
	        Data_in = 10;
	        else if(Data_in < 0)
	        Data_in = 0;  
	        maxspeed = Data_in;   
	        index_in = 0;
		break;
	case 'V' :
		getSpeedIndex();	
		index_in = 0;
		break;   
	case 'P' : 
	
		//sendData('P', keypad, TWI_SENSOR_ADDR); 
		 
		//keypad = keypad_oneKey();    //polling
		//sendData('P', keypad, TWI_SENSOR_ADDR);
		index_in = 0;
		break;
	default:
		break;
	}
}



//**************** CAMERA FUCTIONS ************************
#define CAMWINX 88
#define CAMWINY 72 
#define XRES 88 
#define THRESHOLD 100
char* RST = "RS\r"; 
char* YCrCb = "CR 18 32\r";
char* AG = "CR 19 32\r"; 
char* NF = "NF 2\r";
char* LINMOD = "LM 0 2\r";
char* DS = "DS 1 2\r";
char* TRACKCYCrCb = "TC 85 160 100 255 0 50\r";

unsigned char xmeans[143];
void initcamera(void)
{      
printstr1(RST);
delay_ms(10);   
printstr1(YCrCb);
delay_ms(10);  
printstr1(AG);
delay_ms(10);  
printstr1(NF);
delay_ms(10); 
printstr1(LINMOD);
delay_ms(10);
printstr1(DS);
delay_ms(10); 
printstr1(TRACKCYCrCb);
}    

void calcstats(int ysize)
{ 
	//Find slope of xmeans
	long int slope = 0;
	unsigned long int mean=0;
	int count = 0;
	int cline;
	unsigned long int devsquare=0;
	int speedg=0;
	long int temp = 0;
	int temp0;
	int temp1;
	int temp2;
	for(cline=1; cline<ysize; cline++)
	{
		if(xmeans[cline]!=255 && xmeans[cline-1]!=255)
		{
			temp1 = xmeans[cline];
			temp2 = xmeans[cline-1];
			temp = temp1-temp2;
			slope += temp;
			mean += xmeans[cline];
			count++;
		}
	}

	if(count!=0)
	{       
		temp = slope*100;
		slope = temp/count;
	}
	else
		slope = 0;

	if(count!=0)
		mean = mean/count;
	else
		mean = 0;

	//Find deviation squared
	for(cline=1; cline<ysize; cline++)
	{
		if(xmeans[cline]!=255 && xmeans[cline-1]!=255)
		{
			temp0 = mean-xmeans[cline];
			temp = temp0*temp0;
			devsquare += temp;
		}
	}

	if(count!=0)
		devsquare = devsquare/count;
	else
		devsquare = 0;
		speedg=1;

		if(count>=0 && devsquare<300)
		speedg=2;

	 	if(count>=0 && devsquare <=100)
		speedg=3;

		if(count>=0 && devsquare<=10)
		speedg=4;

	g_speed=speedg;
	g_dev = devsquare;
}

void processcamera()
{  //keypad= getchar1();
 
          


		int ysize=0;
		int y=0;
		int xmean=0;
		int xmin=0;
		int xmax=0;
		int xcnt=0;
		int xconf=0;
		int i = 0;
			
		//Find packet start
		while(getchar1()!=0xFE && i < 25) //Wait for packet start
		{
			//count how many times we loop, time out and reinit if need be
			i++;
		} 
		
  		if(i >= 25)
  		{
  		g_speed = 3;
  		initcamera();
  		return;
  		}
		ysize = getchar1();
		
		//For each line
		for(y=0; y<ysize; y++)
		{
			xmean = getchar1();
			xmin  = getchar1();
			xmax  = getchar1();
			xcnt  = getchar1();
			xconf = getchar1();

			if(xconf > THRESHOLD)
				xmeans[y] = xmean; //change from mean to center if needed
			else
				xmeans[y]=255;     
		     	for(i=0; i<XRES; i++)
			{
				char printchar = ' ';
				if(i>=xmin&&i<=xmax)
					printchar = 'X';
				if(i==xmean)
					printchar = '|';
				if(xconf < THRESHOLD)
					printchar = ' ';
				printf("%c",printchar);
			}
		printf("\r");   
		}
		calcstats(ysize);
}  

void main(void)
{    
        unsigned int UBR = XTAL/(16*BAUD) -1; 
        // USART0 initialization
	UBRR0H=(unsigned char)(UBR >>8);                
	UBRR0L=(unsigned char)(UBR &0xFF); 
    	UCSR0A=0x00;              
	UCSR0B=0xD8;
	UCSR0C=0x06;            // 0000 0110 character size reserved, asynchronous operation
	
	// USART1 initialization
	UBRR1H=(unsigned char)(UBR >>8);                    
  	UBRR1L=(unsigned char)(UBR & 0xFF);             
    	UCSR1A = 0x00;          //normal data rate for asynchronous operation
	UCSR1B=0xD8;            // 1101 1000 enable transmitter, reciever, interupts on transmit and recieve complete
	UCSR1C=0x06;            // 0000 0110 character size reserved, asynchronous operation     

	// SPEED PWM - HBRIDGE CONTROL
	//B 5,6 output compare
	//B 1 SPI clk
	DDRB = 0x60; // 0110 0000
	
	// TIMER INTERRUPT
	//-11- --00
	//outputcompare/timer overflow 
	TIMSK = 0x01; // 0000 0001 
	
	// TIMER 0
	//TCCR
	//interrupt to update pwm
	//---- -xxx
	//0: stopped, 1: no scaling, 2: 8, 3: 32, 4: 64, 5:128, 6: 256, 7: 1024
	//-000 0---
	//no output compare, overflow on max
	TCCR0 = 0x07; // 0000 0111    
	
 	// TIMER 1
	// SPEED PWM - HBRIDGE CONTROL
	//TCCRA: AABB CC--
	//Overflow at top
	//Clear OCR1 on compare match
	
	//TCCRA: ---- --xx TCCRB: ---xx---
	//Fast PWM
	//Top = ICR1
	//Update OCR1 at top

    	//TCCRB: ---- -xxx 
	//0:stopped 1:no prescaling 2: 8 3:64 4:256 5:1024 6,7: external clock
	TCCR1A = 0xF2; // 111100 10
	TCCR1B = 0x19; // 00011 001
	ICR1H = TOP >> 8;
	ICR1L = TOP & 0xFF;
	
    	// TIMER 3
	// ENCODER TICKS COUNTER
	//---- -xxx
	//6: clock on Tn pin rising 7: Clock on Tn pin falling
	TCCR3A = 0x00;
	TCCR3B = 0x07;
    
    //EIMSK = 0b11000000;
    //EICRA = 0b00001000;
    
	//TWI_init(TWI_ADDR);
	initcamera();
	//keypad_init();    
	
	get_calibrated_settings(speed_range, speed_slope, speed_height, speed_max_mem);

	#asm("sei");
	while(1) 
	{
        getstuff();
        getSpeedIndex();
        updatespeed();
        
		if(testvar ==30)
  		{       
  			//printf("\rstate: %d maxspeed: %d ticks: %d topspeed: %d\r", state,maxspeed, ticks, topspeed);    
  			printf("keypad: %d",keypad);
                       	printf("index_in: %d Data_in: %d\r", index_in,Data_in);
                       	testvar =0;
     		}          
	};
}
