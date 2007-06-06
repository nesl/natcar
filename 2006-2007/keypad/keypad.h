/****Wires connections:*********
Keypad pin --------  MCU PORT.pin
   1         --->      PORT.0
   2         --->      PORT.1
   3         --->      PORT.2
   4         --->      PORT.3
   5         --->      PORT.4
   6         --->      PORT.5
   7         --->      PORT.6
   8         --->      PORT.7
   9         --->      GND
*******************************/

#ifndef KEYPAD_H
#define KEYPAD_H

//#include <mega128.h>
//#include <delay.h>

/**TO BE MODIFLIED BY USER!!!!!!*/
#define KEYPADPORT PORTA
#define KEYPADPIN  PINA
#define KEYPADDDR  DDRA          
/*******************************/        


#define KEYPAD_NOT_PRESS 255
#define KEYPAD_UP   	 100
#define KEYPAD_DOWN      101 
#define KEYPAD_SECOND    102
#define KEYPAD_ENTER     103
#define KEYPAD_HELP      104
#define KEYPAD_CLEAR     105

/*function prototypes*/
//The functions need to be called
void keypad_init();                     
int keypad_oneKey();
//Do not call the below functions if you have no idea about them
int keypad_internal_readKey();
int keypad_keyboardLayout(int position);
/********************/

void keypad_init()
{  
   KEYPADPORT=0xF0;
   //seting up direction. pin0-3 bit = out, pin4-7=in 
   KEYPADDDR=0x0F;
}                                                                         

int keypad_keyboardLayout(int position)
{                                      
   switch (position)
   {
      case 1:
      	return 1;
      case 2:
      	return 2;
      case 3:
      	return 3;
      case 4:
      	return KEYPAD_UP;
      case 5:
      	return 4;
      case 6:
      	return 5;
      case 7:
      	return 6;
      case 8:
      	return KEYPAD_DOWN;
      case 9:
      	return 7;
      case 10:
      	return 8;
      case 11:
      	return 9;
      case 12:
      	return KEYPAD_SECOND;
      case 13:
      	return KEYPAD_CLEAR;
      case 14:
      	return 0;
      case 15:
      	return KEYPAD_HELP;
      case 16:
      	return KEYPAD_ENTER;
   }   
   //impossible to get to this point if the input is correct!
   return KEYPAD_NOT_PRESS;
}

int keypad_oneKey()
{                                      
   int returnkey=KEYPAD_NOT_PRESS;
   //wait for input
   while (returnkey==KEYPAD_NOT_PRESS)
      returnkey=keypad_internal_readKey();
   //wait for releasing the key
   while (keypad_internal_readKey()!=KEYPAD_NOT_PRESS);
   return keypad_keyboardLayout(returnkey);
}

/**internal read key function, should not be called by user directly!
 *@return the key pressed; if no key pressed, return 255
*/
int keypad_internal_readKey()
{
   int returnkey=KEYPAD_NOT_PRESS;     
   
   //row1
   KEYPADPORT=0b11111110;
   //delay is required, physical stuff need time to setup
   delay_us(1);
   if (KEYPADPIN.4==0) returnkey=1;
   if (KEYPADPIN.5==0) returnkey=2;
   if (KEYPADPIN.6==0) returnkey=3;
   if (KEYPADPIN.7==0) returnkey=4;

   //row2
   KEYPADPORT=0b11111101;
   delay_us(1);
   if (KEYPADPIN.4==0) returnkey=5;
   if (KEYPADPIN.5==0) returnkey=6;
   if (KEYPADPIN.6==0) returnkey=7;
   if (KEYPADPIN.7==0) returnkey=8;
         
   //row3   
   KEYPADPORT=0b11111011;
   delay_us(1);
   if (KEYPADPIN.4==0) returnkey=9;
   if (KEYPADPIN.5==0) returnkey=10;
   if (KEYPADPIN.6==0) returnkey=11;
   if (KEYPADPIN.7==0) returnkey=12;
   
   //row4
   KEYPADPORT=0b11110111;
   delay_us(1);         
   if (KEYPADPIN.4==0) returnkey=13;
   if (KEYPADPIN.5==0) returnkey=14;
   if (KEYPADPIN.6==0) returnkey=15;
   if (KEYPADPIN.7==0) returnkey=16; 
   
   return returnkey;   
}
#endif