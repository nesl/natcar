#include <math.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
      
void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
	while( EECR & 0xF2)
	 ;

	EEAR = uiAddress;                    
	EEDR = ucData;
	EECR |= 0xF4;
	EECR |= 0xF2;
}

void main(){
	int readings[2][20];
	int i,j;
	int sensor_index[] = {0, 7};   
	int sensor_val[2];
	float distance;  
	int turn, old_turn = 0;
	float error, error1 = 0, error2 = 0;
	
	float average;
		
	float std;
	unsigned int mem = 0;
	char str[8];    
	int k = 0;
	
	DDRE = 0x00;
	DDRD = 0xF0;
	
	                                   
	DDRC = 0xFF;
	
	//setRes(10, 0);
	
	while(1){
		PORTC = 0x00;                             
	        
	        _delay_ms(1000);
	        
	        PORTC = 0xFF;
	        
	        _delay_ms(1000);
	        
		/*
		if (!(PINE & 0x01)){
		        for (j = 0; j < 20;j++){
				for (i = 0; i< 2; i++){
					ADMUX = (0xC0 | sensor_index[i]);
					ADCSRA = 0xC4;
 					while((ADCSRA & 0x40) != 0x00);	// Wait for conversion.
						readings[i][j] = ADCW;
				}
			}
			average = 0;
			for(j = 0; j< 2;j++){
			        average = 0;
			           
				for (i = 0; i< 20; i++){
					average += readings[j][i];
				}
			
		    		average /= 20;
			
				std = 0;      
			        for (i = 0; i<20;i++){
			        	std += (readings[j][i] - average) * (readings[j][i] - average);
			        } 		        		 
		        	sensor_val[j] = sqrt(std / 20.0);
	                 }
	                 distance = sensor_val[0] - sensor_val[1];
	                 
	                 
	                 itoa(sensor_val[0], str);
	                 k = t0;
	                 while (str[k] != 0){
	                 	EEPROM_write(mem++, str[k]);
	                 	k++;
	                 }
	                 EEPROM_write(mem++, ' ');
	                          	                  
	                 itoa(sensor_val[1], str);
	                 k = 0;
	                 while (str[k] != 0){
	                 	EEPROM_write(mem++, str[k]);
	                 	k++;
	                 }
	                 EEPROM_write(mem++, '\t');
	                          	                                 
	                 error = -distance;
	                 turn = old_turn + (7+9)*error - (7 + 18)*error1 + 9*error2;
	                 
	                 itoa(turn, str);
	                 k = 0;
	                 while (str[k] != 0){
	                 	EEPROM_write(mem++, str[k]);
	                 	k++;
	                 }
	                 
	                 EEPROM_write(mem++, '\n');
	                 
	                 error2 = error1;
	                 error1 = error;
	                 old_turn = turn; 
	                 
	                 // if turn is positive then we will turn right
	                 // if turn is negative then we will turn left
	                 if (distance > 0.0)
	                 	PORTC = 0xFE;
	                 else if (distance < 0.0)
	                 	PORTC = 0xEF;
	                 else
	                 	PORTC = 0xEE;
	                 	
	                 	
	                 delay_ms(1500);
	                 		      
		} */
		
	}
	
}		 
	
	
