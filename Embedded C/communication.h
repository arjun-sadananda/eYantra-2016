/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: communication.h
* Theme: Launch a Module
* Functions: uart0_init(void), ISR(USART0_RX_vect) 
* Global Variables: data , communication_array , communication_array_count , velocity_left_wheel , velocity_right_wheel  
*/


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

unsigned char data; //to store received data from UDR1
volatile unsigned int communication_array[30];//to store received data from UDR1 continuously
volatile unsigned int communication_array_count = 0;
unsigned char velocity_left_wheel;
unsigned char velocity_right_wheel;



/*
* Function Name: uart0_init
* Input: NONE
* Output: NONE
* Logic: Function To Initialize UART0
* Example Call: uart0_init();
*/
void uart0_init(void) {
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06; 
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hz set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


/*
* Function Name: ISR(interrupt)
* Input: Serial data from the XBee Module
* Output: NONE
* Logic: A serial interrupt which interrupts the program flow in the "main" Function
* Example Call: Called by Operating System when there is an interrupt
*
*/
ISR(USART0_RX_vect) 		    // ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	UDR0 = data;                //echo data back to PC
	
	if ( data == 0x67 ){        //HEX value of g
		
		// g stands for go with the given velocity. 
		// the array elements with index 1 and 5 indicate the sign of the velocity.
		// if velocity is positive wheel rotates in the forward direction.
		// if velocity is negative wheel rotates in the backward direction.  
		// communication_array[1] stores the sign of the velocity of left wheel.
		// communication_array[5] stores the sign of the velocity of right wheel. 
		// 11 stands for positive velocity. 13 for negative velocity.
		
		communication_array_count = 0;
		communication_array[0] = 1;  
		communication_array[1] = 11; //  both wheels are initialized to a positive velocity.
		communication_array[5] = 11; 
		communication_array_count++;
		return;
	}
	
	
	if((data >= 0x30 && data <= 0x39) || (data == 0x2B) || (data == 0x2D)){ // HEX value of 0 to 9 and + and -
		
		
		data = data & 0x0F; // extract only the 4 least significant bits.So 11 for '+' and 13 for '-'
		int velocity_digits = (int) data;
		communication_array[communication_array_count] = velocity_digits;
		communication_array_count += 1;
		if (communication_array_count == 8){ // if the array count reaches 8 the array count sets to 1. The velocities have to be given like this "g+200+200"
			
			velocity_left_wheel  = (communication_array[2]*100) + (communication_array[3]*10) + (communication_array[4]*1); //calculate velocity of left wheel
			velocity_right_wheel = (communication_array[6]*100) + (communication_array[7]*10) + (communication_array[8]*1); //calculate velocity of right wheel
			communication_array_count = 1;
			
			if((communication_array[1] == 11) && (communication_array[5] == 11) ){//default forward
				velocity(velocity_left_wheel,velocity_right_wheel);
				forward();
			}
			
			if((communication_array[1] == 11) && (communication_array[5] == 13) ){//right 
				velocity(velocity_left_wheel,velocity_right_wheel);
				right();
			}
			
			if((communication_array[1] == 13) && (communication_array[5] == 11) ){//left
				velocity(velocity_left_wheel,velocity_right_wheel);
				left();
			}
			
			if((communication_array[1] == 13) && (communication_array[5] == 13) ){//back
				velocity(velocity_left_wheel,velocity_right_wheel);
				back();
			}
			
			
		}
		return;
	}
	
	if(data == 0x70){  //HEX value of p: PICK the object
		communication_array[0] = 18;
		communication_array_count += 1;
		return;
	}
	if(data == 0x64){  //HEX value of d: DROP the object
		communication_array[0] = 19;
		communication_array_count += 1;
		return;
	}
	
	if(data == 0x65){  //HEX value of e: END 
		
		communication_array[0] = 20;
		_delay_ms(200);
		lifter_down();
		buzzer_on();
		_delay_ms(5000);
		buzzer_off();
		return;
	}
	
	if(data == 0x73){ //HEX value of s: START
		
		communication_array[0] = 21;
		return;
		
	}
}
#endif /* COMMUNICATION_H_ */