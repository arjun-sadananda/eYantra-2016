
/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: motor.h
* Theme: Launch a Module
* Functions: PWM_timer_init() , motion_pin_config() , motion_set() , velocity() , forward() , back() , right() , left() , soft_left() , soft_left() , *			soft_right_2() , soft_left_2() , stop()
* Global Variables: NONE
*/

#ifndef MOTOR_H_
#define MOTOR_H_

/*
* Function Name: PWM_timer_init
* Input: NONE
* Output: NONE
* Logic: Timer 5 initialized in PWM mode for velocity control
* Example Call: PWM_timer_init();
*/

void PWM_timer_init(){
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function Name: motion_pin_config
* Input: NONE
* Output: NONE
* Logic: Configure of PORTA for direction and PORTL for velocity 
* Example Call: motion_pin_config();
*/
void motion_pin_config (void){
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name: velocity
* Input: velocity of left motor , velocity of right motor
* Output: NONE
* Logic: Function for robot velocity control
* Example Call: velocity(200,202);
*/

void velocity (unsigned char left_motor, unsigned char right_motor){
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
/*
* Function Name: motion_set
* Input: HEX value corresponding to which motors you want to run.  
* Output: NONE
* Logic: Function used for setting motor's direction
* Example Call: motion_set(0x08);
*/

void motion_set (unsigned char Direction){
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}


/*
* Function Name: forward
* Input: NONE
* Output: NONE
* Logic: both wheels forward
* Example Call: forward();
*/
void forward (void)		{	motion_set(0x06);	}


/*
* Function Name: back
* Input: NONE
* Output: NONE
* Logic: both wheels backward
* Example Call: back();
*/
void back (void)		{	motion_set(0x09);	}


/*
* Function Name: left
* Input: NONE
* Output: NONE
* Logic: Left wheel backward, Right wheel forward
* Example Call: left();
*/
void left (void)		{	motion_set(0x05);	}


/*
* Function Name: right
* Input: NONE
* Output: NONE
* Logic: Left wheel forward, Right wheel backward
* Example Call: right();
*/
void right (void)		{	motion_set(0x0A);	}


/*
* Function Name: soft_left
* Input: NONE
* Output: NONE
* Logic: Left wheel stationary, Right wheel forward
* Example Call: soft_left();
*/
void soft_left (void)	{	motion_set(0x04);	}


/*
* Function Name: soft_right
* Input:  NONE
* Output: NONE
* Logic: Left wheel forward, Right wheel is stationary
* Example Call: soft_right();
*/
void soft_right (void)	{	motion_set(0x02);	}


/*
* Function Name: soft_left_2
* Input:  NONE
* Output: NONE
* Logic:  Left wheel backward, right wheel stationary
* Example Call: soft_left_2();
*/
void soft_left_2 (void)	{motion_set(0x01);		}


/*
* Function Name: soft_right_2
* Input: NONE
* Output: NONE
* Logic: Left wheel stationary, Right wheel backward
* Example Call: servo1_pin_config();
*/
void soft_right_2 (void){motion_set(0x08);		}


/*
* Function Name: stop
* Input: NONE
* Output: NONE
* Logic: stops both the motors
* Example Call: stop();
*/
void stop (void)		{	motion_set(0x00);	}

#endif /* MOTOR_H_ */
