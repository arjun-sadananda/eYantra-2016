/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: servo.h
* Theme: Launch a Module
* Functions: servo1_pin_config(void) , servo2_pin_config(void) , timer1_init(void) , servo_1(unsigned char) , servo_2(unsigned char) , servo_1_free(void) , servo_2_free(void) , grabber_close() , grabber_open() , lifter_up() , lifter_down()  
* Global Variables: NONE
*/


#ifndef SERVO_H_
#define SERVO_H_


/*
* Function Name: servo1_pin_config
* Input: NONE
* Output: NONE
* Logic: Configure PORTB 5 pin for servo motor 1 operation
* Example Call: servo1_pin_config();
*/

void servo1_pin_config (void){
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
* Function Name: servo2_pin_config
* Input: NONE
* Output: NONE
* Logic: Configure PORTB 6 pin for servo motor 1 operation
* Example Call: servo2_pin_config();
*/

void servo2_pin_config (void){
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*
* Function Name: timer1_init
* Input: NONE
* Output: NONE
* Logic: TIMER1 initialization in 10 bit fast PWM mode
* Example Call: timer1_init();
*/

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
* Function Name: servo_1
* Input: angle by which servo 1 has to be rotated
* Output: NONE
* Logic: Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
* Example Call: servo_1(70);
*/
void servo_1(unsigned char degrees){
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

/*
* Function Name: servo_2
* Input: angle by which servo 2 has to be rotated
* Output: NONE
* Logic: Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
* Example Call: servo_2(80);
*/

void servo_2(unsigned char degrees){
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.


/*
* Function Name: servo_1_free
* Input: NONE
* Output: NONE
* Logic: makes servo 1 free rotating
* Example Call: servo_1_free();
*/


void servo_1_free (void) {
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

/*
* Function Name: servo_2_free
* Input: NONE
* Output: NONE
* Logic: makes servo 2 free rotating
* Example Call: servo_2_free();
*/

void servo_2_free (void) {
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

/*
* Function Name: grabber_close
* Input: NONE
* Output: NONE
* Logic: Function to hold the object
* Example Call: grabber_close();
*/

void grabber_close(){
	int i;
	for (i = 10; i <= 44; i++){
		servo_2(i);
		_delay_ms(10);
	}
	servo_2_free();
}

/*
* Function Name: grabber_open
* Input: NONE
* Output: NONE
* Logic: Function to release the object
* Example Call: grabber_open();
*/

void grabber_open(){
	int i;
	for (i = 44; i > 10; i--){
		servo_2(i);
		_delay_ms(10);
	}
	servo_2_free();
}
/*
* Function Name: lifter_up
* Input: NONE
* Output: NONE
* Logic: Function to lift the object
* Example Call: lifter_up();
*/

void lifter_up(){
	int i;
	for (i = 0; i < 70; i++){
		servo_1(i);
		_delay_ms(10);
	}
	servo_1_free();
}

/*
* Function Name: lifter_down
* Input: NONE
* Output: NONE
* Logic: Function to lower the object for drop
* Example Call: lifter_down();
*/

void lifter_down(){
	int i;
	for (i = 70; i > 0; i--){
		servo_1(i);
		_delay_ms(10);
	}
	servo_1_free();
}
#endif /* SERVO_H_ */