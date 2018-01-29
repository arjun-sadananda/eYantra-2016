/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: pickndrop.h
* Theme: Launch a Module
* Functions: left_encoder_pin_config() , left_position_encoder_interrupt_init() , right_encoder_pin_config() , right_position_encoder_interrupt_init() , ISR(INT5_vect) , ISR(INT4_vect) , linear_distance_mm() , forward_mm() , back_mm()  
* Global Variables: ShaftCountLeft , ShaftCountRight
*/



#ifndef PICKNDROP_H_
#define PICKNDROP_H_


volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder


/*
* Function Name: left_encoder_pin_config
* Input: NONE
* Output: NONE
* Logic: Function to configure INT4 (PORTE 4) pin as input for the left position encoder
* Example Call: left_encoder_pin_config();
*/
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}


/*
* Function Name: right_encoder_pin_config
* Input: NONE
* Output: NONE
* Logic: Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call: right_encoder_pin_config();
*/
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name: left_position_encoder_interrupt_init
* Input: NONE
* Output: NONE
* Logic: Interrupt 4 enable
* Example Call: left_position_encoder_interrupt_init();
*/
void left_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}



/*
* Function Name: right_position_encoder_interrupt_init
* Input: NONE
* Output: NONE
* Logic: Interrupt 5 enable
* Example Call: right_position_encoder_interrupt_init();
*/
void right_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}



/*
* Function Name: ISR
* Input: INT5_vect
* Output: NONE
* Logic: ISR for right position encoder
* Example Call: ISR(INT7_vect);
*/
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


/*
* Function Name: ISR
* Input: INT4_vect
* Output: NONE
* Logic: ISR for left position encoder
* Example Call: ISR(INT6_vect);
*/
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


/*
* Function Name: linear_distance_mm
* Input: distance to move
* Output: NONE
* Logic: move bot by given mm
* Example Call: linear_distance_mm(250);
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

/*
* Function Name: forward_mm
* Input: forward distance in mm
* Output: NONE
* Logic: function to move forward a given distance
* Example Call: forward_mm(200);
*/
void forward_mm(unsigned int DistanceInMM)
{
	velocity(200,200);
	forward();
	linear_distance_mm(DistanceInMM);
}

/*
* Function Name: back_mm
* Input: Distance
* Output: NONE
* Logic: function to move back, a given distance
* Example Call: back_mm(150);
*/
void back_mm(unsigned int DistanceInMM)
{
	velocity(200,200);
	back();
	linear_distance_mm(DistanceInMM);
}


#endif /* PICKNDROP_H_ */