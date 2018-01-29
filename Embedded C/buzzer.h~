/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: buzzer.h
* Theme: Launch a Module
* Functions: buzzer_pin_config(void) , buzzer_on(void) , buzzer_off(void)
* Global Variables: NONE
*/


#ifndef BUZZER_H_
#define BUZZER_H_

/*
* Function Name: buzzer_pin_config
* Input: NONE
* Output: NONE
* Logic: Configure PORTC for operation of buzzer
* Example Call: buzzer_pin_config();
*/

void buzzer_pin_config (void){
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*
* Function Name: buzzer_on
* Input: NONE
* Output: NONE
* Logic: Switches Buzzer on
* Example Call: buzzer_on();
*/

void buzzer_on (void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

/*
* Function Name: buzzer_off
* Input: NONE
* Output: NONE
* Logic: Switches Buzzer off
* Example Call: buzzer_off();
*/

void buzzer_off (void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

#endif /* BUZZER_H_ */
