
/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: sensors.h
* Theme: Launch a Module
* Functions: MOSFET_switch_config() , turn_on_all_proxy_sensors() , turn_on_ir_proxi_sensors() , turn_on_sharp15() , turn_on_sharp234_wl() , turn_off_all_proxy_sensors() , turn_off_ir_proxi_sensors() , turn_off_sharp15() , turn_off_sharp234_wl()
* Global Variables: NONE
*/


#ifndef SENSORS_H_
#define SENSORS_H_


/*
* Function Name: MOSFET_switch_config
* Input: NONE
* Output: NONE
* Logic: MOSFET switch port configuration
* Example Call: MOSFET_switch_config();
*/
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}


/*
* Function Name: turn_on_sharp234_wl()
* Input: NONE
* Output: NONE
* Logic: turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
* Example Call: turn_on_sharp234_wl();
*/
void turn_on_sharp234_wl (void) 
{
	PORTG = PORTG & 0xFB;
}


/*
* Function Name: turn_off_sharp234_wl
* Input: NONE
* Output: NONE
* Logic: turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
* Example Call: turn_off_sharp234_wl();
*/
void turn_off_sharp234_wl (void) 
{
	PORTG = PORTG | 0x04;
}


/*
* Function Name: turn_on_sharp15
* Input: NONE
* Output: NONE
* Logic: turn on Sharp IR range sensors 1,5
* Example Call: turn_on_sharp15();
*/
void turn_on_sharp15 (void) 
{
	PORTH = PORTH & 0xFB;
}


/*
* Function Name: turn_off_sharp15
* Input: NONE
* Output: NONE
* Logic: turn off Sharp IR range sensors 1,5
* Example Call: turn_off_sharp15();
*/
void turn_off_sharp15 (void) 
{
	PORTH = PORTH | 0x04;
}


/*
* Function Name: turn_on_ir_proxi_sensors()
* Input: NONE
* Output: NONE
* Logic: turn on IR Proximity sensors
* Example Call: turn_on_ir_proxi_sensors();
*/
void turn_on_ir_proxi_sensors (void) 
{
	PORTH = PORTH & 0xF7;
}


/*
* Function Name: turn_off_ir_proxi_sensors
* Input: NONE
* Output: NONE
* Logic: turn off IR Proximity sensors
* Example Call: turn_off_ir_proxi_sensors();
*/
void turn_off_ir_proxi_sensors (void) 
{
	PORTH = PORTH | 0x08;
}


/*
* Function Name: turn_on_all_proxy_sensors
* Input: NONE
* Output: NONE
* Logic: turn on Sharp 2, 3, 4, red LED of the white line sensors,sharp 1, 5 and IR proximity sensor
* Example Call: turn_on_all_proxy_sensors();
*/
void turn_on_all_proxy_sensors (void) 
{
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}


/*
* Function Name: turn_off_all_proxy_sensors
* Input: NONE
* Output: NONE
* Logic: turn off Sharp 2, 3, 4, red LED of the white line sensors,sharp 1, 5 and IR proximity sensor
* Example Call: turn_off_all_proxy_sensors();
*/

void turn_off_all_proxy_sensors (void) 
{
	PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
	PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}


#endif /* SENSORS_H_ */