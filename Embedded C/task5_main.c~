
/*
* Team Id: 4787
* Author List: Arjun Sadananda, Arvind Swaminath Kumar, Vinod Jacob Matthew, Vishnu Murali
* Filename: main.c
* Theme: Launch a Module
* Functions: port_init() , init_devices() , main()
* Global Variables: NONE
*/

#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#include "buzzer.h"
#include "motor.h"
#include "servo.h"
#include "communication.h"
#include "sensors.h"
#include "pickndrop.h"


/*
* Function Name: port_init
* Input: NONE
* Output: NONE
* Logic: Function to initialize ports
* Example Call: port_init();
*/
void port_init(){
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	buzzer_pin_config();
	motion_pin_config(); //robot motion pins config
	MOSFET_switch_config();
	left_encoder_pin_config(); //left  encoder pin config
	right_encoder_pin_config();//right encoder pin config
}

/*
* Function Name: init_devices
* Input:  NONE
* Output: NONE
* Logic: Function to initialize all the devices
* Example Call: init_devices();
*/
void init_devices(){
	cli();
	uart0_init(); //Initialize UART1 for serial communication //Clears the global interrupt
	timer1_init(); // timer 1
	PWM_timer_init(); // timer 5
	port_init();  //Initializes all the ports
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}

/*
* Function Name: servo1_pin_config
* Input: NONE
* Output: NONE
* Logic: does the job
* Example Call: Function called by the OS
*/
int main(void){
	
	init_devices();
	servo_1(0);
	_delay_ms(500);
	servo_1_free();
	servo_2(10);
	_delay_ms(500);
	servo_2_free();
	turn_off_all_proxy_sensors();
	turn_off_ir_proxi_sensors();
	turn_off_sharp15();
	turn_off_sharp234_wl();
	
	while(1) {
		if(communication_array[0] == 20){
			
			break;
			
		}
				
		if(communication_array[0] == 21){
			_delay_ms(500);
			lifter_up();
			_delay_ms(500);
			forward_mm(30);
			communication_array[0] = 0;
			
		}
				
		if(communication_array[0] == 18){ 
			//Pick
			forward_mm(50);
			lifter_down();
			_delay_ms(500);
			grabber_close();
			_delay_ms(500);
			lifter_up();
			forward_mm(50);
			communication_array[0] = 0;
		}
		if(communication_array[0] == 19){
			//Drop
			forward_mm(50);
			lifter_down();
			_delay_ms(500);
			grabber_open();
			_delay_ms(500);
			lifter_up();
			_delay_ms(500);
			back_mm(50);
			communication_array[0] = 0;
		}
	}
}
