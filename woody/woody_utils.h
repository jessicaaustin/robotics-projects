/****************************************************************************
 *
 *   Adapted from code given in the Society of Robots $50 robot tutorial:
 *     www.societyofrobots.com
 *
 ****************************************************************************/

//AVR includes
#include <avr/io.h>		    // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

//AVRlib includes
#include "global.h"		// include global settings
#include "a2d.h"		// include A/D converter function library

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)

#define S_LEFT_PORT 0
#define S_RIGHT_PORT 1
#define S_MID_PORT 2

#define  S_LEFT_FOR 36
#define  S_LEFT_MID 43
#define  S_LEFT_BACK 50

#define  S_RIGHT_FOR 47
#define  S_RIGHT_MID 37
#define  S_RIGHT_BACK 37

#define  S_MID_L_UP 46
#define  S_MID_DOWN 43
#define  S_MID_R_UP 39

int s_left_pos = S_LEFT_MID;
int s_right_pos = S_RIGHT_MID;
int s_mid_pos = S_MID_DOWN;


//configure ports for input or output - specific to ATmega8
void configure_ports(void)
{
	DDRC = 0x00;  //configure all C ports for input
	PORTC = 0x00; //make sure pull-up resistors are turned off
	DDRD = 0x37;  //configure D ports 0, 1, 2, 4, 5 for output, and port 3 for input (google search '0b00110111 to hex')
	DDRB = 0x00;  //configure all B ports for input
	PORT_OFF(PORTD, 5);
}

//wait for X amount of cycles (23 cycles is about .992 milliseconds)
//to calculate: 23/.992*(time in milliseconds) = number of cycles
void delay_cycles(unsigned long int cycles)
{
	while(cycles > 0)
		cycles--;
}
void delay(unsigned int millis) {
    delay_cycles(millis * 23); 
}

// helper functions
void LED_on(void)
{
	PORT_OFF(PORTD, 4);//turn LED on
}
void LED_off(void)
{
	PORT_ON(PORTD, 4);//turn LED off
}
void servo(unsigned int port, signed long int speed) {
	PORT_ON(PORTD, port);
	delay_cycles(speed);
	PORT_OFF(PORTD, port);//keep off
	delay_cycles(200);
}
void init_servos() {
    LED_on();
    delay(1000);
    LED_off();
    for (int i=0; i<100; i++) {
        servo(S_LEFT_PORT, s_left_pos);
        servo(S_RIGHT_PORT, s_right_pos);
        servo(S_MID_PORT, s_mid_pos);
    }
    LED_on();
}
void move_to(unsigned int port, int curr_pos, int next_pos) {
    if (curr_pos < next_pos) {
        for (int i=curr_pos; i<next_pos; i++) {
            servo(port, i);
            delay(50);
        }
    } else {
        for (int i=curr_pos; i>next_pos; i--) {
            servo(port, i);
            delay(50);
        }
    }
}
void move_left_servo(int new_pos) {
    move_to(S_LEFT_PORT, s_left_pos, new_pos);
    s_left_pos = new_pos;
}
void move_right_servo(int new_pos) {
    move_to(S_RIGHT_PORT, s_right_pos, new_pos);
    s_right_pos = new_pos;
}
void move_mid_servo(int new_pos) {
    move_to(S_MID_PORT, s_mid_pos, new_pos);
    s_mid_pos = new_pos;
}
void neutral() {
    move_left_servo(S_LEFT_MID);
    move_right_servo(S_RIGHT_MID);
    move_mid_servo(S_MID_DOWN);
}
void sustain_pos() {
    servo(S_LEFT_PORT, s_left_pos);
    servo(S_RIGHT_PORT, s_right_pos);
    servo(S_MID_PORT, s_mid_pos);
}
void hold_pos() {
    for (int i=0; i<50; i++) {
        sustain_pos();
    }
}

// Motion control
void move_left_forward() {
    move_mid_servo(S_MID_L_UP);
    hold_pos();
    move_left_servo(S_LEFT_FOR);
    hold_pos();
    move_mid_servo(S_MID_R_UP);
    hold_pos();
    move_left_servo(S_LEFT_MID);
    hold_pos();
    move_mid_servo(S_MID_DOWN);
    hold_pos();
}
void move_right_forward() {
    move_mid_servo(S_MID_R_UP);
    hold_pos();
    move_right_servo(S_RIGHT_FOR);
    hold_pos();
    move_mid_servo(S_MID_L_UP);
    hold_pos();
    move_right_servo(S_RIGHT_MID);
    hold_pos();
    move_mid_servo(S_MID_DOWN);
    hold_pos();
}
void move_forward() {
    move_mid_servo(S_MID_L_UP);
    hold_pos();
    move_left_servo(S_LEFT_FOR);
    hold_pos();
    move_mid_servo(S_MID_R_UP);
    hold_pos();
    move_left_servo(S_LEFT_MID);
    hold_pos();
    move_right_servo(S_RIGHT_FOR);
    hold_pos();
    move_mid_servo(S_MID_L_UP);
    hold_pos();
    move_right_servo(S_RIGHT_MID);
    hold_pos();
    move_mid_servo(S_MID_DOWN);
    hold_pos();
}

