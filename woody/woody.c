#include "woody_utils.h" 

int main(void) {
	configure_ports(); 
    a2dInit();  
	a2dSetPrescaler(ADC_PRESCALE_DIV32);  
	a2dSetReference(ADC_REFERENCE_AVCC); 
    
    init_servos();
	LED_on();
    
    neutral();
    hold_pos();
    
    while (1) {
        move_forward();
    //    sustain_pos();
    }
    
	return 0;
}
