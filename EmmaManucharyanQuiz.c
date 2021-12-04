/********************************************
 *
 *  Name: Emma Manucharyan
 *  Email: emmanuch@usc.edu
 *  Section: Friday 11am
 *  Assignment: Project- Rangefinder
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "adc.h"
#include <avr/eeprom.h>

volatile error_flag = 0;
volatile char blue_control = 0;
volatile char flagged = 0;
volatile unsigned int buzzer_cnt = 0;
volatile unsigned char pulse_started = 0;
volatile unsigned long pulse_count = 0;
volatile char acquire_flag = 0;
volatile unsigned char watchdog = 0;
volatile unsigned int count = 0;
volatile unsigned char new_state, old_state;
volatile unsigned char changed = 0;  // Flag for state change
volatile  short near;	
volatile  short far;		// Count to display
volatile unsigned char near_far = 0;
volatile unsigned char a, b, c; //for each bit of the encoder input (b-MSB, a-LSB), and button
volatile unsigned char bits = 0;//to read both bits of the encoder input

int main(void) {

    // Initialize DDR and PORT registers and LCD
	DDRD |= (1<<2);
	PORTD |= (1<<3);
	PORTB |= (1<<4);
	DDRC |= (1<<5);
	lcd_init();
	
	//configure port b bit 3 to be output for OC2A bit
	DDRB |= (1<<3);
	PORTB |= (1<<3);

	//configure port c bit 4 to be output for blue contol bit
	DDRC |= (1<<4);
	PORTC |= (1<<4);

	//pull up resistors for port c bit 1,2,3, encoder and button
	PORTC |= ((1<<1)|(1<<2)|(1<<3));
	
    // Write a spash screen to the LCD
	lcd_moveto(0,0);
    lcd_stringout("Emma Manucharyan");
    char buff[14];
    unsigned int year_semester = 20213;
    snprintf(buff, 14, "Project %d", year_semester);
    lcd_moveto(1,1);
    lcd_stringout(buff);
    _delay_ms(1200);
    lcd_writecommand(1);

	//write near far
	lcd_moveto(0,0);
	lcd_stringout("Near");
	lcd_moveto(1,0);
	lcd_stringout("Far");

	///write equal sign initiall next to Near
	lcd_moveto(0,4);
	lcd_stringout("=");


	near = eeprom_read_word((void *) 101);

	if ((near > 395) || (near < 1)){
		near = 30;
	}

	far = eeprom_read_word((void *) 111);
	
	if ((far > 400) || (far < 6)){
		far = 300;
	}

	lcd_moveto(0,5);
	char near_buff[4];
    snprintf(near_buff, 4, "%3d", near);
	lcd_stringout(near_buff);
	lcd_moveto(1,5);
	char far_buff[4];
    snprintf(far_buff, 4, "%3d", far);
	lcd_stringout(far_buff);

	//enable pin change inturupts for port b
	PCICR |= (1<<PCIE0);

	//enable pin change inturupts for bit 4
	PCMSK0 |= (1<<PCINT4);

	//enable pin change inturupts for port d
	PCICR |= (1<<PCIE2);

	//enable pin change inturupts for bit 3
	PCMSK2 |= (1<<PCINT19);

	//enable pin change inturupts for port c
	PCICR |= (1<<PCIE1);

	//enable pin change inturupts for bit 1,2,3
	PCMSK1 |= ((1<<PCINT9)|(1<<PCINT10)|(1<<PCINT11));

	// Set timer 2 to fast PWM mode
    TCCR2A |= ((1 << WGM21)|(1 << WGM20));

	// Enable Timer Interrupt
    TIMSK2 |= (1 << OCIE2A);

	//set to non-inverting mode (clear bit on match)
    TCCR2A |= (1<<COM2A1);

	//Set prescalar to 256 (the T = 4.1ms)
    TCCR2B |= ((1<<CS22)|(1<<CS21));

    //Modulus 16,000,000*400*58/1,000,000/8 / prescaler 8
    OCR2A = 128;



	// Set timer 1 to CTC mode
    TCCR1B |= (1 << WGM12);

    // Enable Timer Interrupt
    TIMSK1 |= (1 << OCIE1A);

    //prescalar 8 set bit, initiallyt have it off
    //TCCR1B |= (1 << CS11);

    //Modulus 16,000,000*400*58/1,000,000 / prescaler 8
    OCR1A = 46400;

	// Set timer 0 to CTC mode
    TCCR0B |= (1 << WGM01);

    // Enable Timer Interrupt
    TIMSK0 |= (1 << OCIE0A);

    //prescalar 64 bits, initially have off
    // TCCR0B |= ((1<<CS01)|(1 << CS00));

    // //Modulus 16,000,000*0.001/64 _
    OCR0A = 250; ///~100Hz frequency pitch

	//enable globle inturrupt
	sei();


    //check initial state for encoder
	bits = PINC;
	b = (bits & (1<<1));
	a = (bits & (1<<2));
	c = (bits & (1<<3));//////////////////////////////////////

    if (!b && !a){
		old_state = 0;
	} else if (!b && a){
		old_state = 1;
	} else if (b && !a){
		old_state = 2;
	} else {
		old_state = 3;
	}
    new_state = old_state;

	//variables for main
	unsigned long distance = 0;
	unsigned int distance_whole = 0;
	unsigned int distance_decimal = 0;
	char count_buf[7];
	unsigned long PWM = 0;
	char encoder_buf[4];

	//play_note(frequency[0]);
    while (1) {   // Loop forever

        if (changed) { // Did state change?
	    	changed = 0;  // Reset changed flag
			if(near_far == 0){
				snprintf(encoder_buf, 4, "%3d", near);
				lcd_moveto(0,5);
				lcd_stringout(encoder_buf);
				eeprom_update_word((void *) 101, near);
				// lcd_moveto(1,14);
				// lcd_stringout("N");
			} else {
				snprintf(encoder_buf, 4, "%3d", far);
				lcd_moveto(1,5);
				lcd_stringout(encoder_buf);
				eeprom_update_word((void *) 111, far);
				// lcd_moveto(1,15);
				// lcd_stringout("F");
			}
		}
		if(flagged){
			flagged = 0;
			_delay_ms(180);
			if(near_far == 0){
				lcd_moveto(1,4);
				lcd_stringout(" ");
				lcd_moveto(0,4);
				lcd_stringout("=");
			} else {
				lcd_moveto(1,4);
				lcd_stringout("=");
				lcd_moveto(0,4);
				lcd_stringout(" ");
			}
		}

		if (acquire_flag){
			acquire_flag = 0;
			pulse_started = 1;
			_delay_ms(150);
			PORTD |= (1<<2);
			_delay_ms(10);
			PORTD &= ~(1<<2);
		}

		if(error_flag){
			error_flag = 0;
			lcd_moveto(1,11);
			lcd_stringout("     ");
			_delay_ms(100);
			lcd_moveto(1,11);
			lcd_stringout("Error");
		}

		if (pulse_count != 0){
			distance = pulse_count/11.6;
			distance_whole = distance/10;
			if((distance_whole>(near-1)) && (distance_whole<(far+1))){
				PWM = (((distance_whole - near)*255)/(far-near));
				OCR2A = PWM;
				PORTC |= (1<<4);

			} else if (distance_whole < near){
				OCR2A = 0;
				PORTC |= (1<<4);

				///buzzer, start timer
				PORTC |= (1<<5);
				TCCR0B |= ((1<<CS01)|(1 << CS00));
			

			} else if(distance_whole>far){
				if(distance_whole < 400){
					OCR2A = 255;
					PORTC |= (1<<4);
					// PORTB |= (1<<3);	

				} 
			}	
			lcd_moveto(1,11);
			lcd_stringout("     ");

			if (distance > 4000){
				lcd_moveto(0,10);
				lcd_stringout(">400.0");
				pulse_count = 0;
				watchdog = 0;
				OCR2A = 0;
				PORTC &= ~(1<<4);
			}	
			else {
				distance_decimal = (distance % 10);
				pulse_count = 0;
				snprintf(count_buf, 7, "%4d.%d", distance_whole,distance_decimal);
				lcd_moveto(0,10);
				lcd_stringout(count_buf);
			} 
		}
}	}

ISR(PCINT2_vect){
	
	if (pulse_started){
		TCNT1 = 0; //timer count set to 0
		pulse_started = 0;
		TCCR1B |= (1 << CS11);
	} else {
		TCCR1B &= ~(1 << CS11);
		pulse_count = TCNT1;
	}
}
ISR(PCINT0_vect){
	
	if((PINB & (1<<4)) == 0){
		acquire_flag = 1;
	}
	
}

ISR(PCINT1_vect){

	bits = PINC;
	b = (bits & (1<<1));
	a = (bits & (1<<2));
	c = (bits & (1<<3));
	if (near_far == 0){
		count = near;
	} else {
		count = far;
	}


	//for CW transition - increment count
	//for CCW transition - decrement count
	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if(a){
			new_state = 1;
			count++;
		}
		else if(b){
			new_state = 2;
			count--;
		}		
	}
	else if (old_state == 1) {
		// Handle A and B inputs for state 1
		if(!a){
			new_state = 0;
			count--;
		}
		else if(b){
			new_state = 3;
			count++;
		}	
	}
	else if (old_state == 2) {
		// Handle A and B inputs for state 2
		if(a){
			new_state = 3;
			count--;
		}
		else if(!b){
			new_state = 0;
			count++;
		}	
	}
	else if (old_state == 3) { 
		// Handle A and B inputs for state 3
		if(!a){
			new_state = 2;
			count++;
		}
		else if(!b){
			new_state = 1;
			count--;
		}
	}	


	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
		old_state = new_state;
		if (near_far == 0){
			if(((far-count) > 4) && (count > 0)){
				near = count;
				changed = 1;
			}
		} else {
			if(((count-near) > 4) && count < 401){
				far = count;
				changed = 1;
			}
		}
	}

	if(c == 0){
		flagged = 1;
		if(near_far == 0){
			near_far = 1;
		} else {
			near_far = 0;
		}
	}
	
}

ISR(TIMER1_COMPA_vect) {
	if (watchdog != 0){
		TCCR1B &= ~(1 << CS11);
		TCNT1 = 0;
		pulse_started = 0;
		watchdog = 0;
		error_flag = 1;
	}
	watchdog = 1;

}

ISR(TIMER0_COMPA_vect) {
	buzzer_cnt++;
	if(buzzer_cnt > 1000){
		buzzer_cnt = 0;
		TCCR0B &= ~((1 << CS00)|(1 << CS01));
		PORTC &= ~(1<<5);
	} else {
		if((buzzer_cnt % 2) != 0){
			PORTC &= ~(1<<5);
		} else {
			PORTC |= (1<<5);
		}
	}
	
}  //buzzer

ISR(TIMER2_COMPA_vect) {}
  //PWM

