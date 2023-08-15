/*
 * main.c
 *
 * Created: 30.04.2020 19:48:03
 * Author : Emre Emir Fidan
 */ 

#define T12

#define buttons_port PINB
#define plus_button 3
#define ok_button 4
#define minus_button 5
#define sensor_channel 5

#define IRON_MAX 250
#define IRON_MIN 0
#define IRON_SAMPLE_TIME 50

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include "lcd.h"
#include "timers.h"
#include "pid.h"

// ADC
static uint16_t t = 0;

// TTP
double target = 200;
double temperature = 0;
float adc = 0;
double pwm_ms = 0;

// Calibration Variables
float weight = 0.5;
float bias = -10.0;

// MENU VARIABLES
int menu = 0;
int menu_t = 0;
int menu_l = 3;

// PID VARIABLES
double kp = 1.6;
double ki = 1.5;
double kd = -1.5;

// IRON
int iron_power = 0;

void check_buttons();
void print_status();

void toogle_iron();

void led(uint8_t idt);

void toogle_menu();
void calibration();
void set_pid_var(int type);

void init_adc();
uint16_t read_adc( uint8_t channel );
void read_sensor();

void toogle_iron() {
	if (iron_power == 0) iron_power = 1;
	else iron_power = 0;
	menu = 0;
}

void led(uint8_t idt) {
	if (idt == 0) {
		PORTB &= ~(1<<2);
	}else {
		PORTB |= (1<<2);
	}
}

void init_adc() {
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0));
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN);
	ADCSRA |= (1<<ADSC);
}

uint16_t read_adc( uint8_t channel ) {
	ADMUX &= 0xF0;
	ADMUX |= 5;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADCW;
}

void read_sensor() {
	#ifdef T12
	set_pwm(0);
	_delay_ms(1);
	#endif
	t = read_adc(sensor_channel);
	_delay_ms(1);
	t += read_adc(sensor_channel);
	_delay_ms(1);
	t += read_adc(sensor_channel);
	_delay_ms(1);
	t += read_adc(sensor_channel);
	adc = t / 4;
	temperature = adc;
}

void calibration() {
	menu = 2;
	lcd_clrscr();
	uint16_t value = eeprom_read_word ((const uint16_t*)10);
	
	lcd_gotoxy(0,1);
	char str[16];
	sprintf(str, "Prg 1: %i", (uint16_t)target);
	lcd_puts(str);

	lcd_gotoxy(0,0);
	if (value == UINT16_MAX) {
		eeprom_write_word (( uint16_t *) 10, (uint16_t)target );
		lcd_puts("Kaydedildi - 1");
	}else {
		eeprom_update_word (( uint16_t *) 10, (uint16_t)target );
		lcd_puts("Kaydedildi - 2");
	}
	_delay_ms(2000);
	menu = 0;
}

void set_pid_var(int type) {
	if (type == 0) kp += 1;
	else if (type == 1) ki += 1;
	else if (type == 2) kd += 1;
	else if (type == 3) ki -= 1;
	else if (type == 4) kd -= 1;
	else if (type == 5) ki -= 1;
}

void toogle_menu() {
	if (menu == 1) {
		lcd_clrscr();
		lcd_gotoxy(0,0);
		lcd_puts("    - MENU -     ");
		lcd_gotoxy(0,1);
		if (menu_t == 0) {
			lcd_puts("Exit");
		}else if (menu_t == 1) {
			lcd_puts("Programi Kaydet");	
		}else if (menu_t == 2) {
			if (iron_power == 0) {
				lcd_puts("Ac");	
			}else if (iron_power == 1) {
				lcd_puts("Kapat");
			}
		}else if (menu_t == 3) {
			uint16_t value = eeprom_read_word((const uint16_t*)10);
			char str[16];
			sprintf(str, "Prg 1: %i", value);
			lcd_puts(str);			
		}
	}
}

void check_buttons() {
	if (buttons_port & (1<<plus_button)) {
		if (menu == 0) {
			target += 5;
		}else if (menu == 1) {
			if (menu_t == menu_l) menu_t = 0; else menu_t += 1;
		}
	}
	if (buttons_port & (1<<ok_button)) {
		if (menu == 1) {
			if (menu_t == 0) {
				menu = 0;
			}else if (menu_t == 1) {
				calibration();
			}else if (menu_t == 2) {
				toogle_iron();
			}
		}else {
			menu = 1;
			toogle_menu();
		}
	}
	if (buttons_port & (1<<minus_button)) {
		if (menu == 0) {
			target -= 5;
		}else if (menu == 1) {
			if (menu_t == 0) menu_t = menu_l; else menu_t -= 1;
		}
	}
}

void print_status() {
	lcd_clrscr();
	
	lcd_gotoxy(0,0);
	char str0 [16];
	sprintf(str0, "Snr:%i�C", (int)temperature);
	lcd_puts(str0);

	char str1 [3];
	if (iron_power) {
		lcd_gotoxy(13,0);
		sprintf(str1, "On");
	}else {
		lcd_gotoxy(13,0);
		sprintf(str1, "Off");	
	}
	lcd_puts(str1);

	lcd_gotoxy(0,1);
	char str2 [16];
	sprintf(str2, "Prg:%i�C", (int)target);
	lcd_puts(str2);
	
	lcd_gotoxy(13,1);
	char str3 [3];
	sprintf(str3, "%i", (int)pwm_ms);
	lcd_puts(str3);	
	
	// lcd_gotoxy(1,15);
	// lcd_custom_char(0x00, heart);
	// lcd_putc(0);
}

int main(void)
{
	enable_timer0();
	// DDRC = 0b11100010;
	DDRB = 0b00000011;
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();
    lcd_home();
    lcd_puts("    Merhaba     ");
    lcd_gotoxy(0,1);
    lcd_puts("            v0.3");
	init_adc();
	enable_timer1();

	int16_t value;
	value = eeprom_read_word((const uint16_t*)10);
	if ((value % 5) == 0 && value > 0) {
		target = (double) value;
	}
	
	set_pid(&temperature, &pwm_ms, &target, kp, ki, kd, IRON_SAMPLE_TIME, IRON_MIN, IRON_MAX);

	while(millis() <= 3000);
	unsigned static long int last_t = 0;
	unsigned static long int last_p = 0;
	unsigned static long int last_tp = 0;
	iron_power = 1;
    while(1)
    {
	    if (millis() - last_tp >= 20) {
		    read_sensor();
		    last_tp = millis();
	    }
		compute();
		if (millis() - last_p >= 50){
			if (iron_power == 1) {
				set_pwm((int)pwm_ms);
			} else {
				pwm_ms = 0;
				set_pwm(0);
			}
			last_p = millis();
		}
		
		if (((int)target - (int)temperature) < 10 && ((int)target - (int)temperature) > -10) {
			led(1);
		}else led(0);

		if (millis() - last_t >= 500) {
			check_buttons();
			if (menu == 0) {
				print_status();
				}else if (menu == 1){
				toogle_menu();
			}
			last_t = millis();	
		}
    }
}

