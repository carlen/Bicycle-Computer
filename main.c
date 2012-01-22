//-----------------------------------------------------
//    Bicycle Computer
//
//Created by Carlen @ Xenti technology
//
//Version: 0.1a
//Relase date: 2011.12.26
//Device: PIC18F2550 @ 20MHz
//Created with MPLAB IDE v8.73
//Language toolsite: C18 v3.38
//For the hw please visit the following website:
// www.google.com 
//-----------------------------------------------------
#if defined(__18F4550)
		#include <p18f4550.h>		//HW environment
		#pragma config FOSC = HS			//HS oscillator, HS used by USB 
		#pragma config MCLRE = ON			//MASTER RESET enable
		#pragma config VREGEN = OFF			//USB Vregulator OFF
		#pragma config PBADEN = OFF			//PORTB<4:0> pins are configured as digital I/O on Reset  
		#pragma config ICPRT = OFF			//ICPORT disabled 	
		#pragma config BORV = 3   			//Brown Out reset Minimum setting
		#pragma config LPT1OSC = ON			//low power timer 1 osc enable
#endif
#include <stdlib.h>
#include <usart.h>
#include <p18f4550.h>
#include "big_font.c"


#define 	SCK 	PORTAbits.RA1
#define 	SDA 	PORTDbits.RD0
#define		DC		PORTCbits.RC2
#define		RESET	PORTEbits.RE0

//-------------------------------------------------
//                LIST OF ROUTINES
//-------------------------------------------------

void lcd_clr(void);
void screen_refresh(void);
void low_isr ();
void write_image(void);
void write_time(void);
void write_misc(void);
void lcd_init(void);
void run_adc(void);

//-------------------------------------------------
//                     VARIABLES
//-------------------------------------------------

volatile int rs_232_input = 0;

volatile unsigned int wheeltime = 0;

volatile char sign = 0;
char operation = 1;

int speed = 0;
int speed_int = 0;
int speed_float = 0;

volatile int adc_result = 0;

volatile unsigned char time[6] = {0,0,0,0,0,0};		//sec:min:hour:day:month:year
volatile unsigned char runtime[6] = {0,0,0,0,0,0};	//sec:min:hour:day:mount:year

int wheeldistance = 22000;							//2*r*? in mm
long int distance[3] = {0,123,987};					//cm, m, km

volatile unsigned char state;
volatile unsigned char prev_state;
volatile unsigned char direction;

//-------------------------------------------------
//                     FUNCTIONS
//-------------------------------------------------

#pragma code vector_low = 0x08
void interrupt_vector_low (void)
{
	_asm
		goto low_isr
	_endasm
}
#pragma code
#pragma interrupt low_isr
void low_isr ()
{
	if (PIR1bits.TMR1IF)	
	{
		TMR1H = 0x80;
		time[0]++;						//Sec icrement
		
		if	(time[0] >= 60)					//Reach 1 min
		{
			time[0] = 0;
			time[1] += 1;					
			run_adc();
		}
		if (time[1] >= 60)					//Reach 1 hour
		{
			time[1] = 0;
			time[2] += 1;					
		}
		if (time[2] >= 24)					//Reach 1 day
		{
			time[2] = 0;
			time[3] += 1;
		}		
		
		write_time();
		PIR1bits.TMR1IF = 0;		
	}
	if (PIR1bits.TMR2IF)					//Timer2 interrupt
	{
		wheeltime++;
	
		/*
		state = PORTBbits.RB1 | PORTBbits.RB2 << 1;
			
		if(prev_state != 0xFF)
		{
			if(prev_state == 0b00 && state == 0b01)
			{
				direction = 1;
				putrsUSART((const far rom char *) "Balra\n\r");
			}	
			if(prev_state == 0b01 && state == 0b00)
			{
				direction = 2;
				putrsUSART((const far rom char *) "Jobbra\n\r");
			}
		}
		prev_state= state;
		*/

		PIR1bits.TMR2IF = 0;				//clear flag bit
	}		
	if	(INTCON3bits.INT1IF || INTCON3bits.INT2IF)
	{
		state = PORTBbits.RB1 | PORTBbits.RB2 << 1;
		if(prev_state != 0xFF)
		{
			if(prev_state == 0b00 && state == 0b01)
			{
				direction = 1;
				putrsUSART((const far rom char *) "Balra\n\r");
			}	
			if(prev_state == 0b01 && state == 0b00)
			{
				direction = 2;
				putrsUSART((const far rom char *) "Jobbra\n\r");
			}
		}
		prev_state= state;
				
		INTCON3bits.INT1IF = 0;
		INTCON3bits.INT2IF = 0;
	}	
}
void init_system(void)
{	
	//--------------PORT--------------
	ADCON1 = 0b00001111;
	ADCON0bits.ADON = 0;
	TRISA = 0;
	PORTA = 0;	
	
	TRISB = 0;
	PORTB = 0;
	TRISBbits.TRISB2 = 1;
	TRISBbits.TRISB1 = 1;
	
	TRISC = 0;
	PORTC = 0;
	
	TRISD = 0;
	PORTD = 0;
	
	TRISE = 0;
	PORTE = 0;
	
	//TRISCbits.TRISC0 = 1;	//RTC crystal (??)
	//TRISCbits.TRISC1 = 1;	//RTC crystal (??)
	
	//--------------RS232--------------
	#define		RS232			  
	SPBRG = 10;				
	
	BAUDCON = 0b00000000;	//RX-TX not invert, 16Bit baud, no rollover
	TXSTA = 0b00100100;		//Enable 8bit async transmit mode, high speed baud
	RCSTA = 0b10010000;		//Enable 8bit receiver module
	
	//--------------TIMER1--------------
	TMR1L = 0;				//Clear low bit
	TMR1H = 0x80;			//Preload timer
	T1CON = 0b00001110;		//8bit, External source, 1:1, Timer1 enable
	
	//--------------TIMER2--------------
	T2CON = 0b01111111;		//1:16 postscale, Timer2 on, 1:16 prescaler	
	PR2 = 0xC3;				//Pre-charging decimal 195 
		
	//--------------EEPROM----------------
	EECON1bits.EEPGD = 0;	//Point to DATA memory
	EECON1bits.CFGS = 0;	//Access EEPROM
	
	//--------------A/DCON--------------
	ADCON2 = 0b10100010;	//Right, 8Tad~, 32Tosc time 
	ADCON1 = 0b00001110;	//Reference Vss, Vdd -> Ref+, AN0 analog 
	ADCON0 = 0b00000001;	//AN0, AD on
	
	//--------------INTERRUPT--------------
	//Global interrupt settings
	RCONbits.IPEN = 0;		//No priority level
	INTCONbits.PEIE = 1;	//Peripheral interrupts enable
	INTCONbits.GIE = 1;		//Enable interrupt

	//Timer1 interrupt settings
	PIE1bits.TMR1IE = 1;	//Timer1 interrupt enable
	PIR1bits.TMR1IF = 0;	//Clear flag bit
	IPR1bits.TMR1IP = 0;	//Low priority (don't care)

	//Timer2 interrupt settings
	PIE1bits.TMR2IE = 1;	//Timer2 interrupt enable
	IPR1bits.TMR2IP = 0;	//Low priority (don't care)
	PIR1bits.TMR2IF = 0;	//Clear flag bit

	////INT2 interrupt settings
	INTCON2bits.INTEDG2 = 1;//INT2 interrupt on falling edge
	INTCON3bits.INT2IE = 0;	//Enable INT2 interrupt
	INTCON3bits.INT2IP = 0;	//Low priotity (don't care)
	INTCON3bits.INT2IF = 0;	//Clear flag bit
	
	//INT1 interrupt settings
	INTCON2bits.INTEDG1 = 1;//INT1 interrupt on falling edge
	INTCON3bits.INT1IE = 0;	//Enable INT1 interrupt
	INTCON3bits.INT1IP = 0;	//Low priority (don't care)
	INTCON3bits.INT1IF = 0;	//Clear flag bit

}
void send_byte(unsigned char data, int DI)
{
	unsigned char i = 0;
		
	if (DI) DC = 1;
	else	DC = 0;
									
	for(i=0;i<=7;++i)
		{
		if ( (data & 0x80) == 0x80 )
			{
			SDA = 1;
			}
		else
			{
			SDA = 0;
			}
		SCK = 1;
		
		SCK = 0;
		SDA = 0;		
	
		data <<= 1;
		}	
}	
void lcd_clr(void)
{
	unsigned int i;
	for( i=0; i<504; i++)
	{
		send_byte(0x00,1);
	}
}
void lcd_init (void)
{		
	RESET = 0;
	RESET = 1;

	send_byte(0x21,0);  // extended instruction set
	send_byte(0x90,0);  // set VOP
	send_byte(0x06,0);  // set temperature co-eff
	send_byte(0x13,0);  // bias (1:48)
	send_byte(0x20,0);  // normal instruction set: PD= 0, V= 0
	send_byte(0x0C,0);  // normal mode: d=1, e=0
	lcd_clr();
	putrsUSART((const far rom char *) "LCD init\n\r");
}
void lcd_xy (unsigned char  x, unsigned char y) 
{
	send_byte((x|0x80),0);  
	send_byte((y|0x40),0); 
}     
void write_bat(unsigned int value)
{
	unsigned int i = 0;

	lcd_xy(68,0);
	for (i=0;i<=10;i++)
	{
		send_byte(battery[value][i],1);
	}		
}
void write_misc(void)
{
	unsigned int i = 0;

	lcd_xy(58,4);
	for (i=0;i<=20;i++)	
	{
		send_byte(misc[i],1);
	}

	lcd_xy(13,0);	
	send_byte(36,1);
	
	lcd_xy(27,0);
	send_byte(36,1);
	if (speed_int != 0)
	{
		lcd_xy(1,4);
		for (i=0;i<=17;i++)
		{
			send_byte(text[4][i],1);
		}		
	}
	else
		{
		lcd_xy(1,4);
		for (i=0;i<=17;i++)
		{
			send_byte(0,1);
		}		
	}		
}
void write_time(void)
{
	int i,temp;

	temp = time[0] / 10; 
	lcd_xy(29,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}
	
	temp = time[0] % 10;
	lcd_xy(35,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}
	
	temp = time[1] / 10;
	lcd_xy(15,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}	
	
	temp = time[1] % 10;
	lcd_xy(21,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}
	
	temp = time[2] / 10;
	lcd_xy(1,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}	
	
	temp = time[2] % 10;
	lcd_xy(7,0);
	for (i=0; i<=4;i++)
	{
		send_byte(number[temp][i],1);
	}	
		
}	

void write_number(unsigned int number)
{
	int i, a, b;
	a = number / 10;
	b = number % 10;
	
	lcd_xy(32,2);
	for(i=0;i<=10;i++)
	{	
		send_byte(big_num[a][i],1);
	}		
	lcd_xy(32,3);
		for(i=0;i<=10;i++)
	{	
		send_byte(big_num[a][(i+11)],1);
	}
	lcd_xy(32,4);
		for(i=0;i<=10;i++)
	{	
		send_byte(big_num[a][(i+22)],1);
	}
	
	lcd_xy(44,2);
	for(i=0;i<=10;i++)
	{	
		send_byte(big_num[b][i],1);
	}		
	lcd_xy(44,3);
		for(i=0;i<=10;i++)
	{	
		send_byte(big_num[b][(i+11)],1);
	}
	lcd_xy(44,4);
		for(i=0;i<=10;i++)
	{	
		send_byte(big_num[b][(i+22)],1);
	}
}
void s_decki (unsigned char b, unsigned short long i)	
{
	unsigned short long j = 1;
	unsigned char a;
	
	{
		for (a = 1; i >= j; a++)
		{
			j = (j * 10);
		}
		j = (j / 10);
		while (b >= a)
		{WriteUSART (0x30);while (BusyUSART( )){Nop();} b--;}
		while ((a) > 1)
		{
			WriteUSART (((i / j)+0x30));
			while (BusyUSART( )){Nop();}
			i = (i % j);
			j = (j / 10);
			a--;
		}
	}
}
unsigned char s_decbe_1 (void)		
{
	while (PIR1bits.RCIF == 0)
	{Nop();}
	while (RCREG < 0x30 || RCREG > 0x3A)
	{
		while (PIR1bits.RCIF == 0)
		Nop();
	}

	return (RCREG - 0x30);
}
void eeprom_write (unsigned char adr, unsigned char data)
{
	EEADR = adr;							//set addres
	EEDATA = data;							//set data
		
	EECON1bits.WREN = 1;					//enable EEPROM write
	INTCONbits.GIE = 0;						//disabel interrupt
	EECON2 = 0x55;						
	EECON2 = 0x0AA;
	EECON1bits.WR = 1;						//start write
	INTCONbits.GIE = 1;						//enable interrupt

	while (!PIR2bits.EEIF) Nop();
	EECON1bits.WREN = 0;	
	PIR2bits.EEIF = 0;
}
unsigned char eeprom_read (unsigned char adr)
{
	EEADR = adr;
	EECON1bits.RD = 1;
	while (EECON1bits.RD) Nop();
	return EEDATA;
}	
void eeprom_decimal_write (int data, unsigned char adr, int lenght)
{
	unsigned int temp = data;
	unsigned int i = 1;
	unsigned int j = 0;
	
	while(i <= lenght )
	{
		j = temp % 10;
		temp /= 10;
		eeprom_write((adr+i - 1),(j + 0x30));	
		i++;	
	}	
}	
unsigned int eeprom_decimal_read (int adr, unsigned char lenght)
{
	unsigned char i = lenght;
	unsigned int out = 0;
	
	while(i)
	{
		out += eeprom_read((adr + (i- 1))) - 48;
		out *= 10;
		i--;
	}
	out /= 10;
	return out;
}

void read_total(void)
{
	distance[1] = eeprom_decimal_read (0x00, 4);	
	distance[2] = eeprom_decimal_read (0x04, 4);
	runtime[2] = eeprom_decimal_read (0x10, 2);
	runtime[1] = eeprom_decimal_read (0x12, 2);
}
void write_total(void)
{
	eeprom_decimal_write(distance[1], 0x00, 4);
	eeprom_decimal_write(distance[2], 0x04, 4);
	eeprom_decimal_write(runtime[2], 0x10, 2);
	eeprom_decimal_write(runtime[1], 0x12, 2);
}
void set_time (void)
{
	unsigned int i = 0;

	putrsUSART ((const far rom char *)"\n\rSet year:  20");
	i = s_decbe_1();
	s_decki(1,i);
	i = i * 10;
	time[5] = i;
	
	i = s_decbe_1();
	s_decki(1,i);
	time[5] += i;

	putrsUSART ((const far rom char *) "\n\rSet mounth: ");
	i = s_decbe_1();
	s_decki(1,i);
	i = i * 10;
	time[4] = i;
	
	i = s_decbe_1();
	s_decki(1,i);
	time[4] += i;
	
	putrsUSART ((const far rom char *) "\n\rSet day: ");
	i = s_decbe_1();
	s_decki(1,i);
	i = i * 10;
	time[3] = i;
	
	i = s_decbe_1();
	s_decki(1,i);
	time[3] += i;

	putrsUSART ((const far rom char *) "\n\rSet hour: ");
	i = s_decbe_1();
	s_decki(1,i);
	i = i * 10;
	time[2] = i;
	
	i = s_decbe_1();
	s_decki(1,i);
	time[2] += i;

	putrsUSART ((const far rom char *) "\n\rSet min: ");
	i = s_decbe_1();
	s_decki(1,i);
	i = i * 10;
	time[1] = i;
	
	i = s_decbe_1();
	s_decki(1,i);
	time[1] += i;
	
	T1CONbits.TMR1ON = 1;
	putrsUSART ((const far rom char *) "\n\r Time saved! RTC started! \n\r");
}	
void write_text (int x, int y, unsigned int data)
{

	int i;
	lcd_xy(x,y);
	for (i=0;i<=4;i++)
	{
	send_byte(font[(data * 5)+i],1);		
	}
}	
void run_adc(void)
{
	int i = 0;
	unsigned int average;
		
	for (i = 0; i<20; i++)
	{
		ADCON0bits.GO = 1;
		while (ADCON0bits.GO) Nop();
		
		adc_result = ADRESH * 255 + ADRESL;
		average += adc_result;
	}
	average /= 20;

	if (average <=799) average = 0;
	if (average >= 700 && average <=823) average = 1;
	if (average >= 824 && average <=841) average = 2;
	if (average >= 842 && average <=856) average = 3;
	if (average >= 857 && average <=883) average = 4;
	if (average >= 884 && average <=892) average = 5;
	if (average >= 893 && average <=909) average = 6;
	if (average >= 910 && average <=928) average = 7;
	if (average >= 930 && average <=949) average = 8;
	if (average >= 950 && average <=969) average = 9;
	if (average >= 970 && average <=985) average = 10;
	if (average >= 986) average = 10;

	lcd_xy(68,0);
	for (i=0;i<=10;i++)
	{
		send_byte(battery[average][i],1);	
	}	
}					
void main ()
{
	unsigned char rs_232_rx_status = 0;	
				
	init_system();
	
	putrsUSART ((const far rom char *) "\n\rLCD v2 test program USART console started! \n\n\rPress 'h' to help menu...\n\rMeg a faszom\n\rBazd meg!!!!!!!!!!\n\r");

	while (1)
		{
		
		
		
		if (sign == 1 && wheeltime > 20)
			{
				sign = 0;										//clear sign bit		
				speed = wheeldistance / (wheeltime * 10) * 36;	//calc speed
				speed_int = speed / 100;						//speed integer part
				speed_float = (speed % 100) / 10;				//speed float part
				
				wheeltime = 0;									//clear time
				
				lcd_init();	
				write_number(speed_int);						//write speed				
				write_misc();
				
				distance[0] += 20;								//one turn distance cm
				distance[1] += 2;								//one turn distance m
				
				if (distance[0] >= 100)							//cm reach one m
				{
					distance[1] ++;
					distance[0] -= 100; 
				}
				
				if (distance[1] >= 1000)						//m reach one km
				{
					distance[2]++;
					distance[1] -= 1000; 
					
				}		
			}	//end if (sign) loop	
		if (wheeltime == 800)
			{
				speed_int = 0;
				speed_float = 0;
				write_misc();
				write_number(speed_int);
			}	//end speed = 0 
		if (wheeltime == 6000)
			{
				wheeltime = 0;
				T2CONbits.TMR2ON = 0;							//Timer 2 off
				write_total();
				operation = 0;									//Sleep mode on
				putrsUSART ((const far rom char *)"\n\r Sleep mode enable\n\r");
				_asm
					SLEEP
					NOP
				_endasm	
			}	
	
		if (PIR1bits.RCIF)			//rs232 reveive
			{
				rs_232_input = RCREG;			//read & flag törlés
				
				if (RCSTAbits.FERR)
				{
					putrsUSART ((const far rom char *)"\n\rFraming Error!!!\n\r");
					rs_232_rx_status = RCSTA;
				
				}
				if (RCSTAbits.OERR)
				{
					putrsUSART ((const far rom char *)"\n\rOverrun Error!!!\n\r");
					RCSTAbits.CREN = 0;			
					RCSTAbits.CREN = 1;
				}
			
				switch(rs_232_input)
				{
					case '0':	lcd_init();
								break;
					case '1':   	set_time();
								break;	
					case '2': 	write_misc();
								break;
					case '5':	write_text(5,2,'5');
								break;
					case 'p': 	putrsUSART ((const far rom char *)"\n\rPONG\n\r");
								break;


				}
			} 
		
		}				
}

