//Created by: James Wyatt - N8040478
//Date: May 2021
//DOOR-LOCK Application

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// avr
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>

// graphics
#include <lcd.h>
#include <macros.h>
#include <ascii_font.h>
#include <graphics.h>

//timer definitions
#define FREQ (16000000.0)
#define PRESCALE (1024.0)
const double timeout = 20; //gives around 15 seconds for timeout

/* Used to store the key state of keypad*/
int Key = 0;

//used to store correct keycode
int sequence[] = {1,2,3,4}; //array giving correct key code
int entered[5];             // array of actual keys entered
int badCode = 0;
int correct = 0;

//uart Initilization 
//---------------------------------------------------------------
void uart_init(unsigned int MyUbrr){
 UBRR0H = (unsigned char)(MyUbrr>>8);
 UBRR0L = (unsigned char)(MyUbrr);
 UCSR0B = (1 << RXEN0) | (1 << TXEN0);
 UCSR0C = (3 << UCSZ00);

}
//uart putchar
//---------------------------------------------------------------
void uart_putchar(unsigned char MyData){

 while (!( UCSR0A & (1 <<UDRE0))); /* Wait for empty transmit buffer */
 UDR0 = MyData; /* Put data into buffer, sends the data */
} 

//uart putstring
//----------------------------------------------------------------
void uart_putstring(unsigned char* a)
{
	// transmit character until NULL is reached
	while(*a > 0) uart_putchar(*a++);
}

//ADC_INIT------------
void ADC_init()
{
  // initalize ADC
	// ADC Enable and pre-scaler of 128: ref table 24-5 in datasheet
	// ADEN  = 1
	// ADPS2 = 1, ADPS1 = 1, ADPS0 = 1
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

//ADC_READ---------------
uint16_t ADC_read()
{
  //CHANNEL 5 PC5 (A5 on the uno) - reading the LDR 
  ADMUX = (1 << REFS0)|(1 << MUX2)|(0 << MUX1)|(1 << MUX0);

  // Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);
 
	// Wait for ADSC bit to clear, signalling conversion complete.
	 while ( ADCSRA & (1 << ADSC) ) {}
  
	// Result now available in ADC
	return ADC;
}

//SETUP---------------------
//--------------------------------------------------------------------
void setup() {

  /* initialize uart serial communication*/
  uart_init(MYUBRR);

  // Timer 0 in normal mode, with pre-scaler 1024 ==> ~60Hz overflow.
  // Timer overflow on.
  TCCR0A = 0;
  TCCR0B = 5;
  TIMSK0 = 1;

  // Enable timer overflow, and turn on interrupts.
  sei();

  //ADC INIT
  ADC_init();

  // Configure ADC To enable PULLUP RESISTOR and ANALOG INPUT
  DDRC &= ~(1<<DDC5);    //Configure PORTC 5 as INPUT (A5)

  PORTC |= (1<<PC5);     //Activate internal PULLUP RESISTOR

  //TURN PD2 to OUTPUT BACKLIGHT CONTROL PIN
  DDRD |= (1 << 2);

  /* Configure the Keypad clock pin - it is an OUTPUT from the Arduino*/

  DDRB |= (1 << 0);                  // or DDRB |= (1 << DDB0) or SET_BIT
  
  /* Configure the Keypad data pin - it is an INPUT to the Arduino*/

  DDRB &= ~(1 << 4);                 // or DDRB &= ~(1 << DDB4) or CLEAR_BIT

  /* Configure the PIEZO SPEAKER */

  DDRB |= (1 << 2);

  /* Configure LEDS - OUTPUT */
  //BLUE_RED_FLASH_LED
  DDRC |= (1 << 2);

  //RED_LED
  DDRC |= (1 << 1);

  //GREEN_LED
  DDRC |= (1 << 0);

  /* Configure SOLENOID - OUTPUT*/
  DDRC |= (1 << 3);

  /* Configure the LCD */
  lcd_init(LCD_DEFAULT_CONTRAST);
  clear_screen();
  show_screen();
}

char myBuffer[50];

// DRAW INT 
void draw_int(uint8_t a, uint8_t b, int num) 
{
  snprintf( myBuffer, 50, "%d", num);
  draw_string(a, b, myBuffer, FG_COLOUR);
}

//LCD FIRST MESSAGE
//---------------------------------------------------------------------
char lcd_begin_string[64];
char lcd_begin_string1[64];
char lcd_begin_string2[64];

void Lcd_Begin(){
	clear_screen();
  sprintf(lcd_begin_string, "YOU ARE BEING");
  sprintf(lcd_begin_string1, "RECORDED ON");
  sprintf(lcd_begin_string2, "CAMERA");
	draw_string(0, 0, lcd_begin_string, FG_COLOUR);
	draw_string(0, 16, lcd_begin_string1, FG_COLOUR);
  draw_string(0, 32, lcd_begin_string2, FG_COLOUR);
  show_screen();
  _delay_ms(100);
}

//LCD SECOND MESSAGE
//---------------------------------------------------------------------
char lcd_enter_num_string[64];
char lcd_enter_num_string1[64];

void Lcd_EnterNum()
{
  //DEBUG - if I take lcd_init away it removes some of the text on LCD
  lcd_init(LCD_DEFAULT_CONTRAST);
  clear_screen();
  sprintf(lcd_enter_num_string, "Enter Your");
  sprintf(lcd_enter_num_string1, "4 Number Code");
	draw_string(0, 0, lcd_enter_num_string, FG_COLOUR);
	draw_string(0, 16, lcd_enter_num_string1, FG_COLOUR);
  _delay_ms(3000);
	show_screen();
  _delay_ms(100);
}

/*------------------READ THE STATE OF THE KEYPAD -------------------------*/ 
int Read_Keypad(void)
{
  int Count;
  int Key_State = 0;
  /* Pulse the clock pin 16 times (one for each key of the keypad) 
     and read the state of the data pin on each pulse */
  for(Count = 1; Count <= 16; Count++)
  {
    //CLEAR_BIT PD0 (LOW)
    //PORTB &= ~(1<<PD0);                                             
    PORTB &= ~(1<<0);   

    _delay_ms(1);                                                      

    /* If the data pin is low (active low mode) then store the 
      current key number */
    //BIT_VALUE PB4 == 0 (LOW)
    //If Key is pressed 
    if ((PINB & (1<<4))== 0)           // DATA FROM KEYBOARD IS RECEIVED ON PORTB PIN4
    {
      Key_State = Count; 
    }
    //SETBIT PD0 (HIGH)
    //PORTB |= (1<<PD0);
    PORTB |= (1<<0);                                                  
  } 
  return Key_State; 
}

/*-------------------------------------------------------------------*/
//                           PIEZO SPEAKER SOUND
//               duration in mSecs, frequency in Hertz
void playTone(long duration, int freq) {
  duration *= 1000;
  int period = (1.0 / freq) * 1000000;
  long elapsed_time = 0;
  while (elapsed_time < duration) 
  {
    PORTB |= (1<<2);   //BIT_BANGING (PWM by changing port from 0 to 1 and back)          
    _delay_us(500);
    PORTB &= ~(1<<2);               
    _delay_us(500);
    elapsed_time += (period);
  }
}

//------- PLAY TONE 2
void playTone2(long duration, int freq) 
{
  duration *= 1000;
  int period = (1.0 / freq) * 1000000;
  long elapsed_time = 0;
  while (elapsed_time < duration) 
  {
    PORTB |= (1<<2); //BIT_BANGING
    _delay_us(1500);
    PORTB &= ~(1<<2); 
    _delay_us(1500);
    elapsed_time += (period);
  }
}

//----------SoundAlarm
void soundAlarm(){
  //BLUE/RED LED TO HIGH
  PORTC |= (1 << 2); //SET PC2 to 1
  for(int i = 0; i <= 10; i++)
  {  
    playTone(300,1550);
    _delay_ms(100);
    playTone2(300,250);
    _delay_ms(100);
  }
  //BLUE/RED LED TO LOW
  PORTC &= ~(1 << 2); //CLEAR PC2 to 0
}
  
/*--------------------------------------------------------------------*/

//                       CHIRP for each key entry 
void chirp(){
  playTone(50, 1000);
  //GREEN LED HIGH
  PORTC |= (1 << 0); //SET PC0 to 1
  _delay_ms(250);
  //GREEN LED LOW
  PORTC &= ~(1 << 0); //CLEAR PC0 to 0
}

//CHIRP CHIRP
void chirpchirp(){
  for (int i = 0; i <= 1; i ++)
  {
  playTone(50, 1000);
  _delay_ms(100);
  }
  //GREEN to HIGH
  PORTC |= (1 << 0); //SET PC0 to 1
  //SOLENOID TO HIGH
  PORTC |= (1 << 3); //SET PC3 to 1
  _delay_ms(2000);
  //GREEN TO LOW
  PORTC &= ~(1 << 0); //CLEAR PC0 to 0
  //SOLENOID TO LOW
  PORTC &= ~(1 << 3); //CLEAR PC3 to 0
}

//WRONG CODE
void wrongCode(){
  playTone2(500,400);
  _delay_ms(100);
  //RED LED HIGH
  PORTC |= (1 << 1); //SET PC1 to 1
  _delay_ms(1000);
  //RED LED LOW
  PORTC &= ~(1 << 1); //CLEAR PC1 to 0
}

//Zero Array
void zeroArray() {
  for (int i = 0; i < 4; i++) {
    entered[i] = 0;
  }
}

char temp_buf[64];
char adc_buff[64];

volatile int overflow_counter = 0;

//ISR
ISR(TIMER0_OVF_vect) {
 overflow_counter++;
}

//------------------main process-----------------------------------//
void process() {
  while(1)
  {
    //LDR - Light dependent resistor 
    uint16_t LDR = ADC_read();
    
    //DEBUG - serial comms
    //convert LDR int to char array
    //itoa(LDR, (char *)adc_buff,10);
    //transmit LDR char array (adc_buff) to putty - the value of the the ADC. <100-130 = daytime
    //uart_putstring((unsigned char *) adc_buff);

    _delay_ms(150);

    //TO READ LIGHT LEVEL AT THE START
    //if LDR is reading >= 300 threshold (out of 1023 max), turn the back light on
    if (LDR >= 300)
    {
      PORTD |= (1 << 2);  //LCD BACK LIGHT ON
    }
    else
    {
      PORTD &= ~(1 << 2); //LCD BACK LIGHT OFF
    }

    //clear lcd buffer
    clear_screen();
    //write first message
    Lcd_Begin();
    //write second message
    Lcd_EnterNum();

    //A WHILE LOOP TO ACQUIRE 4 KEY PRESSES BEFORE ANYTHING ELSE HAPPENS

    int four_keys_yet = 0;
    while(four_keys_yet < 4)
    {

      //storage array for time
      char time_buf[64];

      ////compute elapsed time
      double time = ( overflow_counter * 256.0 + TCNT0 ) * PRESCALE / FREQ;

      //convert float to a string
      dtostrf(time, 7,3,time_buf);

      //DEBUG - serial comms for time_buf
      uart_putstring((unsigned char *) time_buf);
      uart_putchar('\n');

      //Show buffer to lcd - Enter your 4 number code -
      show_screen();

      /* Read the current state of the keypad */
      (Key) = Read_Keypad();
    
      //DEBUG
      //if I take this away some of the text does not display correctly.
      lcd_init(LCD_DEFAULT_CONTRAST);

      //DEBUG
      //this delay is to show the number on the screen - without it the number does not show.
      _delay_ms(150);

      //LDR2 constantly reads the ADC value while in this loop - enables back light to turn on and off quickly
      uint16_t LDR2 = ADC_read();

      if (LDR2 >= 300)
      {
        PORTD |= (1 << 2);  //LCD BACK LIGHT ON
      }
      else
      {
        PORTD &= ~(1 << 2); //LCD BACK LIGHT OFF
      }

      //DEBUG - serial comms
      //convert LDR int to char array
      //itoa(LDR2, (char *)adc_buff,10);
      //transmit LDR2 char array (adc_buff) to putty - the value of the the ADC. <100-130 = daytime > 400 dark
      //uart_putstring((unsigned char *) adc_buff);

      /* If a key has been pressed output it to the LCD */

      if ((Key) != 0) // Enter all except for 0 
      {
        //reset overflow_counter - resets the timer
        overflow_counter = 0;

        //DEBUG - testing uart comms
        //char sent_char = 'a';
        //send serial data
	      //uart_putchar(sent_char);

        //TO SEND KEY(keypad pressed number) TO THE SERIAL FOR DEBUGGING
        //itoa(Key, (char *)temp_buf,10);
        //send serial data
        //uart_putstring((unsigned char *) temp_buf);
    
        //draw number into buffer and print to LCD
        if (four_keys_yet == 0)
        {
          draw_int(0,30, Key);
        }
        else if (four_keys_yet == 1)
        {
          draw_int(16,30, Key);
        }
        else if (four_keys_yet == 2)
        {
          draw_int(32,30, Key);
        }
        else if (four_keys_yet == 3)
        {
          draw_int(44,30, Key);
        }

        //Make sound
        chirp();

        //this[i] is = key
        entered[four_keys_yet] = Key; 

        //delay 
        _delay_ms(300);
    
        //increment four_keys_yet
        four_keys_yet++;
      }
      
      //MAXIMUM TIME EXCEEDED TO ENTER A KEY -> TIMEOUT -> RESTART
      else if (time > timeout )
      {
        //RESET to the start of PROCESS and RESET overflow_counter/timer to 0
        overflow_counter = 0;
        process();
      }

    }
    //end while(four_keys_yet < 4)

    //reset four_keys_yet
    four_keys_yet = 0;
    
    //reset correct
    correct = 0;

    //for loop to check if the correct number is entered 
    for (int i = 0; i < 4; i++)
    {
      _delay_ms(250);

      //if the correct code is entered ++ correct
      if (sequence[i] == entered[i])
      {
        correct++;
        _delay_ms(250);
      }

    }

    //If all 4 numbers are correct - make sound flash led green
    if (correct == 4)
    {
      _delay_ms(500);
      chirpchirp();
      correct = 0;
      badCode = 0;
    }

    //incorrect number in entered numbers wrong code - make sound flash led red
    else
    {
      wrongCode();
      badCode++;
    }

    //if bad code entered 5 times sound alarm
    if (badCode > 4)
    {
      soundAlarm();
      badCode = 0;
    }
    _delay_ms(1000);

    //reset array for next entry
    zeroArray();

    //display screen buffer
    show_screen();
  }
  //end - forever loop
}
//end - process function

//main function - forever loops process
/*------------------------------------------------------------------*/
int main() {
	setup();
	for ( ; ; ){
		process();
		_delay_ms(50);
	}
	return 0;
}
/*--------------------------------------------------------------------*/