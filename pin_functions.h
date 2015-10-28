#include <stdint.h> // needed for uint8_t
#include <inttypes.h>

#define HIGH 1
#define LOW 0
#define TOGGLE 3

void output(int pin){
  //Left side of chip
  if(pin == 1) DDRC |= (1<<DDC6);
  if(pin == 2) DDRD |= (1<<DDD0);
  if(pin == 3) DDRD |= (1<<DDD1);
  if(pin == 4) DDRD |= (1<<DDD2);
  if(pin == 5) DDRD |= (1<<DDD3);
  if(pin == 6) DDRD |= (1<<DDD4);
  //Pin 7 = VCC
  //Pin 8 = GND
  if(pin == 9) DDRB |= (1<<DDB6);
  if(pin == 10) DDRB |= (1<<DDB7);
  if(pin == 11) DDRD |= (1<<DDD5);
  if(pin == 12) DDRD |= (1<<DDD6);
  if(pin == 13) DDRD |= (1<<DDD7);
  if(pin == 14) DDRB |= (1<<DDB0);

  //Right side of chip
  if(pin == 15) DDRB |= (1<<DDB1);
  if(pin == 16) DDRB |= (1<<DDB2);
  if(pin == 17) DDRB |= (1<<DDB3);
  if(pin == 18) DDRB |= (1<<DDB4);
  if(pin == 19) DDRB |= (1<<DDB5);
  //Pin 20 = AVCC
  //Pin 21 = AREF
  //Pin 22 = GND
  if(pin == 23) DDRC |= (1<<DDC0);
  if(pin == 24) DDRC |= (1<<DDC1);
  if(pin == 25) DDRC |= (1<<DDC2);
  if(pin == 26) DDRC |= (1<<DDC3);
  if(pin == 27) DDRC |= (1<<DDC4);
  if(pin == 28) DDRC |= (1<<DDC5);
}

void input(int pin){
  //Left side of chip
  if(pin == 1) DDRC &= ~(1<<PORTC6);
  if(pin == 2) DDRD &= ~(1<<PORTD0);
  if(pin == 3) DDRD &= ~(1<<PORTD1);
  if(pin == 4) DDRD &= ~(1<<PORTD2);
  if(pin == 5) DDRD &= ~(1<<PORTD3);
  if(pin == 6) DDRD &= ~(1<<PORTD4);
  //Pin 7 = VCC
  //Pin 8 = GND
  if(pin == 9) DDRB &= ~(1<<PORTB6);
  if(pin == 10) DDRB &= ~(1<<PORTB7);
  if(pin == 11) DDRD &= ~(1<<PORTD5);
  if(pin == 12) DDRD &= ~(1<<PORTD6);
  if(pin == 13) DDRD &= ~(1<<PORTD7);
  if(pin == 14) DDRB &= ~(1<<PORTB0);

  //Right side of chip
  if(pin == 15) DDRB &= ~(1<<PORTB1);
  if(pin == 16) DDRB &= ~(1<<PORTB2);
  if(pin == 17) DDRB &= ~(1<<PORTB3);
  if(pin == 18) DDRB &= ~(1<<PORTB4);
  if(pin == 19) DDRB &= ~(1<<PORTB5);
  //Pin 20 = AVCC
  //Pin 21 = AREF
  //Pin 22 = GND
  if(pin == 23) DDRC &= ~(1<<PORTC0);
  if(pin == 24) DDRC &= ~(1<<PORTC1);
  if(pin == 25) DDRC &= ~(1<<PORTC2);
  if(pin == 26) DDRC &= ~(1<<PORTC3);
  if(pin == 27) DDRC &= ~(1<<PORTC4);
  if(pin == 28) DDRC &= ~(1<<PORTC5);
}

void digitalWrite(int pin, int state){
  if(state == HIGH){
    //Left side of chip
    if(pin == 1) PORTC |= (1<<PORTC6);
    if(pin == 2) PORTD |= (1<<PORTD0);
    if(pin == 3) PORTD |= (1<<PORTD1);
    if(pin == 4) PORTD |= (1<<PORTD2);
    if(pin == 5) PORTD |= (1<<PORTD3);
    if(pin == 6) PORTD |= (1<<PORTD4);
    //Pin 7 = VCC
    //Pin 8 = GND
    if(pin == 9) PORTB |= (1<<PORTB6);
    if(pin == 10) PORTB |= (1<<PORTB7);
    if(pin == 11) PORTD |= (1<<PORTD5);
    if(pin == 12) PORTD |= (1<<PORTD6);
    if(pin == 13) PORTD |= (1<<PORTD7);
    if(pin == 14) PORTB |= (1<<PORTB0);

    //Right side of chip
    if(pin == 15) PORTB |= (1<<PORTB1);
    if(pin == 16) PORTB |= (1<<PORTB2);
    if(pin == 17) PORTB |= (1<<PORTB3);
    if(pin == 18) PORTB |= (1<<PORTB4);
    if(pin == 19) PORTB |= (1<<PORTB5);
    //Pin 20 = AVCC
    //Pin 21 = AREF
    //Pin 22 = GND
    if(pin == 23) PORTC |= (1<<PORTC0);
    if(pin == 24) PORTC |= (1<<PORTC1);
    if(pin == 25) PORTC |= (1<<PORTC2);
    if(pin == 26) PORTC |= (1<<PORTC3);
    if(pin == 27) PORTC |= (1<<PORTC4);
    if(pin == 28) PORTC |= (1<<PORTC5);
  }

  if(state == LOW){
    //Left side of chip
    if(pin == 1) PORTC &= ~(1<<PORTC6);
    if(pin == 2) PORTD &= ~(1<<PORTD0);
    if(pin == 3) PORTD &= ~(1<<PORTD1);
    if(pin == 4) PORTD &= ~(1<<PORTD2);
    if(pin == 5) PORTD &= ~(1<<PORTD3);
    if(pin == 6) PORTD &= ~(1<<PORTD4);
    //Pin 7 = VCC
    //Pin 8 = GND
    if(pin == 9) PORTB &= ~(1<<PORTB6);
    if(pin == 10) PORTB &= ~(1<<PORTB7);
    if(pin == 11) PORTD &= ~(1<<PORTD5);
    if(pin == 12) PORTD &= ~(1<<PORTD6);
    if(pin == 13) PORTD &= ~(1<<PORTD7);
    if(pin == 14) PORTB &= ~(1<<PORTB0);

    //Right side of chip
    if(pin == 15) PORTB &= ~(1<<PORTB1);
    if(pin == 16) PORTB &= ~(1<<PORTB2);
    if(pin == 17) PORTB &= ~(1<<PORTB3);
    if(pin == 18) PORTB &= ~(1<<PORTB4);
    if(pin == 19) PORTB &= ~(1<<PORTB5);
    //Pin 20 = AVCC
    //Pin 21 = AREF
    //Pin 22 = GND
    if(pin == 23) PORTC &= ~(1<<PORTC0);
    if(pin == 24) PORTC &= ~(1<<PORTC1);
    if(pin == 25) PORTC &= ~(1<<PORTC2);
    if(pin == 26) PORTC &= ~(1<<PORTC3);
    if(pin == 27) PORTC &= ~(1<<PORTC4);
    if(pin == 28) PORTC &= ~(1<<PORTC5);
  }

  if(state == TOGGLE){
    //Left side of chip
    if(pin == 1) PORTC ^= (1<<PORTC6);
    if(pin == 2) PORTD ^= (1<<PORTD0);
    if(pin == 3) PORTD ^= (1<<PORTD1);
    if(pin == 4) PORTD ^= (1<<PORTD2);
    if(pin == 5) PORTD ^= (1<<PORTD3);
    if(pin == 6) PORTD ^= (1<<PORTD4);
    //Pin 7 = VCC
    //Pin 8 = GND
    if(pin == 9) PORTB ^= (1<<PORTB6);
    if(pin == 10) PORTB ^= (1<<PORTB7);
    if(pin == 11) PORTD ^= (1<<PORTD5);
    if(pin == 12) PORTD ^= (1<<PORTD6);
    if(pin == 13) PORTD ^= (1<<PORTD7);
    if(pin == 14) PORTB ^= (1<<PORTB0);

    //Right side of chip
    if(pin == 15) PORTB ^= (1<<PORTB1);
    if(pin == 16) PORTB ^= (1<<PORTB2);
    if(pin == 17) PORTB ^= (1<<PORTB3);
    if(pin == 18) PORTB ^= (1<<PORTB4);
    if(pin == 19) PORTB ^= (1<<PORTB5);
    //Pin 20 = AVCC
    //Pin 21 = AREF
    //Pin 22 = GND
    if(pin == 23) PORTC ^= (1<<PORTC0);
    if(pin == 24) PORTC ^= (1<<PORTC1);
    if(pin == 25) PORTC ^= (1<<PORTC2);
    if(pin == 26) PORTC ^= (1<<PORTC3);
    if(pin == 27) PORTC ^= (1<<PORTC4);
    if(pin == 28) PORTC ^= (1<<PORTC5);
  }

}

int digitalRead(int pin){
  //Left side of chip
  if(pin == 1) return PINC & (1<<PORTC6);
  if(pin == 2) return PIND & (1<<PORTD0);
  if(pin == 3) return PIND & (1<<PORTD1);
  if(pin == 4) return PIND & (1<<PORTD2);
  if(pin == 5) return PIND & (1<<PORTD3);
  if(pin == 6) return PIND & (1<<PORTD4);
  //Pin 7 = VCC
  //Pin 8 = GND
  if(pin == 9) return PINB & (1<<PORTB6);
  if(pin == 10) return PINB & (1<<PORTB7);
  if(pin == 11) return PIND & (1<<PORTD5);
  if(pin == 12) return PIND & (1<<PORTD6);
  if(pin == 13) return PIND & (1<<PORTD7);
  if(pin == 14) return PINB & (1<<PORTB0);

  //Right side of chip
  if(pin == 15) return PINB & (1<<PORTB1);
  if(pin == 16) return PINB & (1<<PORTB2);
  if(pin == 17) return PINB & (1<<PORTB3);
  if(pin == 18) return PINB & (1<<PORTB4);
  if(pin == 19) return PINB & (1<<PORTB5);
  //Pin 20 = AVCC
  //Pin 21 = AREF
  //Pin 22 = GND
  if(pin == 23) return PINC & (1<<PORTC0);
  if(pin == 24) return PINC & (1<<PORTC1);
  if(pin == 25) return PINC & (1<<PORTC2);
  if(pin == 26) return PINC & (1<<PORTC3);
  if(pin == 27) return PINC & (1<<PORTC4);
  if(pin == 28) return PINC & (1<<PORTC5);

  if(pin == 15) return PINB & (1<<PORTB1);
  return 0; //If pin not on microcontroller
}

int analogRead(uint8_t adctouse){
    int ADCval;
    adctouse = adctouse - 23;
    ADMUX = adctouse;         // use #1 ADC
    ADMUX |= (1 << REFS0);    // use AVcc as the reference
    ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution
    
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
    ADCSRA |= (1 << ADEN);    // Enable the ADC

    ADCSRA |= (1 << ADSC);    // Start the ADC conversion

    while(ADCSRA & (1 << ADSC));      // Thanks T, this line waits for the ADC to finish 


    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;    // ADCH is read so ADC can be updated again

    return ADCval;
}

void analogWrite(int pin, uint8_t pwmAmount){

  //Set the timer counter control registers for fast PWM
  // TCCR1B |= (1<<WGM12);
  // TCCR1A |= (1<<WGM10);
  // TCCR2 |= (1<<WGM21);
  // TCCR2 |= (1<<WGM20);

  //TCCR1B |= (1<<CS11); //Set the prescaler to clock divided by 8

  //ICR1 = 10000; //Set the upper limit of timer1, will generate 50Hz

  //Set the PWM amount on the selected pin (atmega8 only has 3 PWM pins)
  if(pin == 15) OCR1A = pwmAmount;
  if(pin == 16) OCR1B = pwmAmount;
  if(pin == 17) {
    TCCR2 |= (1<<WGM21); //Set timers to 8 bit fast PWM
    TCCR2 |= (1<<WGM20);
    TCCR2 |= (1<<COM21); //Set to non-inverting
    OCR2 = pwmAmount;
  } 
}

void servoInit(int pin){
  if(pin == 15){
    TCCR1A |= (1<<COM1A1) | (1<<WGM11); // non-inverting mode for OC1A
    TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Mode 14, Prescaler 8

    ICR1 = 10000;

    DDRB |= (1<<PB1); // OC1A set to output
  }
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servoAngle(int pin, int angle){
  angle = map(angle, 0, 180, 544, 2400); //map the angle to the servos high and low
  if(pin == 15) OCR1A = angle;
}

void serialInit(uint16_t ubrr_value){
  //Set Baud rate
  //ubbr = (8MHz/16*baud)-1
  //uint16_t ubrr_value = (8000000 / 16 * baud) - 1;

  UBRRL = ubrr_value; //Set to 51 for 9600 baud on 8 MHz
  UBRRH = (ubrr_value>>8);
  /*Set Frame Format
  >> Asynchronous mode
  >> No Parity
  >> 1 StopBit

  >> char size 8
  */

  UCSRC=(1<<URSEL)|(3<<UCSZ0);

  //Enable The receiver and transmitter
  UCSRB=(1<<RXEN)|(1<<TXEN);
}

char serialRead(){
  //Wait untill a data is available
  while(!(UCSRA & (1<<RXC))){
    //Do nothing
  }
  //Now USART has got data from host
  //and is available is buffer
  return UDR;
}

char serialCheck(){
  if ((UCSRA & (1<<RXC))) {
    return UDR;
  }
}

void serialWrite(char data){
   //Wait until the transmitter is ready
   while(!(UCSRA & (1<<UDRE))){
    //Do nothing
   }
   UDR=data; //Now write the data to USART buffer
}

void serialWriteString(char* StringPtr){

  while(*StringPtr != 0x00){
    serialWrite(*StringPtr);
    StringPtr++;
  }
  serialWrite('\r');
  serialWrite('\n');

}

void serialWriteInt(int integer){

  char String[10];
  char* StringPtr = itoa(integer, String, 10);

  while(*StringPtr != 0x00){
    serialWrite(*StringPtr);
    StringPtr++;
  }
  serialWrite('\r');
  serialWrite('\n');
  

}