#include <avr/io.h>
#include "pin_functions.h"
#include <util/delay.h>
#include "lcd.c"
#include "lcd.h"

int main(void){
   
   /**
   * INIT DISPLAY AND CHARACTERS FOR DISPLAY
   */
   lcd_init(LCD_DISP_ON);
   char String[10];
   char distString[10];

   /**
   * SET PIN NUMBERS
   */
   int led = 13;
   int speaker = 12;
   int reed = 11;
   int light = 24;
   int ping = 23;
   int timer = 12;

   /**
   * SET I/O
   */
   output(led);
   output(speaker);
   output(timer);
   input(reed);

   /**
   * INIT COUNT AND STAND STILL CHECKER FOR REED SENSOR
   */
   int count = 0;
   int still = 0;
   int lux = 0;
   double dist = 0.0;
   int a,b;
   int speakerCount = 0;

   /**
   * INIT PING SENSOR BORDER
   */
   int maxDist = 158;

   int breaking = 0;
   int night = 0;

   serialInit(51);

   while(1){

      speakerCount++;
   
      //Start speaker and send serial signal if border is crossed
      if (analogRead(ping) < maxDist) {
         if (breaking == 0) {
            digitalWrite(speaker, HIGH);
            serialWrite('b');
            breaking = 1;
         }
      } else {
         if (breaking == 1) {
            digitalWrite(speaker, LOW);
            serialWrite('c');
            breaking = 0;
         }
      }
      
      //Check reed sensor
      if(digitalRead(reed)){
        if(still == 0){
            count++;
            still = 1;
         }
      }
      else{
         still = 0;
      }
      
      //Turn lights on when it's dark
      lux = map(analogRead(light), 450, 920, 0, 10);
      if(lux < 4){
         if (night == 0) {
            digitalWrite(led, HIGH);
            serialWrite('h');
            night = 1;
         }
      } else{
         if (night == 1) {
            digitalWrite(led, LOW);
            serialWrite('l');
            night = 0;
         }
      }

      //216 cm / omgang
      dist = count * 2.16;

      //Convert the dist to two ingegers
      a = floor(dist);
      b = dist * pow(10,2) - a * pow(10,2);

      //Clear LCD display
      lcd_clrscr();
      //Write to LCD display
      //lcd_puts(itoa(count, String, 10));
      lcd_puts(itoa(a, String, 10));
      lcd_puts(".");
      lcd_puts(itoa(b, String, 10));
      lcd_puts(" m");
      
      _delay_ms(100);
   }
	
}