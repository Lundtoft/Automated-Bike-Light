#include <avr/io.h>
#include "../pin_functions.h"
#include <util/delay.h>

int main(void) { 

   /**
   * LED PIN NUMBERS
   */
   int leftLED = 15;
   int rightLED = 18;
   int breakLED = 16;
   int rearLED = 17;

   /**
   *  ACCELEROMETER INSTANTIATIONS
   */
   int x;
   int y;

   /**
   * INPUT AND OUTPUT PINS
   */
   output(leftLED);
   output(rightLED);
   output(breakLED);
   output(rearLED);

   /**
   * INIT SERIAL CONNECTION WITH 9600 BAUD
   */
   serialInit(51);

   /**
   * COUNTER FOR FLASHING LIGHTS
   */
   int counter = 0;

   /**
   *  CALIBRATE ACCELEROMETER
   */
   int caX = analogRead(23);
   int caY = analogRead(24);
   int rightCounter = 0;
   int leftCounter = 0;

   int prevX = analogRead(23);

   /**
   *  MAIN LOOP
   */
   while(1) {
      counter++;
   
      //Read accelerometer values
      x = map(analogRead(23), caX - 50, caX + 50, 1, 50);
      y = map(analogRead(24), caY - 50, caY + 50, 1, 50);

      
      //BRAKE LIGHT SIGNAL
      if (y > 30) {
         digitalWrite(breakLED, HIGH);
         _delay_ms(1000);
         digitalWrite(breakLED, LOW);
      }
      
      //RIGHT LIGHT SIGNAL
      if (x > 26) {
         if (rightCounter < 4) rightCounter++;
         else digitalWrite(rightLED, TOGGLE);
      } else {
         if (rightCounter > 0) rightCounter--;
         else digitalWrite(rightLED, LOW);
      }

      //LEFT LIGHT SIGNAL
      if (x < 24) {
         if (leftCounter < 4) leftCounter++;
         else digitalWrite(leftLED, TOGGLE);
      } else {
         if (leftCounter > 0) leftCounter--;
         else digitalWrite(leftLED, LOW);
      }

      _delay_ms(100);

      //TURN LIGHTS ON/OFF ACCORDING TO AMBIENT LIGHT CONDITIONS
      if (serialCheck() == 0x68) {
         digitalWrite(rearLED, HIGH);
      } else if (serialCheck() == 0x6C) {
         digitalWrite(rearLED, LOW);
      } else if (serialCheck() == 0x62) {
         digitalWrite(breakLED, HIGH);
      } else if (serialCheck() == 0x63) {
         digitalWrite(breakLED, LOW);
      }
   }

}