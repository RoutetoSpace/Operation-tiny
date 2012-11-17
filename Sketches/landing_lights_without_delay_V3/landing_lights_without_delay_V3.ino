/*
  For Loop Iteration
 
 Demonstrates the use of a for() loop. 
 Lights multiple LEDs in sequence, then in reverse.
 
 The circuit:
 * LEDs from pins 2 through 7 to ground
 
 created 2006
 by David A. Mellis
 modified 30 Aug 2011
 by Tom Igoe 

This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/ForLoop
 */

long timer = 150;           // The higher the number, the slower the timing.
long previousMillis = 0;
unsigned long currentMillis = millis();

void setup() {
  // use a for loop to initialize each pin as an output:
  for (int thisPin = 2; thisPin < 8; thisPin++)  {
    pinMode(thisPin, OUTPUT);      
  }
}

void loop() {
  // loop from the lowest pin to the highest:
  for (int thisPin = 2; thisPin < 8; thisPin++) { 
    // turn the pin on:
    if (currentMillis - previousMillis > timer)
        previousMillis = currentMillis; 
    digitalWrite(thisPin, HIGH);                   
    if (currentMillis - previousMillis > timer)
        previousMillis = currentMillis;    
    // turn the pin off:
    digitalWrite(thisPin, LOW);    
  }

  // loop from the highest pin to the lowest:
 // for (int thisPin = 7; thisPin >= 2; thisPin--) { 
    // turn the pin on:
 //   digitalWrite(thisPin, HIGH);
 //   delay(timer);
    // turn the pin off:
 //   digitalWrite(thisPin, LOW);
//  }
}
