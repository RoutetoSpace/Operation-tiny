/* Landing lights without delay

The aim is to make a series of LED's (connected to the digital pins)
turn on then off in a specified order without using the delay button.
This means other code can be run while the landing lughts are running.

The circuit:

LEDs attached from pins 2 - 7 to ground.

Created 2012
by Chris Atherton

With thanks to David A. Mellis and Paul Stoffregen who gave me the 
inspiration for this sketch.


This is example code and for the public domain.
*/

//There are no contants at the moment

//Variables to change
int ledPin = 2; 
//int ledState = LOW;  //led state used to set the LED
long previousMillis = 0; //Will store last time LED was updated

//The following variable is a long becaused the time, measured in miliseconds
//will quickly become a big number that can't be stored in an int
long interval = 1500;  //interval at which to blink

void setup (){
  //set the digital pin as output:
  pinMode(ledPin, OUTPUT);
}

void loop(){
  //This is where you put the code that need to run all the time

  //check to see if it's time to blink the next led.  If the
  //difference between the current time and the last time the
  //LED blinked is bigger than the interval at which you want
  //to blink the LED
  unsigned long currentMillis = millis();
  
  //if(currentMillis - previousMillis > interval) {
    //save the last time you blinked the LED
    //previousMillis = currentMillis;
    
   //if the LED is off turn it on and visa versa
   // if (ledState == LOW)
    //  ledState = HIGH;
   // else
   //   ledState = LOW;
   //   
  //  if (ledPin == 8)
   //   ledPin = 2;
      
      // Set the LED with the ledSate of the variable:
   for(ledPin; ledPin <= 7; ledPin++){
       if(currentMillis - previousMillis > interval)
          previousMillis = currentMillis;  //save the last time you blinked the LED      
            
       digitalWrite(ledPin, HIGH);
      }      
      
      for(ledPin; ledPin <= 7; ledPin++){
       if(currentMillis - previousMillis > interval)
          previousMillis = currentMillis;  //save the last time you blinked the LED      
            
       digitalWrite(ledPin, LOW);
      }
  if (ledPin == 8){
          ledPin = 2;      
  }  
}
