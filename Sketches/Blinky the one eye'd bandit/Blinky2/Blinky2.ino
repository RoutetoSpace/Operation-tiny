/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int array[6] = {2, 4, 3, 8, 3, 1};
int size = sizeof(array);
int i = 0;
int x;


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  for (x=0; x < size -1;){
    if (i = array[x])
      {
        digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(array[x] * 100);               // wait for value of array x 100
        digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
        delay(array[x] * 100);               // wait for value of array x 100
        x++;                       // skip to the next value in the array
        i = 0;                               //reset i 
      }
      else
      {
       i++;
      }
  }
}
