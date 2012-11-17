/*
  Blinky the one eye'd bandit
  Turns on an LED based on a series of values in an array, repeatedly.
*/
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int array[6] = {22, 4, 13, 8, 3, 19};
int size = sizeof(array);
int i = 0;



// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  for (int x = 0; x = size -1;){
    if (i = array[x])
      {
        digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(array[x] * 100);               // wait for value of array x 100
        digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
        delay(array[x] * 100);               // wait for value of array x 100
        i = 0;                               //reset i
        x++;
      }
      else
      {
       i++;
      }
   }
   for (int x = size; x > size -1;)
   {
     x=0;
   } 
}

