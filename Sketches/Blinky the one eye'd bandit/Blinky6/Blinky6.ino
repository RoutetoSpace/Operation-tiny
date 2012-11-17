/*
  Blinky the one eye'd bandit
  Turns on an LED based on a series of values in an array, repeatedly.
*/
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int array[6] = {2, 4, 9, 5, 3, 7};
int size = sizeof(array);
int i = 0;
int y;
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}
// the loop routine runs over and over again forever:
void loop() {
  for (int x = 0; x = size -1;){  //define x and limit x to 1 less the size of the array
    if (i = array[x])             //iffy bit, if the value in the array matches the value of i
      {
        y = i * 1000;      //defin the value of i * 1000
        digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(y);               // wait for value of array in seconds
        digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
        delay(9000);               // wait for value of array x 100
        x++;
        i = 0;                               //reset i
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

