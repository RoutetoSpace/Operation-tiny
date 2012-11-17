//Define pin 9 to communicate with the base pin
int tip120pin = 9;
int potpin = 0;
int val;
void setup()
{
  pinMode(tip120pin, OUTPUT); //set the base pin
}
void loop()
{
    val =  analogRead(potpin);   //Reads the value of the potentometer
    val = map(val, 0, 1023, 0 , 255); //scales the value to that of the fan
    analogWrite(tip120pin, val); //Writes the value of val to the fan
    delay(15); //Give the fan time to catch up
}
