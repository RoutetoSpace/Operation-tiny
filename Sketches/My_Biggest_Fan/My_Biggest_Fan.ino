//Define pin 9 to communicate with the base pin
int tip120pin = 9;
void setup()
{
  pinMode(tip120pin, OUTPUT); //set the base pin
  analogWrite(tip120pin, 254); //change motor speed by changing values from 0 to 255
}
void loop()
{
}
