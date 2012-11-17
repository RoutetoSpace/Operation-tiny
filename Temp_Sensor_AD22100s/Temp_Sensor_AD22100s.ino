//Temporature sensor for AD22100S
int AD22100S = 0; //define temp sensor pin
int val;
int valout;
//float cont = 23.875;
//float cont = 22.5;
float cont = 29.375;

void setup()
{
  Serial.begin(9600);//setup Serial Library at 9600 bps
}
//long result = readVcc();
void loop()
{
  //val = analogRead(AD22100S); //Reads data from the AD22100S pin
  valout = map(analogRead(AD22100S), 0, 1024, -50, 150); //maps the analogue read output range to the temperature range
  Serial.println("TEMP");
  Serial.println(valout);
 // Serial.println("Vcc");
 // Serial.println(result);
  delay(150);
}
