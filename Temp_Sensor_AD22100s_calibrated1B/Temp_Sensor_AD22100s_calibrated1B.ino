//Temporature sensor for AD22100S
int AD22100S = 0; //define temp sensor pin
int val;
int Valout;
int Valout2;
unsigned int ADCValue; //ADC = analog digital converter
double Voltage; //Voltage of the analog pins when compared against the internal voltage source. 
double Vcc;
//float cont = 23.875;
//float cont = 22.5;
//float cont = 29.375;

//Voltage meter reading gained by comparing the external voltage with the internal 1.1V
long readVcc() {
  long result;
  //Read the 1.1V reference against AVcc
  //ADMUX analog voltage reference byte
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring 
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; //Back calculate AVcc in mV
  // uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH ADC=Analog digital converter 
  // uint8_t high = ADCH; // unlocks both
 
  //  long result = (high<<8) | low;
 
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
void setup()
{
  Serial.begin(9600);//setup Serial Library at 9600 bps
}
//long result = readVcc();
void loop()
{
  //val = analogRead(AD22100S); //Reads data from the AD22100S pin
  Vcc = readVcc()/1000.0;
  ADCValue = analogRead(AD22100S);
  Voltage = (ADCValue / 1023.0) * Vcc;  
  Valout = map(ADCValue, 0, 1024, -50, 150); //maps the analogue read output range to the temperature range
  Valout2 = map(Voltage, 0, 1024, -50, 150); //maps the corrected analogue read output to the temp range
  Serial.print(Vcc); //print the Vcc value
  Serial.print("\t");
  Serial.println("Vcc");
  Serial.print(Valout);
  Serial.print("\t");
  Serial.println("Deg C");
  Serial.print(Voltage);
  Serial.print("\t");
  Serial.println("mV");
  Serial.print(Valout2);
  Serial.print("\t");
  Serial.println("Deg C");
  Serial.print(ADCValue);
  Serial.print("\t");
  Serial.println("Value from pin");
  Serial.println(" ");
  delay(1500);
}
