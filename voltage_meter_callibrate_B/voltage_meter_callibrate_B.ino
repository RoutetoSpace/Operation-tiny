long readVcc() {
  long result;
  //Read the 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); //ADMUX analog voltage reference byte
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
void loop()
{
  Serial.println(readVcc(), DEC );
 // long res = readVcc(); //call the result from the readVcc function
 // long scale_constant = internal1.1Ref * 1023 * 1000
 // where
 // internatl1.1Ref = 1.1 * 5130 / res  
 // Serial.println(scale_constant); //print to the screen
  delay(150); //wait 150 mili seconds before doing it again
}
