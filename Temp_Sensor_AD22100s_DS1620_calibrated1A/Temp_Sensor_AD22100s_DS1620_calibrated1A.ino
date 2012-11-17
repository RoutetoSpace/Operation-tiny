#include <ds1620.h>

//Temporature sensor for AD22100S
int AD22100S = 0; //define temp sensor pin
int val;
double Valout;
unsigned int ADCValue;
double Voltage;
double Vcc;
double Vout;
double conts = 4.882;
//float cont = 23.875;
//float cont = 22.5;
//float cont = 29.375;

// Define pins for 3-wire serial communication
int dq = 5;
int clk = 4;
int rst = 3;

// Call DS1620 constructor using pin variables
DS1620 d = DS1620(dq, clk, rst);



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
  // Write TH and TL setpoints to DS1620 EEPROM  
  // Settings are retained even with no power
  d.write_th(30);
  d.write_tl(15);
  
  // Write config to DS1620 configuration/status register
  // Decimal 10 = Binary 00001010
  // Enables CPU-Mode and disables 1-Shot Mode.
  // See Datasheet for details
  d.write_config(10);
  
  // Start continuous temperature conversion
  // Readings can be read about once per second this way
  d.start_conv();

  
}
//long result = readVcc();
void loop()
{
  //val = analogRead(AD22100S); //Reads data from the AD22100S pin
  Vcc = readVcc()/1000.0;
  ADCValue = analogRead(AD22100S);
  Vout = ADCValue * conts;
  Voltage = (ADCValue / 1023.0) * Vcc;  
  //Valout = map(ADCValue, 0, 1024, -50, 150); //maps the analogue read output range to the temperature range
  Valout = (Vout/22.5)-61.11111111;
  Serial.print(Vcc); Serial.print("\t"); Serial.println("Vcc");
  Serial.print(Vout); Serial.print("\t"); Serial.println("Vout");
  Serial.print(Valout); Serial.print("\t"); Serial.println("Deg C AD2210");
  Serial.print(Voltage); Serial.print("\t"); Serial.println("mV");
  Serial.print(ADCValue); Serial.print("\t"); Serial.println("Value from pin");
  Serial.print(d.read_temp()); Serial.print("\t"); Serial.println("Deg C DS1620");
  
  Serial.println(" ");
  delay(1500);
}
