//Temporature sensor for AD22100S with the internal temp sensor for comparison
int AD22100S = 0; //define temp sensor pin
int val;
float valout;
//float cont = 23.875;
//float cont = 22.5;
float cont = 29.375;

void setup()
{
  Serial.begin(9600);//setup Serial Library at 9600 bps
}
void loop()
{
  val = analogRead(AD22100S); //Reads data from the AD22100S pin
  valout = val/cont;
  Serial.println(("AD22100S ")) valout ("C Internal ") GetTemp() ("C"));//print the value of the AD22100S compared with Internal sensor
  delay(150);
}
double GetTemp(void)
{
  unsigned int wADC;
  double t;
  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.
  
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}
  
