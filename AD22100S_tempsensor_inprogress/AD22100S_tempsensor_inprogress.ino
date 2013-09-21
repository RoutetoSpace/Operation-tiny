//Vcc global declaration
long VccResult = NULL;
float Vcc = NULL;

//Temp Sensor declarations
int AD22100S = A0; //define temp sensor pin for AD22100S
double AD22100Sout;
unsigned int ADCValue;
double ADCVout;


void setup() {
  Serial.begin(9600);

}

void loop() {
  readVcc();
  IntTemp(); 
  
  Serial.print(VccResult); Serial.print("\t"); Serial.println("VccResult");
  Serial.print(Vcc); Serial.print("\t"); Serial.println("Vcc");
  Serial.print(AD22100Sout); Serial.print("\t"); Serial.println("Deg C AD2210");
  
  
 delay(2500);
}

long readVcc() 
  {
          //Read the 1.1V reference against AVcc
          ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); //ADMUX analog voltage reference byte
          delay(2); // Wait for Vref to settle
          ADCSRA |= _BV(ADSC); // Start conversion
          while (bit_is_set(ADCSRA,ADSC)); // measuring 
          VccResult = ADCL;
          VccResult |= ADCH<<8;
          VccResult = 1125300L / VccResult; //Back calculate AVcc in mV
                    
  }
  
void IntTemp()
  {
  ADCValue = analogRead(AD22100S); //Reads data from the AD22100S pin
  ADCVout = (ADCValue / 1023.0) * VccResult;  
  AD22100Sout = (ADCVout/22.5)-63.11111111; //previous value was 61.11111 in line with formula from tech report.  Calibration against house themostat required 3 degrees less. Changed Vout to Voltage to take in to account voltage fultuations  
  
}
  
