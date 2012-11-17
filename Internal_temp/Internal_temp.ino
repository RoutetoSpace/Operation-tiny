#include <EEPROM.h>

#include <avr/pgmspace.h>
#include <ChipTemp.h>

ChipTemp chipTemp;

void setup() 
{ Serial.begin(9600);
  Serial.println("Celsius deciCelsius Fahrenheit deciFahrenheit");
}

void loop() 
{ delay(1000);
  Serial.print(chipTemp.celsius()); Serial.print(" "); 
  Serial.print(chipTemp.deciCelsius()); Serial.print(" "); 
  Serial.print(chipTemp.fahrenheit()); Serial.print(" "); 
  Serial.println(chipTemp.deciFahrenheit());
}
