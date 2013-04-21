#include <string.h>
#include <iostream.h>

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  
}

void loop() 
{
  lineprt();
  delay(2500);  
 }
  
void lineprt(){
  Serial.println("RTSHAB01 ");
  Serial.println("High alititude tracker for use in habbing, developed by Chris Atherton ");
  Serial.println("NTX2 code from UPU ");
  Serial.println("TinyGPS code from Mikal Hart ");
  Serial.println("Checksum code from Lunar_Lander ");
  Serial.println("http://www.RoutetoSpace.com ");
  Serial.println();
}
