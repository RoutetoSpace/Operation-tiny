#include <SPI.h>
#include <RFM22.h>
 
//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);


void setup()
{
 Serial.begin(9600);
}

void loop()
{
  //Quick test
  Serial.println("Before radio write");
  //Quick test  
  radio1.write(0x07, 0x08); // turn tx on
  delay(2500);
  radio1.write(0x07, 0x01); // turn tx off
  delay(1000);
  Serial.println("after radio write");
  
}
 
void setupRadio(){
 
  digitalWrite(5, LOW);
 
  delay(1000);
 
  rfm22::initSPI();
 
  radio1.init();
 
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
 
  radio1.setFrequency(434.201);
 
  
}
