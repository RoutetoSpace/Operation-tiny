/*

This piece of code is to create a checksum based on the RFC 1145.
This is to be used to determine the checksums for Ublox 6 configuration codes.

*/

//include string.h so that I can use the strtok_r function

#include <string.h>

//Idea is to have a script that will add each element of a hex string, seperated by commas, together to produce a final figure. 
//declare variables
//char *sting()
void setup()
{
  Serial.begin (9600);
}

void loop()
{
  byte frame[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&frame[2], sizeof(frame));
  delay(2500);
} 

void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
  Serial.println(CK_A, HEX);
  Serial.println(CK_B, HEX);
}



