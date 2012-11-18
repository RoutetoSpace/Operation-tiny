/*

This piece of code is to create a checksum based on the RFC 1145.
This is to be used to determine the checksums for Ublox 6 configuration codes.

*/

long sting;
int stinlng;
long stingarr[];
void setup()
{
  Serial.begin (9600);
}


