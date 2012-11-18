/*

This piece of code is to create a checksum based on the RFC 1145.
This is to be used to determine the checksums for Ublox 6 configuration codes.

*/

//include string.h so that I can use the strtok_r function
#include <string.h>

//Idea is to have a script that will add each element of a hex string, seperated by commas, together to produce a final figure. 

//declare the variables
char sting;
int stingin;
char *sep = ", ";
char *wordd, *phrase, *brkt;

void setup()
{
  Serial.begin (9600);
}

void loop()
{
sting = ("0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00");
  
  //parse the string
  for (word = strtok_r(sting, sep, &brkt);
    printf(word);
}



