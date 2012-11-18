/*

This piece of code is to create a checksum based on the RFC 1145.
This is to be used to determine the checksums for Ublox 6 configuration codes.

*/

#include <string.h>

//Idea is to have a script that will add each element of a hex string, seperated by commas, together to produce a final figure. 
char *sting(){
  char frame[] = {0x06, 0x24, 0x24, 0x00};
  return frame;
}
int stingin;
char *sep = ", ";
char *wor, *phrase, *brkt, *brkb;

void setup()
{
  Serial.begin (9600);
}

void loop()
{
  char *frame = sting();
  for (wor = strtok_r(frame, sep, &brkt);
       wor;
       wor = strtok_r(NULL, sep, &brkb))
     { 
    printf("%s",wor);
     }
}




