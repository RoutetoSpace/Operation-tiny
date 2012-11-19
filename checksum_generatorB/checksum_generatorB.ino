/*

This piece of code is to create a checksum based on the RFC 1145.
This is to be used to determine the checksums for Ublox 6 configuration codes.

*/

//include string.h so that I can use the strtok_r function

#include <string.h>

//Idea is to have a script that will add each element of a hex string, seperated by commas, together to produce a final figure. 
//declare variables
//char *sting()
String frame = "0x06,0x24,0x24,0x00";
int inchar = ","; 
char *input[4];
int stingin;
char *sep = ", ";
char *wor, *phrase, *brkt, *brkb;
int i;

void setup()
{
  Serial.begin (9600);
}

void loop()
{
  Serial.println(frame);
  while (inchar !=","){inchar = Serial.read();}  //wait for the start of the messages
  if(inchar ==",")
  {
    while(Serial.available() < 5) //wait for 5 characters
	{ 
      ;
    }
    for (int i=0; i < 5; i++)
	{
      input[i] = Serial.read(); //Read charactors into an array
    }
   
  }
  Serial.println(input[1]);
}




