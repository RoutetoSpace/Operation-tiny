/*  NTX2 Radio with temp sensor.  Adapted from NTX2 Radio Test Part 2 by Rob Harrison and M0UPU
 
    Transmits temp via RTTY with a checksum.
 
    By C Atherton 2012 - www.routetospace.com
*/ 
 
#define RADIOPIN 13
#define AD22100S 0 //define temp sensor pin
#include <string.h>
#include <util/crc16.h>
 
char datastring[80];
int Valout;
unsigned int ADCValue; //ADC = analog digital converter
 
void setup() {                
  pinMode(RADIOPIN,OUTPUT);
}
 
void loop() {
 
 
  sprintf(datastring,"RTTY TEST BEACON RTTY TEST BEACON"); // Puts the text in the datastring
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
 
  rtty_txstring (datastring);
  delay(2000);
}
 
 
void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first	
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0);	
 
    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);
 
  }
 
  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
                            // largest value that will produce an accurate delay is 16383
                            // See : http://arduino.cc/en/Reference/DelayMicroseconds
 
}
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}    
