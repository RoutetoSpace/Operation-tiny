/*  NTX2 Radio with GPS.  Adapted from NTX2 Radio Test Part 2 by Rob Harrison and M0UPU http://ukhas.org.uk/guides:linkingarduinotontx2
    GPS code adapted from 2E0UPU and James Coxon http://ukhas.org.uk/guides:ublox6
    Additional help from chris-stubbs http://paste.chris-stubbs.co.uk/qbfgdNMu
    
    Transmits GPS via RTTY with a checksum.
 
    By C Atherton 2013 - www.routetospace.com
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 

//Includes section
 
#include <TinyGPS.h>
#include <string.h>
#include <util/crc16.h>
#include <SoftwareSerial.h>

//Definition section
#define RADIOPIN 13 

//GPS declarations sestion
TinyGPS GPS;
SoftwareSerial GPS_SS(3, 4);  //GPS RX -> Digital pin 3, GPS TX -> Digital pin 4 
int heading;
int velocity;
int sats;
char timechara[9];
int alt;
    char latstr[10] = "0";
    char lonstr[10] = "0";

 
//Data string declaration 
char datastring[200];

int msgcount = 0;
 
void setup() {                
  pinMode(RADIOPIN,OUTPUT);
  delay(5000); //allows the GPS to settle
  
  //Start the serial and GPS serial
  Serial.begin(9600);
  GPS_SS.begin(9600);
  
  //Change GPS baud rate to 4800
  GPS_SS.print("$PUBX,41,1,0007,0003,4800,0*13rn");
  GPS_SS.begin(4800);
  GPS_SS.flush();
  
  //Initialisation script
  
  Serial.println("RTSHAB01 ");
  Serial.println("High alititude tracker for use in habbing, developed by Chris Atherton ");
  Serial.println("NTX2 code from UPU ");
  Serial.println("TinyGPS code from Mikal Hart ");
  Serial.println("Checksum code from Lunar_Lander ");
  Serial.println("http://www.RoutetoSpace.com ");
  Serial.println();
  
  //Send string via RTTY to say RTSHAB01 is active
  sprintf(datastring,"$$NSE POWER ON n");
  noInterrupts();  //Stop all other tasks
  rtty_txtstring (datastring);  
  interrupts();
  
}
 
void loop() {
 
  //Start watchkeeping.  If message count reaches upper limit of Int then it is reset. 
  msgcount = msgcount + 1;
    if(msgcount > 32000)
     {
       msgcount = 1;
     }
    
   //function declations 
 bool newGPSData = false; 
 unsigned long chars;
 unsigned short sentences, failed; 
    
    
//GPS SECTION START    
    
   //Begin parsing GPS data 
   for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS_SS.available())
    {
      char c = GPS_SS.read();
       //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (GPS.encode(c)) // Check to see if a new valid sentence come in?
        newGPSData = true;
    }
  }

if (newGPSData)
  {
     
    float flat, flon;
    unsigned long age;
    GPS.f_get_position(&flat, &flon, &age);
    
    dtostrf(flat,9,6,latstr); // convert lat from float to string
    dtostrf(flon,9,6,lonstr); // convert lon from float to string
    
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long fix_age;
    GPS.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
 
    //build up time stamp
    String tmstr;
    if(hour < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(hour, DEC);
   // tmstr = tmstr + ":";
    if(minute < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(minute, DEC);
   // tmstr = tmstr + ":";
    if(second < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(second, DEC);
   
    tmstr.toCharArray(timechara, 9) ;
   
   //correct no of 0's in long, (output: 00.575950/-0.575950)
    if (lonstr[0] == ' ') {
    lonstr[0] = '0';
    }
   
   alt = GPS.f_altitude(); // +/- altitude in meters
   velocity = GPS.f_speed_mps()*10;
   heading = GPS.f_course();
   sats = GPS.satellites();
    
  }
 
 /*Error checking
     If no data is received from the GPS then the error message is sent.
 */
 if(newGPSData == false)
   {
    Serial.println("No new data");
    sprintf(datastring,"$$NSE,NOGPS");
    noInterrupts();  //Stop all other tasks
    rtty_txtstring (datastring);  //Send "No new data" text string via RTTY
    interrupts();    //Start tasks again
   } 
 
 //Provide statistics info from GPS
 GPS.stats(&chars, &sentences, &failed);
  
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
 
//SGPS SECTION END

//RTTY SECTION START
  sprintf(datastring,"$$$$NSE,%i,%s,%s,%s,%i,%i,%i,%i,%i",msgcount,timechara,latstr,lonstr,alt,sats); // Puts the required data in the datastring
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
 
  Serial.println(datastring);//Output data to serial
  noInterrupts();
  rtty_txtstring (datastring);//Output data via RTTY
  interrupts();
  delay(2000);
  
//RTTY SECTION END
}
 
 
void rtty_txtstring (char * string)
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
