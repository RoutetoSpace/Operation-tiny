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
    
    declaration of sats incorrect.  wrong value - moved declarations to after the point the GPS is called
    test payload with gps data to see if there are any issues - see ukhas payload testing page for details
    need to update GPS configuration setting to match the values I require on startup
    Need to correct the upload issue.  Arduino fails to upload when GPS is active.  When disabled (rx or tx removed) upload completes successfully.  Issue is the same when uploading GPS test script.
       
    */ 

//Includes section
 
#include <TinyGPS.h>
#include <string.h>
#include <util/crc16.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdint.h>

//Definition section
#define RADIOPIN 13 
byte GPS_TX=3; //GPS RX (DB0) -> Digital pin 3
byte GPS_RX=4; //GPS TX (DB1) -> Digital pin 4

//GPS declarations sestion
TinyGPS GPS;
SoftwareSerial GPS_SS(GPS_TX, GPS_RX);  //GPS RX (DB0) -> Digital pin 3, GPS TX (DB1) -> Digital pin 4 
byte gps_set_sucess = 0 ;  //success byte for GPS Software Serial
char timechara[9];
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
  
  //Select pinmode for testing purposes only
  pinMode(GPS_TX, INPUT);
  pinMode(GPS_RX, OUTPUT);
  
  // THIS COMMAND SETS UP THE GPS FIGHT MODE AND CONFIRMS IT - This section was missing from my previous code
  
  
 /* GPS_SS.println("Setting uBlox nav mode: ");
 // uint8_t setNav[] = {
 //   0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                      };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
 */ 
  
  //Initialisation script sent via serial
  
  Serial.println("RTSHAB01 ");
  Serial.println("High alititude tracker for use in habbing, developed by Chris Atherton ");
  Serial.println("NTX2 code from UPU ");
  Serial.println("TinyGPS code from Mikal Hart ");
  Serial.println("Checksum code from Lunar_Lander ");
  Serial.println("http://www.RoutetoSpace.com ");
  Serial.println();
  
  //Send string via RTTY to say RTSHAB01 is active
  sprintf(datastring,"$$RTSHAB-TEST POWER ON /n");
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
      char c = GPS_SS.read();  //Should be char c = Serial.read(); but changed for testing purposes
    //  GPS_SS.write(c); // uncomment this line if you want to see the GPS data flowing
      Serial.print(c); //prints GPS data to serial.  Used for testing purposes only.
      if (GPS.encode(c)) // Check to see if a new valid sentence come in?
        newGPSData = true;
    }
  }

 int sats = GPS.satellites(); //number of satellites that are in view
 int alt = GPS.f_altitude(); // +/- altitude in meters
 int velocity = GPS.f_speed_mps()*10;
 int heading = GPS.f_course();

if (newGPSData)
  {
    
    sats = 0;
    
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
   
   //RTTY SECTION START

     //constructs the data string
    sprintf(datastring, "$$RTSHAB-TEST,%i,%s,%s,%s,%i,%i,",msgcount,timechara,latstr,lonstr,alt,sats); // Puts the required data in the datastring  -  removed 3 unnecessary %i's --1/4/13 - added an extra , to string becuase of error with checksum
            //sizeof(datastring) removed due to compiler error - invalid change from int to const*Char
     //constructs the checksum  
    char checksum_str[10]; //checksum size changed from 6 to 10 to match ukhas wiki
    sprintf(checksum_str, "*%04X\n", gps_CRC16_checksum(datastring));//removed * from "*%04X\n" null setting.  %04X/n means (%)placeholder, use 0 instead of spaces (0), length of 4 (4), type unsigned int as a hexidecimal number uppercase(X), with (/), type (n) which means print nothing. see http://en.wikipedia.org/wiki/Printf_format_string#Format_placeholders
            //sizeof(checksum_str) removed due to compiler error - invalid change from int to const*Char
     //error checking 
      if (strlen(datastring) > sizeof(datastring) - 4 - 1)
            {
              //don't over flow the buffer.  You should make it bigger
              return;
            }
     //Copy the checksum terminating \o (hence the +1
     //  removed this line becuase two checksums were sent memcpy(datastring + strlen(datastring), checksum_str, strlen(checksum_str) + 1);
     //Concatinate the datastring and the checksum together  
    strcat(datastring,checksum_str);
     //Send the data string with the checksum 
  Serial.println(datastring);//Output data to serial
  noInterrupts();
  rtty_txtstring (datastring);//Output data via RTTY
  interrupts();
  delay(2000); 
 }
//RTTY SECTION END


//RTTY NO GPS SECTION BEGINS
 /*Error checking
     If no data is received from the GPS then the error message is sent.
 */
 if(newGPSData == false)
   {
    sats = 0;
    Serial.println("No new data");
    sprintf(datastring,"$$RTSHAB-TEST,%i,NOGPS,%i,",msgcount,sats);
    char checksum_str[10];
    sprintf(checksum_str, "*%04X\n", gps_CRC16_checksum(datastring));
    if (strlen(datastring) > sizeof(datastring) - 4 - 1)
            {
  //don't over flow the buffer.  You should make it bigger
              return;
            }
//Concatinate the datastring and the checksum together  
    strcat(datastring,checksum_str);
//Send NO GPS Data with a checksum    
    Serial.println(datastring);//Output data to serial
    noInterrupts();  //Stop all other tasks
    rtty_txtstring (datastring);  //Send "No new data" text string via RTTY
    interrupts();    //Start tasks again
   } 
 
//RTTY NO GPS SECTION ENDS


 
 //Provide statistics info from GPS
 GPS.stats(&chars, &sentences, &failed);
  
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
 
//Serial GPS SECTION END
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
 
 
 //Checksum code
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

//GPS Software Serial data

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    GPS_SS.print(MSG[i], HEX);
  }
  Serial.println();
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  GPS_SS.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      GPS_SS.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      GPS_SS.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        GPS_SS.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}
