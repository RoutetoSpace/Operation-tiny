/*  NTX2 Radio with Max 6 GPS and AD22100S and DS1620 temp sensors.  Adapted from NTX2 Radio Test Part 2 by Rob Harrison and M0UPU http://ukhas.org.uk/guides:linkingarduinotontx2
    GPS code adapted from 2E0UPU and James Coxon http://ukhas.org.uk/guides:ublox6
    Transmits temp via RTTY with a checksum.
 
    By C Atherton 2012-2013 - www.routetospace.com
    
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

#include <ds1620.h>
#include <string.h>
#include <util/crc16.h>
#include <SoftwareSerial.h>
SoftwareSerial GPS(4, 5);  //RX pin 4, TX pin5

#define RADIOPIN 13 //radio output pin
#define AD22100S 0 //define temp sensor pin

// AD22100S and Voltage definitions
int val;
double AD22100Sout;
unsigned int ADCValue;
double Voltage;
double Vcc;
double Vout;
double conts = 4.882;

//GPS success byte
byte gps_set_sucess = 0 ;

//Data sting definition
char datastring[80];
char GPSdatastr[80];

// Define pins for 3-wire serial communication
int dq = 11;
int clk = 10;
int rst = 9;

// Call DS1620 constructor using pin variables
DS1620 d = DS1620(dq, clk, rst);



//Voltage meter reading gained by comparing the external voltage with the internal 1.1V
long readVcc() {
  long result;
  //Read the 1.1V reference against AVcc
  //ADMUX analog voltage reference byte
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring 
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; //Back calculate AVcc in mV
  return result; // Vcc in millivolts
}
void setup()
{
  Serial.begin(9600);//setup Serial Library at 9600 bps
  // Write TH and TL setpoints to DS1620 EEPROM  
  // Settings are retained even with no power
  d.write_th(30);
  d.write_tl(15);
  
  // Write config to DS1620 configuration/status register
  // Decimal 10 = Binary 00001010
  // Enables CPU-Mode and disables 1-Shot Mode.
  // See Datasheet for details
  d.write_config(10);
  
  // Start continuous temperature conversion
  // Readings can be read about once per second this way
  d.start_conv();
  
  //Set pin mode for NTX2 pin to output 
  pinMode(RADIOPIN,OUTPUT);
  
  //Start GPS comms
  GPS.begin(9600);
  
  //Send initialisation text via radio to confirm startup
  sprintf(GPSdatastr,"Route to Space RTS01");
  rtty_txstring (GPSdatastr);
  sprintf(datastring,"31/12/2012 CATHERTON");
  rtty_txstring (GPSdatastr);
  sprintf(datastring,"Initialising....");
  rtty_txstring (GPSdatastr);
  
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  
  GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  GPS.begin(4800);
  GPS.flush();
  
  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  sprintf(GPSdatastr,"Setting uBlox nav mode: ");
  rtty_txstring (GPSdatastr);
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                   };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
}
//long result = readVcc();
void loop()
{
  Vcc = readVcc()/1000.0;
  ADCValue = analogRead(AD22100S); //Reads data from the AD22100S pin
  Vout = ADCValue * conts;
  Voltage = (ADCValue / 1023.0) * Vcc;  
  AD22100Sout = (Vout/22.5)-63.11111111; //previous value was 61.11111 in line with formula from tech report.  Calibration against house themostat required 3 degrees less. Changed Vout to Voltage to take in to account voltage fultuations
  Serial.print(Vcc); Serial.print("\t"); Serial.println("Vcc");
  Serial.print(AD22100Sout); Serial.print("\t"); Serial.println("Deg C AD2210");
  Serial.print(d.read_temp()); Serial.print("\t"); Serial.println("Deg C DS1620");
  
  Serial.println(" ");
  delay(1500);
}
