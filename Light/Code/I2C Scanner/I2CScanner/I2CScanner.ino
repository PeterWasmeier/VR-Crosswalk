#include "Wire.h"
extern "C" {
#include "utility/twi.h" // from Wire library, so we can do bus scanning
}

void TCADisable (byte TCA)
{
  Wire.beginTransmission(TCA);
  Wire.write(0);
  Wire.endTransmission();  
}

void TCASelect (byte TCA, byte channel) 
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void ScanTCA (byte TCA)
{
  byte address;
  byte error;
  TCADisable(0x70);
  TCADisable(0x71);
  TCADisable(0x74);
  TCADisable(0x75);
  for (byte channel=0; channel<8; channel++) 
  {
    TCASelect(TCA, channel);
    for(address = 1; address < 127; address++ )
    {
      if ((address!=0x70)&&
          (address!=0x71)&&
          (address!=0x74)&&
          (address!=0x75))
      {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
          Serial.print("TCA 0x"); 
          Serial.print(TCA,HEX);
          Serial.print(", channel ");
          Serial.print(channel);
          Serial.print (": device found with address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }
        else if (error==4)
        {
          Serial.print("TCA 0x"); 
          Serial.print(TCA,HEX);
          Serial.print(", channel ");
          Serial.print(channel);
          Serial.print (": unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }
      }    
    }
  }  
}

void setup()
{
  while (!Serial);
  delay(1000);
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Scanning left footplate:");
  ScanTCA (0x70);
  ScanTCA (0x71);
  Serial.println("Scanning right footplate:");
  ScanTCA (0x74);
  ScanTCA (0x75);
  Serial.println("done");
}

void loop()
{
  //
}
