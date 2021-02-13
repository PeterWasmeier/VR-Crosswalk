#include <SPI.h>

const char SPI_CHIPSELECT = 10; // This signal is not needed, just put CS/SS at the slave to ground

void setup() {
  // INIT SPI:
  pinMode (SPI_CHIPSELECT, OUTPUT); // Use PIN #10 as "chip select" for SPI protocoll
  SPI.begin ();
  SPI.beginTransaction (SPISettings(14000000, MSBFIRST, SPI_MODE1));  
  digitalwrite (SPI_CHIPSELECT,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
