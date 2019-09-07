void fSTOP_I2C (byte bI2CAddress, byte bI2CResult, char *pFunction) {
  Serial.println ("SYSTEM STOPPED BECAUSE OF I2C ERROR.");
  Serial.print   ("Current TCA9548A: 0x");  Serial.println (bSTOP_CurrentTCA9548A,HEX);
  Serial.print   ("TCA9548A Port   : 0x");  Serial.println (bSTOP_CurrentTCA9548A_Port);
  Serial.print   ("I2C destination : 0x");  Serial.println   (bI2CAddress,HEX);
  Serial.print   ("Message         : ");
  switch (bI2CResult)
  {
    case 0: Serial.println ("success"); break;
    case 1: Serial.println ("busy"); break;
    case 2: Serial.println ("timeout"); break;
    case 3: Serial.println ("data too long to fit in transmit buffer"); break;
    case 4: Serial.println ("memory allocation failure"); break;
    case 5: Serial.println ("attempted illegal transition of state"); break;
    case 6: Serial.println ("received NACK on transmit of address"); break;
    case 7: Serial.println ("received NACK on transmit of data"); break;
    case 8: Serial.println ("illegal start or stop condition on bus"); break;
    case 9: Serial.println ("lost bus arbitration to other master"); break;
    case 100: Serial.println ("Value overflow error at GY271"); break;
  }
  Serial.print   ("Functionname    : ");
  Serial.println (pFunction);
  while (1==1) 
  {
    delay(1);
    Serial.flush();
  };
}
