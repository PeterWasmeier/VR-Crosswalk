void fGY271_Read_Value (tGY271 *pGY271) {
  byte bStatus;
  pGY271_Callback_Sensor = pGY271;
  pGY271->Valid=0;
  if (bStatus=nI2C->Write (I2C_GY271, &cZero[0], 1)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Read_Value");
  if (bStatus=nI2C->Read (I2C_GY271, &cGY271_ReceiveBuffer[0], (uint32_t)6, fGY271_Callback)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Read_Value");
}

void fGY271_Read_Values () {
  static byte bIndex=0;
  // RIGHT FOOTPRINT:
  // ~~~~~~~~~~~~~~~~
  // Keep in mind: because the footplate is upside down, X and Y must be inverted
  switch (bIndex)
  {
    case 0:   FootprintRight.SensorOffsetValid=false;                                           // The current data is not yet valid
              fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 0);  // select right footprint, upper, GY271 channel #0
              fGY271_Read_Value (&FootprintRight.GY271[0]);                           // Request values
              bIndex++;
              break;
    case 1:   if (FootprintRight.GY271[0].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 3);// select right footprint, upper, GY271 channel #3
                fGY271_Read_Value (&FootprintRight.GY271[1]);                         // Request values
                bIndex++;
              }
              break;
    case 2:   if (FootprintRight.GY271[1].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 4);// select right footprint, upper, GY271 channel #4
                fGY271_Read_Value (&FootprintRight.GY271[2]);                         // Request values
                bIndex++;
              }
              break;
    case 3:   if (FootprintRight.GY271[2].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 0);// select right footprint, lower, GY271 channel #0
                fGY271_Read_Value (&FootprintRight.GY271[3]);                         // Request values
                bIndex++;
              }
              break;
    case 4:   if (FootprintRight.GY271[3].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 3);// select right footprint, lower, GY271 channel #3
                fGY271_Read_Value (&FootprintRight.GY271[4]);                         // Request values
                bIndex++;
              }
              break;
    case 5:   if (FootprintRight.GY271[4].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 4);// select right footprint, lower, GY271 channel #4
                fGY271_Read_Value (&FootprintRight.GY271[5]);                         // Request values
                bIndex++;
              }
              break;
    case 6:   if (FootprintRight.GY271[5].Valid)
              {
                fCalculate_GY271_CenterOfFootplate (&FootprintRight);
                FootprintRight.SensorOffset.X = -round(FootprintRight.SensorOffset.X); // For unknown reason, this value is inverted
                FootprintRight.SensorOffset.Y = -round(FootprintRight.SensorOffset.Y);
                bIndex=0;
              }
  }
}

void fGY271_Callback (const uint8_t bStatus) {
  int x, y, z;
  byte s;
  if (bStatus!=0)
  {
    fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Callback");
  }  
  x = (int)(int16_t)(cGY271_ReceiveBuffer[0] | cGY271_ReceiveBuffer[1] << 8);
  y = (int)(int16_t)(cGY271_ReceiveBuffer[2] | cGY271_ReceiveBuffer[3] << 8);
  z = (int)(int16_t)(cGY271_ReceiveBuffer[4] | cGY271_ReceiveBuffer[5] << 8);
  s = cGY271_ReceiveBuffer[6];
  pGY271_Callback_Sensor->Value.X = x;
  pGY271_Callback_Sensor->Value.Y = y;
  pGY271_Callback_Sensor->Value.Z = z;
/*
  if (((s & 0x02)==0)&&((x!=0)||(y!=0))) { // no overflow?
    // If compass module lies flat on the ground with no tilt,
    // just x and y are needed for calculation
    pGY271_Callback_Sensor->Valid=1;
  }
  else
  {
    pGY271_Callback_Sensor->Valid=1;
    //fSTOP ("Value overflow at GY271","fGY271_Callback");
  }
  */
  pGY271_Callback_Sensor->Valid=1;
}

void fGY271_Write (byte bRegister, byte bValue) {
  byte bStatus;
  if (bStatus=nI2C->Write (I2C_GY271, bRegister, &bValue, (uint32_t)1)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Write");
}

void fGY271_Init (CI2C::Handle bModule, byte bPort) { 
  fTCA9548A_Select_I2C_BusDevice (bModule, bPort);
  fGY271_Write(0x0A, 0x80);  // Soft Reset
  fGY271_Write(0x0B, 0x01);  // Define Set/Reset period
  //fGY271_Write(0x09, 0x1D);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
  fGY271_Write(0x09, 0x15);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 50Hz, set continuous measurement mode
  //fGY271_Write(0x09, 0x05);  // Define OSR = 512, Full Scale Range = 2 Gauss, ODR = 50Hz, set continuous measurement mode
}
