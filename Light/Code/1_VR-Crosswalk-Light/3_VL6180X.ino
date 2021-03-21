// VL6180X must be initialized like that:
// 1. Initialize the VL6180x (see Section 8) in the file "VL6180_ApplicationNote.pdf"
// 2. Write value 0x00 to register 0x016

// To start a single measurement, this must be done:
// 1. start single range measurement
//    Write value 0x01 to register 0x018.
// 2. poll the VL6180X till new sample ready
//    Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
// 3. Read range result (mm)
//    The result is in register 0x062.
// 4. Clear interrupts
//    Write value 0x07 to register 0x015


void I2C_VL6180X_Write (unsigned int uiRegister, byte Value)
{
  byte bStatus;
  byte *pointer=I2C_GetBufferpointer (Value);
  while (bStatus=nI2C->Write (I2C_HANDLE_VL6180X, uiRegister, pointer,1)!=0)
  { // Something went wrong
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit and try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }
}

void I2C_VL6180X_Init () 
{ 
  I2C_VL6180X_Write(0x0207, 0x01);
  I2C_VL6180X_Write(0x0208, 0x01);
  I2C_VL6180X_Write(0x0096, 0x00);
  I2C_VL6180X_Write(0x0097, 0xfd);
  I2C_VL6180X_Write(0x00e3, 0x00);
  I2C_VL6180X_Write(0x00e4, 0x04);
  I2C_VL6180X_Write(0x00e5, 0x02);
  I2C_VL6180X_Write(0x00e6, 0x01);
  I2C_VL6180X_Write(0x00e7, 0x03);
  I2C_VL6180X_Write(0x00f5, 0x02);
  I2C_VL6180X_Write(0x00d9, 0x05);
  I2C_VL6180X_Write(0x00db, 0xce);
  I2C_VL6180X_Write(0x00dc, 0x03);
  I2C_VL6180X_Write(0x00dd, 0xf8);
  I2C_VL6180X_Write(0x009f, 0x00);
  I2C_VL6180X_Write(0x00a3, 0x3c);
  I2C_VL6180X_Write(0x00b7, 0x00);
  I2C_VL6180X_Write(0x00bb, 0x3c);
  I2C_VL6180X_Write(0x00b2, 0x09);
  I2C_VL6180X_Write(0x00ca, 0x09);
  I2C_VL6180X_Write(0x0198, 0x01);
  I2C_VL6180X_Write(0x01b0, 0x17);
  I2C_VL6180X_Write(0x01ad, 0x00);
  I2C_VL6180X_Write(0x00ff, 0x05);
  I2C_VL6180X_Write(0x0100, 0x05);
  I2C_VL6180X_Write(0x0199, 0x05);
  I2C_VL6180X_Write(0x01a6, 0x1b);
  I2C_VL6180X_Write(0x01ac, 0x3e);
  I2C_VL6180X_Write(0x01a7, 0x1f);
  I2C_VL6180X_Write(0x0030, 0x00);

  // Recommended : Public registers - See data sheet for more detail
  I2C_VL6180X_Write(0x0011, 0x10);       // Enables polling for 'New Sample ready' when measurement completes
  I2C_VL6180X_Write(0x010a, 0x30);       // Set the averaging sample period (compromise between lower noise and increased execution time)
  I2C_VL6180X_Write(0x003f, 0x46);       // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
  I2C_VL6180X_Write(0x0031, 0xFF);       // sets the # of range measurements after which auto calibration of system is performed
  I2C_VL6180X_Write(0x0040, 0x63);       // XXX Set ALS integration time to 100ms
  I2C_VL6180X_Write(0x002e, 0x01);       // perform a single temperature calibration of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  I2C_VL6180X_Write(0x001b, 0x09);       // XXX Set default ranging inter-measurement period to 100ms
  I2C_VL6180X_Write(0x003e, 0x31);       // Set default ALS inter-measurement period to 500ms
  I2C_VL6180X_Write(0x0014, 0x24);       // Configures interrupt on 'New Sample Ready threshold event'

  // Special settings:
  //  6=25ms
  //  7=55ms aber auch 7=14ms
  // 10=52ms 
  // 20=59ms
  I2C_VL6180X_Write(0x001C, 6);       // Maximum time to run measurement in Ranging modes (1unit=1ms)
  
  I2C_VL6180X_Write(0x0016, 0x00);       // VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET
}


void I2C_VL6180X_StartMeasurement (tVL6180X_Values *pVL6180X)
{
  //return;  
  byte bStatus;
  byte *pointer;
  pointer = I2C_GetBufferpointer (1);
  // start single range measurement:
  while (bStatus = nI2C->Write (I2C_HANDLE_VL6180X, 0x018, pointer, 1) != 0) 
  { // There is an issue with nI2C, send error message to host and stop the arduino
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit an try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
      while (1) { }; // STOP Arduino
    }
  }
}

void I2C_VL6180X_ReadValues (tVL6180X_Values *pVL6180X)
{
  byte bStatus;
  I2C_VL6180X_Value_Pointer = pVL6180X; 
  pVL6180X->Valid = 0;
  // poll the VL6180X till new sample ready
  if (bStatus = nI2C->Read (I2C_HANDLE_VL6180X, 0x04f, &I2C_VL6180X_ReceiveBuffer[0], 1, I2C_VL6180X_OnPollForNewSampleReady) != 0)
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  }
}

void I2C_VL6180X_OnPollForNewSampleReady (const uint8_t bStatus)
{
  byte result;
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
  result = I2C_VL6180X_ReceiveBuffer[0] & 0x04;
  if (result>0)
  { 
    // Read range result (mm)
    if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x062, &I2C_VL6180X_ReceiveBuffer[0], 1, I2C_VL6180X_OnDistance) != 0)
    { // There is an issue with this library:
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
      while (1) { }; // STOP Arduino
    }
  }
  else
  {
    // Poll again
    // poll the VL6180X till new sample ready
    if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x04f, &I2C_VL6180X_ReceiveBuffer[0], 1, I2C_VL6180X_OnPollForNewSampleReady) != 0)
    { // There is an issue with this library:
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
      while (1) { }; // STOP Arduino
    }
  }
}

void I2C_VL6180X_OnDistance (const uint8_t bStatus)
{
  byte distance;
  byte bStatus2;
  byte *pointer;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  distance=I2C_VL6180X_ReceiveBuffer[0];
  I2C_VL6180X_Value_Pointer->Distance = distance;
  // Read status information (error code out of this sensor):
  if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x04d, &I2C_VL6180X_ReceiveBuffer[0], 1, I2C_VL6180X_OnErrorCode) != 0)
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  } 
}

void I2C_VL6180X_OnErrorCode (const uint8_t bStatus)
{
  byte errorcode;
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Read register 0x04d
  errorcode = I2C_VL6180X_ReceiveBuffer[0];
  I2C_VL6180X_Value_Pointer->ErrorCode = errorcode;
  // Read signal rate:
  if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x066, &I2C_VL6180X_ReceiveBuffer[0], 2, I2C_VL6180X_OnSignalRate) != 0)
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  } 
}

void I2C_VL6180X_OnSignalRate (const uint8_t bStatus)
{
  unsigned long signalrate;
  byte bStatus2;
  byte *pointer;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }

  // Now, clear interrupts
  // Write value 0x07 to register 0x015
  pointer = I2C_GetBufferpointer (7);
  while (bStatus2=nI2C->Write (I2C_HANDLE_VL6180X, 0x015, pointer,1)!=0)
  { // Something went wrong
    if (bStatus2==1) // I2C library is busy
    {
      // Wait a bit and try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }
  // interprete register 0x066
  signalrate = I2C_VL6180X_ReceiveBuffer[1];
  signalrate = signalrate << 8;
  signalrate = signalrate + I2C_VL6180X_ReceiveBuffer[0];
  signalrate = signalrate >> 1;
  // Rate has changed from high to low:
  I2C_VL6180X_Value_Pointer->SignalRate = signalrate;
  I2C_VL6180X_Value_Pointer->Valid = true;
}
