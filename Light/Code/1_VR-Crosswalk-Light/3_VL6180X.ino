// VL6180X must be initialized like that:
// 1. Check device has powered up (Optional)
//    For that, read register 0x016 and wait until its value is 0x01
// 2. Initialize the VL6180x (see Section 8) in the file "VL6180_ApplicationNote.pdf"
// 3. Write value 0x00 to register 0x016

// To start a single measurement, this must be done:
// 1. start single range measurement
//    Write value 0x01 to register 0x018.
// 2. poll the VL6180X till new sample ready
//    Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
// 3. Read range result (mm)
//    The result is in register 0x062.
// 4. Clear interrupts
//    Write value 0x07 to register 0x015


// -----------------------------------------------------------------
// I2C_VL6180X_Init
// -----------------------------------------------------------------
// This function will start the init procedure.
// Next step of the init is continues in the function 
// "I2C_VL6180X_OnPoweredUp"
// -----------------------------------------------------------------
void I2C_VL6180X_Init ()
{
  byte bStatus;
  VL6180X_IsInitialized=false;
  // Check device has powered up (Optional)
  // For that, read register 0x016 and wait until its value is 0x01  
  if (bStatus = nI2C->Read (I2C_HANDLE_VL6180X, 0x016, &I2C_VL6180X_ReceiveBuffer.data[0], 1, I2C_VL6180X_OnPoweredUp) != 0) 
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Inform the host computer that there is an error
    while (1) { }; // STOP Arduino
  }
  // Wait until init is finished:
  while (VL6180X_IsInitialized==false)
  {
    delay (1);
  }
}

void I2C_VL6180X_Write (unsigned int uiRegister, byte *Value)
{
  byte bStatus;
  if (bStatus=nI2C->Write (I2C_HANDLE_VL6180X, uiRegister, Value,1)!=0)
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer that there is an issue
    while (1) { }; // STOP ardurino
  }
}


void I2C_VL6180X_SendInitValues () 
{ 
  I2C_VL6180X_Write(0x0207, &cValue[0x01]);
  I2C_VL6180X_Write(0x0208, &cValue[0x01]);
  I2C_VL6180X_Write(0x0096, &cValue[0x00]);
  I2C_VL6180X_Write(0x0097, &cValue[0xfd]);
  I2C_VL6180X_Write(0x00e3, &cValue[0x00]);
  I2C_VL6180X_Write(0x00e4, &cValue[0x04]);
  I2C_VL6180X_Write(0x00e5, &cValue[0x02]);
  I2C_VL6180X_Write(0x00e6, &cValue[0x01]);
  I2C_VL6180X_Write(0x00e7, &cValue[0x03]);
  I2C_VL6180X_Write(0x00f5, &cValue[0x02]);
  I2C_VL6180X_Write(0x00d9, &cValue[0x05]);
  I2C_VL6180X_Write(0x00db, &cValue[0xce]);
  I2C_VL6180X_Write(0x00dc, &cValue[0x03]);
  I2C_VL6180X_Write(0x00dd, &cValue[0xf8]);
  I2C_VL6180X_Write(0x009f, &cValue[0x00]);
  I2C_VL6180X_Write(0x00a3, &cValue[0x3c]);
  I2C_VL6180X_Write(0x00b7, &cValue[0x00]);
  I2C_VL6180X_Write(0x00bb, &cValue[0x3c]);
  I2C_VL6180X_Write(0x00b2, &cValue[0x09]);
  I2C_VL6180X_Write(0x00ca, &cValue[0x09]);
  I2C_VL6180X_Write(0x0198, &cValue[0x01]);
  I2C_VL6180X_Write(0x01b0, &cValue[0x17]);
  I2C_VL6180X_Write(0x01ad, &cValue[0x00]);
  I2C_VL6180X_Write(0x00ff, &cValue[0x05]);
  I2C_VL6180X_Write(0x0100, &cValue[0x05]);
  I2C_VL6180X_Write(0x0199, &cValue[0x05]);
  I2C_VL6180X_Write(0x01a6, &cValue[0x1b]);
  I2C_VL6180X_Write(0x01ac, &cValue[0x3e]);
  I2C_VL6180X_Write(0x01a7, &cValue[0x1f]);
  I2C_VL6180X_Write(0x0030, &cValue[0x00]);

  // Recommended : Public registers - See data sheet for more detail
  I2C_VL6180X_Write(0x0011, &cValue[0x10]);       // Enables polling for 'New Sample ready' when measurement completes
  I2C_VL6180X_Write(0x010a, &cValue[0x30]);       // Set the averaging sample period (compromise between lower noise and increased execution time)
  I2C_VL6180X_Write(0x003f, &cValue[0x46]);       // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
  I2C_VL6180X_Write(0x0031, &cValue[0xFF]);       // sets the # of range measurements after which auto calibration of system is performed
  I2C_VL6180X_Write(0x0040, &cValue[0x63]);       // XXX Set ALS integration time to 100ms
  I2C_VL6180X_Write(0x002e, &cValue[0x01]);       // perform a single temperature calibration of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  I2C_VL6180X_Write(0x001b, &cValue[0x09]);       // XXX Set default ranging inter-measurement period to 100ms
  I2C_VL6180X_Write(0x003e, &cValue[0x31]);       // Set default ALS inter-measurement period to 500ms
  I2C_VL6180X_Write(0x0014, &cValue[0x24]);       // Configures interrupt on 'New Sample Ready threshold event'

  I2C_VL6180X_Write(0x0016, &cValue[0x00]);       // VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET
}

// -----------------------------------------------------------------
// I2C_VL6180X_OnPoweredUp
// -----------------------------------------------------------------
// Callback function of "Check device has powered up (Optional)"
// For that, check if the received value is 0x01.
// -----------------------------------------------------------------
void I2C_VL6180X_OnPoweredUp (const uint8_t bStatus)
{
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  if (I2C_VL6180X_ReceiveBuffer.data[0]!=0x01)
  { // ask again, until it is powered up
    if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x016, &I2C_VL6180X_ReceiveBuffer.data[0], 1, I2C_VL6180X_OnPoweredUp) != 0) 
    { // There is an issue with this library:
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Inform the host computer that there is an error
      while (1) { }; // STOP Arduino
    }
  }
  else
  { // sensor is powered up. Continue with next step.
    // Write setting to this sensor:
    I2C_VL6180X_SendInitValues ();
    VL6180X_IsInitialized=true;
  }
}

// -----------------------------------------------------------------
// I2C_VL6180X_ReadValues
// -----------------------------------------------------------------
// This function will initiate the reading procedure.
// It will switch to "I2C_GY271_OnPollForNewSampleReady".
// -----------------------------------------------------------------
void I2C_VL6180X_ReadValues (tVL6180X_Values *pVL6180X)
{
  byte bStatus;
  I2C_VL6180X_Value_Pointer = pVL6180X; 
  pVL6180X->Valid = 0;
  // start single range measurement:
  if (bStatus = nI2C->Write (I2C_HANDLE_VL6180X, 0x018, &cValue[0x01], 1) != 0) 
  { // There is an issue with nI2C, send error message to host and stop the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  }
  // poll the VL6180X till new sample ready
  if (bStatus = nI2C->Read (I2C_HANDLE_VL6180X, 0x04f, &I2C_GY271_ReceiveBuffer.data[0], 1, I2C_GY271_OnPollForNewSampleReady) != 0)
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  }
}

// -----------------------------------------------------------------
// I2C_GY271_OnPollForNewSampleReady
// -----------------------------------------------------------------
// This is the ISR function of "I2C_VL6180X_ReadValues"
// It will check if the laser result is available. If so, it will
// ask for the measured value. Otherwise it will poll again.
// -----------------------------------------------------------------
void I2C_GY271_OnPollForNewSampleReady (const uint8_t bStatus)
{
  byte result;
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
  result = I2C_VL6180X_ReceiveBuffer.data[0] & 0x07;
  if (result==0x04)
  {
    // Read range result (mm)
    if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x062, &I2C_GY271_ReceiveBuffer.data[0], 1, I2C_GY271_OnDistance) != 0)
    { // There is an issue with this library:
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
      while (1) { }; // STOP Arduino
    }
  }
  else
  {
    // Poll again
    // poll the VL6180X till new sample ready
    if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x04f, &I2C_GY271_ReceiveBuffer.data[0], (uint32_t)1, I2C_GY271_OnPollForNewSampleReady) != 0)
    { // There is an issue with this library:
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
      while (1) { }; // STOP Arduino
    }
  }
}

// -----------------------------------------------------------------
// I2C_GY271_OnDistance
// -----------------------------------------------------------------
// This is the callback / ISR of the function 
// "I2C_GY271_OnPollForNewSampleReady". It will receive the measure
// value of the sensor.
// -----------------------------------------------------------------
void I2C_GY271_OnDistance (const uint8_t bStatus)
{
  byte distance;
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
  distance=I2C_VL6180X_ReceiveBuffer.data[0];
  I2C_VL6180X_Value_Pointer->Distance = distance;
  // XXX I2C_VL6180X_Value_Pointer->Valid = true;
  // Now, clear interrupts
  // Write value 0x07 to register 0x015
  if (bStatus2=nI2C->Write (I2C_HANDLE_VL6180X, 0x015, &cValue[0x07],1)!=0)
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer that there is an issue
    while (1) { }; // STOP ardurino
  }
  // Read status information (error code out of this sensor):
  if (bStatus2 = nI2C->Read (I2C_HANDLE_VL6180X, 0x04d, &I2C_GY271_ReceiveBuffer.data[0], 2, I2C_GY271_OnErrorCode) != 0)
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus2, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address));
    while (1) { }; // STOP Arduino
  }  
}

void I2C_GY271_OnErrorCode (const uint8_t bStatus)
{
  byte errorcode;
  byte bStatus2;
  if (bStatus != 0) // is there an issue with the received data from the current VL6180X sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_VL6180X.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Read register 0x04f (do a and-operation with value 0x07) until its result is 0x04
  errorcode=I2C_VL6180X_ReceiveBuffer.data[0];
  errorcode=errorcode >> 4;
  // XXX
}  
