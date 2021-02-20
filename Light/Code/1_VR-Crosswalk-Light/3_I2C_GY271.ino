// -----------------------------------------------------------------
// I2C_GY271_Write
// -----------------------------------------------------------------
// Will write a value to a given register of the GY-271/HMC5883L
// magnetic field sensor.
// -----------------------------------------------------------------
void I2C_GY271_Write (byte Register, byte Value)
{
  byte bStatus;
  if (bStatus = nI2C->Write (I2C_HANDLE_GY271, Register, &Value, (uint32_t)1) != 0)
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_GY271.device_address)); // Tell the host computer that there is an issue
    while (1) { }; // STOP ardurino
  }
}

// -----------------------------------------------------------------
// I2C_GY271_Init
// -----------------------------------------------------------------
// Use this function to initialize the GY-271/HMC5883L magnetic
// field sensor.
// -----------------------------------------------------------------
void I2C_GY271_Init ()
{
  I2C_GY271_Write(0x0A, 0x80);  // Soft Reset
  I2C_GY271_Write(0x0B, 0x01);  // Define Set/Reset period
  //I2C_GY271_Write(0x09, 0x1D);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
  I2C_GY271_Write(0x09, 0x15);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 50Hz, set continuous measurement mode
  //I2C_GY271_Write(0x09, 0x05);  // Define OSR = 512, Full Scale Range = 2 Gauss, ODR = 50Hz, set continuous measurement mode
}

// -----------------------------------------------------------------
// I2C_GY271_Read_Values
// -----------------------------------------------------------------
// This function is asking one GY271 sensor for its values.
// The parameter "pGY271" contains a pointer to one of the
// bufferplaces for each GY271. Doesnt matter if it is footplate
// left or right.
// Before this function is called, it is important to use
// the function "I2C_Select_GY271" before.
// -----------------------------------------------------------------
void I2C_GY271_Read_Values (tGY271_Values *pGY271)
{
  byte bStatus;
  I2C_GY271_Value_Pointer = pGY271; // The ISR function ("I2C_GY271_OnDataReceived") needs to know where to put the received values. So copy the pointer to this global variable.
  pGY271->Valid = 0;                // Set the valid variable in this bufferplace to false, so that ardurino wont use these values for any calculation.
  if (bStatus = nI2C->Write (I2C_HANDLE_GY271, &cValue[0], 1) != 0) // Tell this GY271 sensor we want to read beginning of register zero (there are the measured values stored)
  { // There is an issue with nI2C, send error message to host and stop the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_GY271.device_address)); // Inform the host computer that there is an error with I2C
    while (1) { }; // STOP Arduino
  }
  // Tell the "nI2C" library that the next data, which is received over the I2C bus, comes from this GY271 sensor:
  if (bStatus = nI2C->Read (I2C_HANDLE_GY271, &I2C_GY271_ReceiveBuffer.data[0], 6, I2C_GY271_OnDataReceived) != 0) // Tell the library where to store the received values and which ISR to call when data is received
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_GY271.device_address)); // Inform the host computer that there is an error
    while (1) { }; // STOP Arduino
  }
}

// -----------------------------------------------------------------
// I2C_GY271_OnDataReceived
// -----------------------------------------------------------------
// This is the callback function of the I2C library. Will be called
// in case data is received by the GY271 sensor.
// -----------------------------------------------------------------
void I2C_GY271_OnDataReceived (const uint8_t bStatus)
{
  int x, y, z;
  byte s;
  if (bStatus != 0) // is there an issue with the received data from the current GY271 sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_GetErrorSource(I2C_HANDLE_GY271.device_address)); // Tell the host computer, there is something wrong
    while (1) { }; // Stop the arduino
  }
  // Looks like to be fine:
  // Copy the received value from the global buffer to local variables (I know this is stupid):
  x = (int)(int16_t)(I2C_GY271_ReceiveBuffer.data[0] | I2C_GY271_ReceiveBuffer.data[1] << 8);
  y = (int)(int16_t)(I2C_GY271_ReceiveBuffer.data[2] | I2C_GY271_ReceiveBuffer.data[3] << 8);
  z = (int)(int16_t)(I2C_GY271_ReceiveBuffer.data[4] | I2C_GY271_ReceiveBuffer.data[5] << 8);
  s = I2C_GY271_ReceiveBuffer.data[6];
  // Now, copy these values from the local variables to the buffer of the current GY271 sensor:
  I2C_GY271_Value_Pointer->X = x;
  I2C_GY271_Value_Pointer->Y = y;
  I2C_GY271_Value_Pointer->Z = z;
  /* XXX The programmer has to think about this one:
      a) it is important?
      b) do we really need this one?
      c) what should we do in such a case?
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
  I2C_GY271_Value_Pointer->Valid = 1; // Tell the arduino, that we now can calculate with these values, because they are valid
}
