void I2C_Select_TCA9548A (bool LEFT, bool UPPER, byte ChannelsToActivate)
{
  byte bStatus;
  byte *pointer;
  if (ChannelsToActivate > 63) // There are only 0...7 channels for each TCA9548A I2C multiplexer. But only six of them (0..5) are being used. Keep in mind this is a bitmask. So value 3 will enable channel 0 and channel 1.
  { // Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_CODE, 2, 0); // ERROR: Invalid parameter 'ChannelsToActivate' in function 'I2C_Select_TCA9548A'
    while (1) { }; // STOP ardurino
  }
  I2C_CHANNEL_Current_TCA9548A = ChannelsToActivate;
  // First, disable all of them:
  pointer = I2C_GetBufferpointer (0);
  while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, pointer, 1) != 0)
  { // Something went wrong
    if (bStatus!=1)
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Upper.device_address); 
      while (1) { }; // STOP ardurino
    }
  }  
  while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, pointer, 1) != 0)
  { // Something went wrong
    if (bStatus!=1)
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Lower.device_address); 
      while (1) { }; // STOP ardurino
    }
  }  
  while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, pointer, 1) != 0)
  { // Something went wrong
    if (bStatus!=1)
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Upper.device_address); 
      while (1) { }; // STOP ardurino
    }
  }
  while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, pointer, 1) != 0)
  { // Something went wrong
    if (bStatus!=1)
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Lower.device_address); 
      while (1) { }; // STOP ardurino
    }
  }  
  
  // Activate the desired channels:
  pointer = I2C_GetBufferpointer (ChannelsToActivate);
  if (LEFT==true)
  {
    if (UPPER==true)
    {
      I2C_HANDLE_Current_TCA9548A_device_address = Footplate_Left.I2C_HANDLE_TCA9548A_Upper.device_address;
      while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, pointer, 1) != 0)
      { // Something went wrong
        if (bStatus!=1)
        {
          RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Upper.device_address); 
          while (1) { }; // STOP ardurino
        }
      }  
    }
    else
    {
      I2C_HANDLE_Current_TCA9548A_device_address = Footplate_Left.I2C_HANDLE_TCA9548A_Lower.device_address;
      while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, pointer, 1) != 0)
      { // Something went wrong
        if (bStatus!=1)
        {
          RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Lower.device_address); 
          while (1) { }; // STOP ardurino
        }
      }  
    }
  }  
  else
  {// Right footplate
    if (UPPER==true)
    {
      I2C_HANDLE_Current_TCA9548A_device_address = Footplate_Right.I2C_HANDLE_TCA9548A_Upper.device_address;
      while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, pointer, 1) != 0)
      { // Something went wrong
        if (bStatus!=1)
        {
          RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Upper.device_address); 
          while (1) { }; // STOP ardurino
        }
      }  
    }
    else
    {
      I2C_HANDLE_Current_TCA9548A_device_address = Footplate_Right.I2C_HANDLE_TCA9548A_Lower.device_address;
      while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, pointer, 1) != 0)
      { // Something went wrong
        if (bStatus!=1)
        {
          RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Lower.device_address); 
          while (1) { }; // STOP ardurino
        }
      }  
    }    
  }
}
