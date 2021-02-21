void I2C_Select_TCA9548A (byte ChannelsToActivateLeftUpper, byte ChannelsToActivateLeftLower, byte ChannelsToActivateRightUpper,byte ChannelsToActivateRightLower)
{
  byte bStatus;
  /*
  if (ChannelsToActivate > 63) // There are only 0...7 channels for each TCA9548A I2C multiplexer. But only six of them (0..5) are being used. Keep in mind this is a bitmask. So value 3 will enable channel 0 and channel 1.
  { // Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_CODE, 1, 0); // Tell the host computer that there is an internal error in the arduino source code, because this should never happen. Looks like the programmer did a mistake.
    while (1) { }; // STOP ardurino
  }
  */
  
  while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, &cValue[ChannelsToActivateLeftUpper], 1) != 0)
  { // Something went wrong
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit an try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Upper.device_address); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }

  while (bStatus = nI2C->Write (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, &cValue[ChannelsToActivateLeftLower], 1) != 0)
  { // Something went wrong
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit an try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Left.I2C_HANDLE_TCA9548A_Lower.device_address); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }

  while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, &cValue[ChannelsToActivateRightUpper], 1) != 0)
  { // Something went wrong
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit an try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Upper.device_address); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }

  while (bStatus = nI2C->Write (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, &cValue[ChannelsToActivateRightLower], 1) != 0)
  { // Something went wrong
    if (bStatus==1) // I2C library is busy
    {
      // Wait a bit an try again
    }
    else
    {
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, Footplate_Right.I2C_HANDLE_TCA9548A_Lower.device_address); // Tell the host computer that there is an issue
      while (1) { }; // STOP ardurino
    }
  }

}
