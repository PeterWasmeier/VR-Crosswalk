// -----------------------------------------------------------------
// I2C_Select_TCA9548A
// -----------------------------------------------------------------
// On the I2C bus, there are four TCA9548A I2C multiplexer.
// This function is switching to another TCA9548A I2C multiplexer.
// Keep in mind: on each I2C multiplexer there are six sensors
// connected. One on each channel from 0 to 5 of this multiplexer.
// So there are six channels in use for each TCA9548A.
// -----------------------------------------------------------------
void I2C_Select_TCA9548A (CI2C::Handle HandleOfTCA9548AToActivate, byte ChannelsToActivate)
{
  byte bStatus;
  if (ChannelsToActivate > 63) // There are only 0...7 channels for each TCA9548A I2C multiplexer. But only six of them (0..5) are being used. Keep in mind this is a bitmask. So value 3 will enable channel 0 and channel 1.
  { // Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_CODE, 1, 0); // Tell the host computer that there is an internal error in the arduino source code, because this should never happen. Looks like the programmer did a mistake.
    while (1) { }; // STOP ardurino
  }
  // Disable the current TCA9548A I2C multiplexer, if needed:
  if ((I2C_HANDLE_Current_TCA9548A.device_address > 0) && (I2C_HANDLE_Current_TCA9548A.device_address != HandleOfTCA9548AToActivate.device_address))
  {
    if (bStatus = nI2C->Write (I2C_HANDLE_Current_TCA9548A, &cValue[0], 1) != 0) // Tell this TCA9548A to disable all channels.
    { // Something went wrong
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, I2C_HANDLE_Current_TCA9548A.device_address); // Tell the host computer that there is something wrong
      while (1) { }; // STOP ardurino
    }
  }
  // Now activate the new TCA9548A multiplexer with the correct channel:
  if (bStatus = nI2C->Write (HandleOfTCA9548AToActivate, ChannelsToActivate, 1) != 0)
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, HandleOfTCA9548AToActivate.device_address); // Tell the host computer that there is an issue
    while (1) { }; // STOP ardurino
  }
  I2C_HANDLE_Current_TCA9548A = HandleOfTCA9548AToActivate;
  I2C_CHANNEL_Current_TCA9548A = ChannelsToActivate;
}
