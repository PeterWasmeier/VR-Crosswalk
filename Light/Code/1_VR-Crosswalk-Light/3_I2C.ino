// -----------------------------------------------------------------
// I2C_GetBufferpointer
// -----------------------------------------------------------------
// The I2C library needs a global variable with a pointer to the
// data what is send. We will fill a send buffer here with the
// value to send and return a pointer to this global variable.
// When the buffer is full, we wait until I2C is not busy anymore
// and we clear the buffer.
// -----------------------------------------------------------------
unsigned char *I2C_GetBufferpointer (byte Value)
{
  // First we check, if this value is already inside the buffer:
  byte i;
  for (i=0;i<sizeof(I2CSendBuffer);i++)
  {
    if (I2CSendBuffer[i]==Value)
    {
      return &I2CSendBuffer[i];
    }
  }
  // This value is new, try to add it to the buffer:
  if (I2CSendBufferSize==sizeof(I2CSendBuffer))
  {
    // I2C Sendbuffer is already full, we have to wait:
    while (nI2C->IsCommActive()) { };
    I2CSendBufferSize=0;
  }
  I2CSendBuffer[I2CSendBufferSize]=Value;
  I2CSendBufferSize++;
  return &I2CSendBuffer[I2CSendBufferSize-1];
}

// -----------------------------------------------------------------
// I2C_loop
// -----------------------------------------------------------------
// Call this function very often. It will communicate with each
// footplate and with each of its sensors.
// Once the data is collected from all sensors of all footplates,
// it will begin to calculate where all the axis have to move to.
// -----------------------------------------------------------------
void I2C_loop ()
{
  static byte index = 0;
  switch (index)
  {
    case 0: // read sensor value of left footplate
      if (I2C_Footplate_Read_Sensors (&Footplate_Left)==true) index=2; // XXX ++ instead
      break;
    case 1: // read sensor value of right footplate
      if (I2C_Footplate_Read_Sensors (&Footplate_Right)==true) index++;
      break;
    case 2: // all sensor values are available
      // XXX Do the calculation:
      index=0;
      break;
  }
}

// -----------------------------------------------------------------
// I2C_Footplate_Read_Sensors
// -----------------------------------------------------------------
// Will read all the sensor values of one footplate.
// Return true in case all sensor values are available.
// False means not yet finished.
// Keep in mind: each footplate has got 12 sensors and there are
// two TCA9548A I2C multiplexer for each footplate. Each TCA9548A
// has got 8 I2C channels, but only six of them are being used:
// Channel  Device
// ---------------
// 0:   GY-271/HMC5883L
// 1:   VL6180X
// 2:   VL6180X
// 3:   GY-271/HMC5883L
// 4:   GY-271/HMC5883L
// 5:   VL6180X
// -----------------------------------------------------------------
bool I2C_Footplate_Read_Sensors (TFootplate *Footplate)
{
  static byte index = 0;
  static unsigned long msStart=0;
  static unsigned long msStop=0;
  switch (index)
  {
    default:
      index=0;
      break;
    case 0: // Activate Channel 0 and 1 of the upper TCA9548A at once:
      msStart = millis ();
      Footplate->GY271[0].Valid=true;
      Footplate->GY271[1].Valid=true;
      Footplate->GY271[2].Valid=true;
      Footplate->GY271[3].Valid=true;
      Footplate->GY271[4].Valid=true;
      Footplate->GY271[5].Valid=true;
      Footplate->VL6180X[0].Valid=true;
      Footplate->VL6180X[1].Valid=true;
      Footplate->VL6180X[2].Valid=true;
      Footplate->VL6180X[3].Valid=true;
      Footplate->VL6180X[4].Valid=true;
      Footplate->VL6180X[5].Valid=true;
      I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[0]);
      I2C_GY271_Read_Values (&Footplate->GY271[0]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[0]);
      index++;
      break;
    case 1: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[0].Valid == true) && (Footplate->VL6180X[0].Valid == true)) index++;
      break;  
    case 2: // Activate Channel 2 and 3 of the upper TCA9548A at once:
      I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[1]);
      I2C_GY271_Read_Values (&Footplate->GY271[1]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[1]);
      index++;
      break;
    case 3: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[1].Valid == true) && (Footplate->VL6180X[1].Valid == true)) index++;
      break;
    case 4: // Activate Channel 4 and 5 of the upper TCA9548A at once:
      I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[2]);
      I2C_GY271_Read_Values (&Footplate->GY271[2]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[2]);
      index++;
      break;
    case 5: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[2].Valid == true) && (Footplate->VL6180X[2].Valid == true)) index++;
      break;
      

    case 6: // Activate Channel 0 and 1 of the lower TCA9548A at once:
      I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[3]);
      I2C_GY271_Read_Values (&Footplate->GY271[3]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[3]);
      index++;
      break;
    case 7: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[3].Valid == true) && (Footplate->VL6180X[3].Valid == true)) index++;
      break;
      
    case 8: // Activate Channel 2 and 3 of the lower TCA9548A at once:
      I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[4]);
      I2C_GY271_Read_Values (&Footplate->GY271[4]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[4]);
      index++;
      break;
    case 9: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[4].Valid == true) && (Footplate->VL6180X[4].Valid == true)) index++;
      break;
    case 10: // Activate Channel 4 and 5 of the lower TCA9548A at at once:
      I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5);
      I2C_VL6180X_StartMeasurement (&Footplate->VL6180X[5]);
      I2C_GY271_Read_Values (&Footplate->GY271[5]); 
      I2C_VL6180X_ReadValues (&Footplate->VL6180X[5]);
      index++;
      break;
    case 11: // Wait until the value of both sensors has been received:
      if ((Footplate->GY271[5].Valid == true) && (Footplate->VL6180X[5].Valid == true)) 
      {
        index=0;
        msStop=millis();
        //RS232_Debug16 (Footplate->VL6180X[4].ErrorCode,0);//Footplate->VL6180X[4].SignalRate);
        return true; // Finished
      }
      break;
  }
  return false; // Not yet finished
}

// -----------------------------------------------------------------
// I2C_GetErrorSource
// -----------------------------------------------------------------
// This function is calculating the I2C source, where an I2C
// error happened.
// The result are two byte values in one unsigned int.
// The lower 8 bit of the result contains the channel of the TCA9548A,
// the higher 8 bit contains the I2C bus adress of the TCA9548A.
// If the lower 8 bits are zero, the error happend at the TCA9548A
// itself.
// [I2C Address of a TCA9548A] [CHANNEL as a bitmask]
// Now it depends, what is connected to this channel:
// Channel  Device
// ---------------
// 0:   GY-271/HMC5883L
// 1:   VL6180X
// 2:   VL6180X
// 3:   GY-271/HMC5883L
// 4:   GY-271/HMC5883L
// 5:   VL6180X

// Return value:
// 0x7000, left footplate, upper TCA9548A
// 0x7001, left footplate, upper TCA9548A, channel 0 (GY-271)
// 0x7002, left footplate, upper TCA9548A, channel 1 (VL6180X)
// 0x7004, left footplate, upper TCA9548A, channel 2 (VL6180X)
// 0x7008, left footplate, upper TCA9548A, channel 3 (GY-271)
// 0x7010, left footplate, upper TCA9548A, channel 4 (GY-271)
// 0x7020, left footplate, upper TCA9548A, channel 5 (VL6180X)
// 0x7100, left footplate, lower TCA9548A
// 0x7101, left footplate, lower TCA9548A, channel 0 (GY-271)
// 0x7102, left footplate, lower TCA9548A, channel 1 (VL6180X)
// 0x7104, left footplate, lower TCA9548A, channel 2 (VL6180X)
// 0x7108, left footplate, lower TCA9548A, channel 3 (GY-271)
// 0x7110, left footplate, lower TCA9548A, channel 4 (GY-271)
// 0x7120, left footplate, lower TCA9548A, channel 5 (VL6180X)
// 0x7400, right footplate, upper TCA9548A
// 0x7401, right footplate, upper TCA9548A, channel 0 (GY-271)
// 0x7402, right footplate, upper TCA9548A, channel 1 (VL6180X)
// 0x7404, right footplate, upper TCA9548A, channel 2 (VL6180X)
// 0x7408, right footplate, upper TCA9548A, channel 3 (GY-271)
// 0x7410, right footplate, upper TCA9548A, channel 4 (GY-271)
// 0x7420, right footplate, upper TCA9548A, channel 5 (VL6180X)
// 0x7500, right footplate, lower TCA9548A
// 0x7501, right footplate, lower TCA9548A, channel 0 (GY-271)
// 0x7502, right footplate, lower TCA9548A, channel 1 (VL6180X)
// 0x7504, right footplate, lower TCA9548A, channel 2 (VL6180X)
// 0x7505, right footplate, lower TCA9548A, channel 3 (GY-271)
// 0x7510, right footplate, lower TCA9548A, channel 4 (GY-271)
// 0x7520, right footplate, lower TCA9548A, channel 5 (VL6180X)
// -----------------------------------------------------------------
unsigned int I2C_GetErrorSource (byte device_address)
{
  byte channel;
  unsigned int result;
  if (device_address == I2C_HANDLE_GY271.device_address) // There is an issue with GY271?
  { // The GY271 can be connected to channel 0,3 or 4 of any TCA9548A:
    // Channel  Device
    // ---------------
    // 0:   GY-271/HMC5883L
    // 1:   VL6180X
    // 2:   VL6180X
    // 3:   GY-271/HMC5883L
    // 4:   GY-271/HMC5883L
    // 5:   VL6180X
    channel = I2C_CHANNEL_Current_TCA9548A & (1 + 8 + 16);
  }
  else if (device_address == I2C_HANDLE_VL6180X.device_address) // There is an issue with VL6180X?
  { // The VL6180X can be connected to channel 1,2 or 5 of any TCA9548A:
    // Channel  Device
    // ---------------
    // 0:   GY-271/HMC5883L
    // 1:   VL6180X
    // 2:   VL6180X
    // 3:   GY-271/HMC5883L
    // 4:   GY-271/HMC5883L
    // 5:   VL6180X
    channel = I2C_CHANNEL_Current_TCA9548A & (2 + 4 + 32);
  }
  // XXX TCA itself are missing
  else
  {
    RS232_SendError (RS232_FRAMEID_ERROR_CODE, 1, 0); // ERROR: parameter 'device_address' in function 'I2C_GetErrorSource' is invalid.
    while (1) { }; // STOP arduino
  }
  device_address = I2C_HANDLE_Current_TCA9548A_device_address;
  result = device_address;
  result = result << 8;
  result = result + channel;
  return result;
}

// -----------------------------------------------------------------
// I2C_setup
// -----------------------------------------------------------------
// Will initialize the I2C bus with all of its components.
// Take care, that both footplates are connected to the arduino I2C
// bus. Do not forget to use the DIP switches on each footplate
// to define the I2C address. Each footplate must have a different
// DIP switch position. Otherwise the arduino will raise an error
// to the host computer and will stop to work.
// I2C bus speed will be 400khz/400kbps.
// -----------------------------------------------------------------
void I2C_setup ()
{
  byte bStatus;
  I2C_CHANNEL_Current_TCA9548A=0;
  nI2C->SetTimeoutMS(50); // use 5ms as a timeout for any I2C communication
  // For each I2C device on the bus, we have to create a handle.
  // The handles will be used to read and write later on over the I2C bus.
  // There are four TCA9548A I2C multiplexer in total, two on each footplate:
  Footplate_Left.I2C_HANDLE_TCA9548A_Upper  = nI2C->RegisterDevice(0x70, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x70
  Footplate_Left.I2C_HANDLE_TCA9548A_Lower  = nI2C->RegisterDevice(0x71, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x71
  Footplate_Right.I2C_HANDLE_TCA9548A_Upper = nI2C->RegisterDevice(0x74, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x74
  Footplate_Right.I2C_HANDLE_TCA9548A_Lower = nI2C->RegisterDevice(0x75, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x75
  // We need only one I2C handle for all GY271 and VL6180X sensors, because they are separated by an TCA9548A I2C multiplexer
  I2C_HANDLE_GY271   = nI2C->RegisterDevice(0x0D, 1, CI2C::Speed::FAST); // There is only one I2C handle for all the GY271 sensors, because I2C multiplexer are being used
  I2C_HANDLE_VL6180X = nI2C->RegisterDevice(0x29, 2, CI2C::Speed::FAST); // There is only one I2C handle for all the VL6180X sensors, because I2C multiplexer are being used
  // Initialize all the GY-271/HMC5883L sensors on each footplate. At TCA9548A channel 0, 3 and 4 are GY271 connected:
  // Channel Value Sensor
  // 0:      1     GY-271/HMC5883L
  // 1:      2     VL6180X
  // 2:      4     VL6180X
  // 3:      8     GY-271/HMC5883L
  // 4:      16    GY-271/HMC5883L
  // 5:      32    VL6180X
  // Left footplate, upper TCA9548A I2C multiplexer
  // First, disable all TCA9548A channels:
  I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (true,true, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 

  // Left footplate, lower TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (true,false, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5);
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 

/*
  // Right footplate, upper TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1); // Enable Channel 0 and 1
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 

  // Right footplate, lower TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, TCA9548A_CHANNEL_0 + TCA9548A_CHANNEL_1); // Enable Channel 0 and 1
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, TCA9548A_CHANNEL_2 + TCA9548A_CHANNEL_3); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, TCA9548A_CHANNEL_4 + TCA9548A_CHANNEL_5); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor
  I2C_VL6180X_Init (); // Initialize this VL6180X sensor 
  */
}
