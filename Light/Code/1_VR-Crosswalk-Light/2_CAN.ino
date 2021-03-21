// -----------------------------------------------------------------
// CAN_setup
// -----------------------------------------------------------------
// Initialising SPI and CAN bus. SPI will use 10mbit/second,
// and the CAN bus will use 250kbit/second (ODRIVE default speed).
// -----------------------------------------------------------------
void CAN_setup ()
{
  // Init stuff for SPI/CAN Bus communication with ODRIVE
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  SPI.begin ();
  ACAN2515Settings CANsettings (8UL * 1000UL * 1000UL, 250UL * 1000UL); // Crystal of MCR2515 is 8MHZ, CAN Bus speed has to be 250kb/s
  CANsettings.mRequestedMode = ACAN2515Settings::NormalMode; // Tell the CAN bus module, that we want to communicate in real (with ODRIVE)
  CANsettings.mReceiveBufferSize = 1;
  CANsettings.mTransmitBuffer0Size = 1;
  const uint16_t errorCode = can.begin (CANsettings, NULL) ; // ISR is null, we dont use interrupt for the CAN bus.
  if (errorCode != 0)
  {
    RS232_SendError (RS232_FRAMEID_ERROR_CANBUS, errorCode, 0); // Tell the host computer, that there is an issue with the CAN bus.
    while (1) { }; // STOP, arduino needs to be restarted
  }
}


// -----------------------------------------------------------------
// CAN_loop
// -----------------------------------------------------------------
// Polling the CAN bus, checking if ODRIVE has something send to
// this arduino. This function is also being used to ask the ODRIVE
// some stuff. 
// -----------------------------------------------------------------
void CAN_loop ()
{
  static byte index=0;
  float tempfloat;
  CANMessage message;
  // Check if data is received from ODRIVE over the CAN-bus:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (can.receiveErrorCounter()>0)
  {
    RS232_SendError (RS232_FRAMEID_ERROR_CANBUS,0,1); // ERROR: Receive error 
    while (1) { };
  }
  if (can.transmitErrorCounter()>0)
  {
    RS232_SendError (RS232_FRAMEID_ERROR_CANBUS,0,1); // ERROR: Send error
    while (1) { };
  }
  can.poll () ;         // Because we do not use interrupt for the can-bus, we have to poll the can bus.
  if (can.available ())
  { // Something was received from the CAN-bus:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    can.receive (message);
    switch (message.id)
    {
      case  ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Heartbeat_Message:  // Heartbeat message from AXIS0
        // Message from AXIS0 with its current state and axis error:
        ODrive.Axis0.AxisError = message.data32[0];
        ODrive.Axis0.AxisCurrentState = message.data32[1];
        break;
      case  ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Heartbeat_Message:  // Heartbeat message from AXIS1
        // Message from AXIS1 with its current state and axis error:
        ODrive.Axis1.AxisError = message.data32[0];
        ODrive.Axis1.AxisCurrentState = message.data32[1];
        break;
      case ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_Encoder_Count: // Encoder position from AXIS0
        // Message from AXIS0 with its current encoder value:
        ODrive.Axis0.EncoderShadowCount = message.data32[0];
        ODrive.Axis0.EncoderCountinCPR  = message.data32[1];
        break;
      case ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_Encoder_Count:  // Encoder position from AXIS1
        // Message from AXIS1 with its current encoder value:
        ODrive.Axis1.EncoderShadowCount = message.data32[0];
        ODrive.Axis1.EncoderCountinCPR  = message.data32[1];
        break;
      case ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_VBus_Voltage:  // Message from ODRIVE with the current power supply voltage
      case ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_VBus_Voltage:
        // Message from ODRIVE with its current power supply voltage:
        ODrive.VBus_Voltage = message.dataFloat[0];
        break;
    }
    // Here comes the send/polling part to ask ODRIVE something:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    switch (index)  // We ask the ODRIVE each available parameter in a row, we ask all the time the same stuff:
    {
      case 0: // Ask for the current encoder position of AXIS0 (rotation axis)
        message.id = ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_Encoder_Count;
        message.rtr = true;
        message.len = 0;
        if (can.tryToSend (message)) index++; // Next stuff to ask for, in the next cycle
        break;
      case 1: // Ask for the current encoder position of AXIS1 (horizontal axis)
        message.id = ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_Encoder_Count;
        message.rtr = true;
        message.len = 0;
        if (can.tryToSend (message)) index++; // Next stuff to ask for, in the next cycle
        break;
      case 2: // Ask for the current voltage of the power supply from the ODRIVE
        message.id = ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_VBus_Voltage;
        message.rtr = true;
        message.len = 0;
        if (can.tryToSend (message)) index++; 
        break;
      case 3: // Is there a need to rotate the device?
        if (ODrive.Axis0.ExecuteMovement==true)
        {
          tempfloat = ODrive.Axis0.Targetposition; // XXX
          message.id = ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Set_Input_Pos;
          message.rtr = false;
          message.len = 0x04;
          message.dataFloat[0]=tempfloat; // the desired position, in [turns]
          if (can.tryToSend (message)) 
          {
            ODrive.Axis0.ExecuteMovement=false;
            index++; 
          }
        }
        else
        {
          index++;
        }
        break;
      case 4: // Is there a need to move?
        if (ODrive.Axis1.ExecuteMovement==true)
        {
          tempfloat = ODrive.Axis1.Targetposition; // XXX
          message.id = ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Set_Input_Pos;
          message.rtr = false;
          message.len = 0x04;
          message.dataFloat[0]=tempfloat; // the desired position, in [turns]
          if (can.tryToSend (message)) 
          {
            ODrive.Axis1.ExecuteMovement=false;
            index++; 
          }
        }
        else
        {
          index++;
        }
        break;
      default: // Ups, something went wrong, but we don't care. We just repeat the loop from the beginning (no need to raise an emergency stop)
        index = 0;
        break;
    }
  }
}
