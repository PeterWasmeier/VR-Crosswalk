void RS232_Debug16 (int Value1, int Value2)
{
  RS232Message message;
  message.data16[0] = Value1;
  message.data16[1] = Value2;
  RS232_SendMessage (RS232_FRAMEID_DEBUG_16BIT, message);
  Serial.flush();
}

void RS232_Debug32 (long Value)
{
  RS232Message message;
  message.data32 = Value;
  RS232_SendMessage (RS232_FRAMEID_DEBUG_32BIT, message);
  Serial.flush();
}

// -----------------------------------------------------------------
// RS232_SendMessage
// -----------------------------------------------------------------
// Sends a data package to the host computer. Each datapackage is
// always 6 bytes long (sizeof(message)). Doesnt matter if there
// are parameters used or not.
// -----------------------------------------------------------------
void RS232_SendMessage (byte FrameID,  RS232Message message)
{
  message.id = FrameID;
  message.CRC = message.id ^ message.data[0] ^ message.data[1] ^ message.data[2] ^ message.data[3]; // Calculate the CRC
  Serial.write (0xFF);
  Serial.write (&message.id, sizeof(message));
}

// -----------------------------------------------------------------
// RS232_RaiseError
// -----------------------------------------------------------------
// Transfer an error data package to the host computer
// The error datapackage contains only one parameter. The second
// parameter is not being used.
// -----------------------------------------------------------------
void RS232_SendError (byte FrameID, unsigned int Errornumber, unsigned int ErrorSource)
{
  RS232Message message;
  message.data16[0] = Errornumber;
  message.data16[1] = ErrorSource;
  RS232_SendMessage (FrameID, message);
  Serial.flush ();
}

// -----------------------------------------------------------------
// RS232_setup
// -----------------------------------------------------------------
// Because the bluetooth module is connected to TX/RX, there is
// nothing special to do, just set the baudrate to its default
// speed.
// -----------------------------------------------------------------
void RS232_setup ()
{
  byte cByte;
  Serial.begin (38400,SERIAL_8N1);                       // This is the default speed in AT mode
  // Empty receive buffer:
  while (Serial.available()>0)
  {
    Serial.readBytes (&cByte,1);
  }
  // Wait until HC05 is up and running:
  delay (1000);                               // Wait for a second, so that HC05 has enough time to switch into AT mode (if button is pressed)
  // Check if the Bluetooth module HC05 is in "AT" mode:
  Serial.println ("AT");
  delay (1000);                               // Wait for a bit
  cByte = Serial.available();
  if (cByte == 4)                // There is an answer
  { // Bluetooth module seems to be in AT Mode
    Serial.println ("AT+UART=38400,0,0");     // Change baud rate to 38400
    delay (2000);
    Serial.println ("AT+NAME=VR-Crosswalk");  // Change its name to VR-Crosswalk
    delay (2000);
    RS232_SendError (RS232_FRAMEID_ERROR_RS232,4,0); // // ERROR: please restart the device
    while (1) { }; // STOP                      // STOP arduino, need to be restarted
  }
}

bool RS232_Checksum (RS232Message *message)
{
  byte checksum;
  checksum = message->id ^ message->data[0] ^ message->data[1] ^ message->data[2] ^ message->data[3];
  return (checksum==message->CRC);
}

// -----------------------------------------------------------------
// RS232_loop
// -----------------------------------------------------------------
// Handles the communication with the host computer.
// -----------------------------------------------------------------
void RS232_loop ()
{
  RS232Message message;
  static byte StartCodeReceived=0;
  // Check if there is something received from the host computer:
  
  if (StartCodeReceived==0)
  {
    if (Serial.available()>0)
    {
      if (Serial.readBytes(&message.id,1)==1)
      {
        if (message.id==0xFF)
        {
          StartCodeReceived=1;
        }
      }
    }
  }
  
  if (StartCodeReceived==1)
  {
    if (Serial.available() >= sizeof(RS232Message))
    { // Data received from the host computer
      if (Serial.readBytes(&message.id, sizeof(message)) == sizeof(RS232Message))
      { // Message is read
        // Check if the checksum is correct:
        StartCodeReceived=false;
        if (RS232_Checksum (&message)==false)
        {
          RS232_SendError (RS232_FRAMEID_ERROR_RS232, 2, 0); // ERROR: received checksum (CRC) is wrong from the previous message
          // Clear the receive buffer:
          while (Serial.available())
          {
            Serial.readBytes (&message.id,1);
          }
        }
        else
        {
          switch (message.id)
          {
            default: // Unknown message
              RS232_SendError (RS232_FRAMEID_ERROR_RS232, 3, 0); // ERROR: unknown message received. FRAMEID is unknown.
              break;
            case RS232_FRAMEID_VBUS_VOLTAGE:
              message.id = RS232_FRAMEID_VBUS_VOLTAGE;
              message.dataFloat = ODrive.VBus_Voltage;
              RS232_SendMessage (RS232_FRAMEID_VBUS_VOLTAGE, message);
              break;          
            case RS232_FRAMEID_ENCODER_AXIS0:
              message.id = RS232_FRAMEID_ENCODER_AXIS0;
              message.data32 = ODrive.Axis0.EncoderShadowCount;
              RS232_SendMessage (RS232_FRAMEID_ENCODER_AXIS0, message);
              break;
            case RS232_FRAMEID_ENCODER_AXIS1:
              message.id = RS232_FRAMEID_ENCODER_AXIS1;
              message.data32 = ODrive.Axis1.EncoderShadowCount;
              RS232_SendMessage (RS232_FRAMEID_ENCODER_AXIS1, message);
              break;
            case RS232_FRAMEID_ROTATE: // Rotate the device by the given parameter in units
              ODrive.Axis0.Targetposition = message.data32;
              ODrive.Axis0.ExecuteMovement=true;
              break;
            case RS232_FRAMEID_MOVE: // Move the device by the given parameter in units
              ODrive.Axis1.Targetposition = message.data32;
              ODrive.Axis1.ExecuteMovement=true;
              break;
          }
        } 
      }
      else
      { // Something went wrong
        RS232_SendError (RS232_FRAMEID_ERROR_RS232, 1, 0); // ERROR: received data does not match the buffersize
        while (1) { };
      }
    }
  }
}
