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
  message.CRC = FrameID ^ message.data[0] ^ message.data[1] ^ message.data[2] ^ message.data[3]; // Calculate the CRC
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
  //RS232_SendMessage (FrameID, message);
  Serial.println ();
  Serial.print ("Error FrameID=0x");
  Serial.print (FrameID,HEX);
  Serial.print (", Errornumber=0x");
  Serial.print (Errornumber,HEX);
  Serial.print (", ErrorSource=0x");
  Serial.println (ErrorSource,HEX);
  Serial.flush();
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
  // Check if the Bluetooth module HC05 is in "AT" mode:
  Serial.begin (38400);                       // This is the default speed in AT mode
  //delay (1000);                               // Wait for a second, so that HC05 has enough time to switch into AT mode (if button is pressed)
  Serial.println ("AT");                      // Check if HC05 is in AT mode
  //delay (1000);                               // Wait for a bit
  if (Serial.available() >= 2)                // There is an answer
  { // Bluetooth module seems to be in AT Mode
    Serial.println ("AT+UART=38400,0,0");     // Change baud rate to 38400
    delay (2000);
    Serial.println ("AT+NAME=VR-Crosswalk");  // Change its name to VR-Crosswalk
    delay (2000);
    while (1) {}; // STOP                      // STOP arduino, need to be restarted
  }
}

// -----------------------------------------------------------------
// RS232_loop
// -----------------------------------------------------------------
// Handles the communication with the host computer.
// -----------------------------------------------------------------
void RS232_loop ()
{
  RS232Message message;
  // Check if there is something received from the host computer:
  if (Serial.available() >= sizeof(RS232Message))
  { // Data received from the host computer
    if (Serial.readBytes(&message.id, sizeof(message)) == sizeof(RS232Message))
    { // Message is read
      // XXX add some stuff here:
    }
    else
    { // Something went wrong
      RS232_SendError (RS232_FRAMEID_ERROR_RS232, 1, 0); // tell the host computer that there is something wrong. Should never be executed.
    }
  }
}
