// VR-Crosswalk Light
// This is the arduino source code for the VR-Crosswalk Light
// The IDE, which was used to program this source code, is Version 1.8.9
// You will need two additional librarys to compile this 
// source code:
// "nI2C": see https://github.com/nitacku/nI2C
// "ACAN2515": see https://github.com/pierremolinaro/acan2515

// IMPORTANT: this source code is not yet finished.
// ================================================
// Note: if you see "// XXX" in the sourcecode, this line is still not finished, need to think about

// Header files for "ACAN2515":
#include <ACAN2515.h>
#include <ACAN2515Settings.h>
#include <ACANBuffer.h>
#include <ACANBuffer16.h>
#include <CANMessage.h>
#include <MCP2515ReceiveFilters.h>

// Header files for "nI2C":
#include <avrlibdefs.h>
#include <nI2C.h>
#include <nTWI.h>
#include <queue.h>

// Other header files:
#include <SPI.h>

// Communication ARDUINO <=> ODRIVE via canbus
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The ODRIVE is connected via canbus to the arduino.
// In order to communicate with the ODRIVE, the node id
// of each ODRIVE axis must be configured as follow:
// odrv0.axis0.config.can_node_id = 0
// odrv0.axis1.config.can_node_id = 1
// odrv0.can.set_baud_rate(250000)
// odrv0.save_configuration()
// odrv0.reboot()
// Use ODRIVETOOL to do that while ODRIVE is connected via 
// USB cable to your computer.
// Do not forget to enable both terminator resistors:
// one at the ODRIVE, the other one on arduino side.
// axis0 is responsible for the rotation of VR-Crosswalk
// axis1 is responsible for the footplate position/horizontal axis

// Communication ARDUINO <=> HOST computer
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// This is done by using the HC05 bluetooth device (RS232).
// Before you power on the VR-Crosswalk the first
// time, press the button on the HC05 to enable the AT
// mode of the HC05. The arduino will set the baudrate 
// to 38400 and will change the name of the bluetooth 
// device automatically to "VR-Crosswalk".
// Otherwise arduino is not able to communicate with HC05,
// because by default its baudrate is 9600 instead
// of 38400.
// To connect the host computer via bluetooth to the
// HC05 module, use the PIN: 1234
// The baudrate of 38400 bits per seconds allows to send
// and receive 800 datapackages per second, because each
// datapackage is always 6 bytes long.

// Communication ARDUINO <=> Footplates
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// There are two footplates, one for each foot. On such
// a footplate are 12 sensors mounted. They will
// communicate via I2C with the arduino.
// Take care that each footplate is configured as "left" or
// "right". For that, use the DIP switches on each 
// footplate. So, the left footplate must be configured
// as left (using the DIP switch on the footplate itself)
// and right for the other one.
// On each footplate, there are two TCA9548A I2C 
// multiplexer. Each of them is talking to three VL6180X
// and three GY-271/HMC5883L sensors.
// The TCA9548A are separated as "upper" and "lower".
// Each of them use 6 channels, to communicate with:
// Channel	Device
// ---------------
// 0:		GY-271/HMC5883L
// 1:		VL6180X
// 2:		VL6180X
// 3:		GY-271/HMC5883L
// 4:		GY-271/HMC5883L
// 5:		VL6180X
// The two TCA9548A multiplexer are called "upper" and "lower" 
// in this sourcecode.

// global variable stuff for bluetooth/Host computer communication
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Each datapackage, between host computer and the arduino, has got
// a FRAMEID. Telling the host or arduino, what kind of datapackage
// it is. Some datapackages have none, 1, 2 or 4 parameters, depending
// on the FRAMEID. But anyway: each datapackage is 6 bytes long. 
// Doesn't matter is a parameter is being used or not.
// Here are the FRAMEIDs:
const byte RS232_FRAMEID_ERROR_CANBUS = 1; // Message to host computer: Error with CAN-bus.  
											// The error code contains a set of bits. Each bit represents a single error:
											// 1.Parameter (Errornumber as a bitmask):
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
											// Bit	Static constant
											// 0	kNoMCP2515 
											// 1	kTooFarFromDesiredBitRate 
											// 2	kInconsistentBitRateSettings 
											// 3	kINTPinIsNotAnInterrupt 
											// 4	kISRIsNull
											// 5	kRequestedModeTimeOut
											// 6	kAcceptanceFilterArrayIsNULL
											// 7	kOneFilterMaskRequiresOneOrTwoAcceptanceFilters
											// 8	kTwoFilterMasksRequireThreeToSixAcceptanceFilters 
											// 9	kCannotAllocateReceiveBuffer
											// 10	kCannotAllocateTransmitBuffer0
											// 11	kCannotAllocateTransmitBuffer1
											// 12	kCannotAllocateTransmitBuffer2
											// 13	kISRNotNullAndNoIntPin
											// Example: if the value is 5, bit 0 and bit 2 are true and both errors are active at the same time.
											// For more details see chapter "11.3 The error code" in the document: acan2515.pdf, page 25.
											//
											// 2.Parameter (ErrorSource):
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~
											// always zero
											
const byte RS232_FRAMEID_ERROR_RS232  = 2; // Message to host computer: Error with RS232 interface. There are no parameters.
											// 1.Parameter (Errornumber):
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~
											// 1: could not get all the received data from internal RS232 buffer
											//
											// 2.Parameter (ErrorSource):
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~
											// always zero
											
const byte RS232_FRAMEID_ERROR_I2C    = 3; // Message to host computer: Error with I2C interface. The first parameter contains an error number (see below).
											// 1.Parameter (Errornumber)
											// ~~~~~~~~~~~~~~~~~~~~~~~~~
											// 0:success
											// 1:busy
											// 2:timeout
											// 3:data too long to fit in transmit buffer
											// 4:memory allocation failure
											// 5:attempted illegal transition of state
											// 6:received NACK on transmit of address
											// 7:received NACK on transmit of data
											// 8:illegal start or stop condition on bus
											// 9:lost bus arbitration to other master
											//
											// 2. Parameter (ErrorSource)
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~
											//  1: Issue happend at the 1. GY-271/HMC5883L sensor on the left footplate
											//  2: Issue happend at the 2. GY-271/HMC5883L sensor on the left footplate
											//  3: Issue happend at the 3. GY-271/HMC5883L sensor on the left footplate
											//  4: Issue happend at the 4. GY-271/HMC5883L sensor on the left footplate
											//  5: Issue happend at the 5. GY-271/HMC5883L sensor on the left footplate
											//  6: Issue happend at the 6. GY-271/HMC5883L sensor on the left footplate
											//  7: Issue happend at the 1. GY-271/HMC5883L sensor on the right footplate
											//  8: Issue happend at the 2. GY-271/HMC5883L sensor on the right footplate
											//  9: Issue happend at the 3. GY-271/HMC5883L sensor on the right footplate
											// 10: Issue happend at the 4. GY-271/HMC5883L sensor on the right footplate
											// 11: Issue happend at the 5. GY-271/HMC5883L sensor on the right footplate
											// 12: Issue happend at the 6. GY-271/HMC5883L sensor on the right footplate
											// 13: Issue happend at the 1. VL6180X sensor on the left footplate
											// 14: Issue happend at the 2. VL6180X sensor on the left footplate
											// 15: Issue happend at the 3. VL6180X sensor on the left footplate
											// 16: Issue happend at the 4. VL6180X sensor on the left footplate
											// 17: Issue happend at the 5. VL6180X sensor on the left footplate
											// 18: Issue happend at the 6. VL6180X sensor on the left footplate
											// 19: Issue happend at the 1. VL6180X sensor on the right footplate
											// 20: Issue happend at the 2. VL6180X sensor on the right footplate
											// 21: Issue happend at the 3. VL6180X sensor on the right footplate
											// 22: Issue happend at the 4. VL6180X sensor on the right footplate
											// 23: Issue happend at the 5. VL6180X sensor on the right footplate
											// 24: Issue happend at the 6. VL6180X sensor on the right footplate
											// 25: Issue happend at the TCA9548A, responsible for all GY271, on the right footplate
											// 26: Issue happend at the TCA9548A, responsible for all GY271, on the left footplate
											// 27: Issue happend at the TCA9548A, responsible for all VL6180X, on the right footplate
											// 28: Issue happend at the TCA9548A, responsible for all VL6180X, on the left footplate
											
const byte RS232_FRAMEID_ERROR_CODE = 4; // Message to host computer, that there is something wrong in the source code of this program.
											// 1.Parameter (Errornumber):
											// ~~~~~~~~~~~~~~~~~~~~~~~~~~
											//  1: Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
											//  2: The parameter Footplate for the function I2C_Select_GY271 is invalid. Arduino stopped.
											
const byte RS232_FRAMEID_GETENCODER_AXIS0 = 1; // Message to arduino: Host Computer wants to know the position of the horizontal and rotation axis.

// This is the datapackage, which is being used between host computer and the arduino:
typedef struct
{
  byte id;                     // will contain later on one of the "RS232_FRAMEID..."
  union {                      // Here are the parameters, which can be used/filled:
          uint32_t data32;     // one parameter as unsigned long (32 bit), or
          uint16_t data16[2] ; // up to two parameters as unsigned int (16 bit), or
          float dataFloat[1] ; // one parameter as float
          uint8_t data [4] = {0, 0, 0, 0}; // or four parameters as four bytes
  };
  byte CRC;                    // The CRC is just a XOR of all bytes, including "FRAMEID"
}RS232Message;

// global variable stuff for SPI/CAN bus communication with ODRIVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef struct
{
  byte bPollIndex;                // Ardurino has to poll some stuff from ODRIVE. This is the index, which stuff must be polled. Once this is done, the index is being increased to poll the next data from ODRIVE
} TCanBus;
TCanBus CanBus;                   // Contains all needed stuff which is being used for CAN-bus communication.
ACAN2515 can (10, SPI, 255) ;     // For SPI communication: Chip select pin = 10, Last argument is 255 -> no interrupt pin is being used for can-bus communication

// Global variables for/from ODRIVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Each axis of ODRIVE has got its own node id. 
// By default, Axis0 has got the node id zero.
// The other axis, Axis1, has got the node id one.
const unsigned int ODRIVE_NODE_ID_AXIS0 =  0;      // Upper 6 bit of the can-bus-frame-id is the NODE id of axis0
const unsigned int ODRIVE_NODE_ID_AXIS1 = (1)<<5;  // Upper 6 bit of the can-bus-frame-id is the NODE id of axis1
// The arduino is communicating with ODRIVE using CAN bus.
// For that, there are datapackages being send and received.
// Each datapackage has got an identifier, telling the 
// receiver what kind of content this datapackage has.
// Here are the identifiers listet:
const unsigned int CAN_FRAMEID_Heartbeat_Message = 0x001;
const unsigned int CAN_FRAMEID_Get_Encoder_Count = 0x00A;
const unsigned int CAN_FRAMEID_Get_VBus_Voltage = 0x017;

// The following structure will contain later one the values from each 
// axis of the ODrive. This arduino program will collect all values for 
// each axis into this structur:
typedef struct
{
  unsigned long AxisError;
  unsigned long AxisCurrentState;
  signed long EncoderShadowCount;
  signed long EncoderCountinCPR;
} TODriveAxis;

// General values from the ODRIVE will be stored in this structur:
typedef struct
{
  float VBus_Voltage;
  TODriveAxis Axis0;
  TODriveAxis Axis1;
} TODrive;

TODrive ODrive; // global variable, containing all ODRIVE values, including both axis


// Global stuff for I2C communication
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The arduino is also talking to several I2C devices. They are
// separated by I2C multiplexer.
// Here you see a structure which will later one keep the measured
// values from the magnetic field sensor (GY-271/HMC5883L). 
// There are 6 of them on each footplate:
typedef struct {                    // tGY271_XYZ, contains the measured values for each axis
  bool Valid;                       // Value in "X,Y,Z" are valid.
  int X;                            // Current X value, measured by the GY271 sensor
  int Y;                            // Current Y value, measured by the GY271 sensor
  int Z;                            // Current Z value, measured by the GY271 sensor
} tGY271_Values;

typedef struct {                    // tGY271_XYZ, contains the measured values for each axis
  bool Valid;                       // Value in "Distance" and "SignalStrength" are valid.
  unsigned int Distance;            // Current Distance value in mm
  unsigned int SignalStength;       // Current SignalStrength 
} tVL6180X_Values;

// This structur is being used for each footplate to store all sensor values.
// Because each footplate has got six GY271 sensors and six VL6180X sensors, 
// we have to keep 12 sensor values:
typedef struct {
  tGY271_Values GY271[6];       // This variable will keep the measured values from all six GY271 sensors from one footplate
  tVL6180X_Values VL6180X[6];   // This variable will keep the measured values from all six VL6180X sensors from one footplate
  CI2C::Handle   I2C_HANDLE_TCA9548A_Upper; // Each footplate has got two TCA9548A I2C multiplexer. This one is for the upper one.
  CI2C::Handle   I2C_HANDLE_TCA9548A_Lower; // Each footplate has got two TCA9548A I2C multiplexer. This one is for all lower one.
} TFootplate;

// Because there are two footplates, we have to define both of them. One left and one right one:
TFootplate Footplate_Left;      // This global variable will store all sensor values from the left footplate
TFootplate Footplate_Right;     // This global variable will store all sensor values from the right footplate

CI2C::Handle I2C_Current_TCA9548A;  // This variable contains the handle of the current active TCA9548A I2C multiplexer. There can be only one of them being active at the same time.

// Because we use the library "nI2C", each I2C device (like GY271 or VL6180X) gets its own handle, which is later on needed to communicate with
CI2C::Handle I2C_HANDLE_GY271;   // Each footplate has got six GY271 magnet field sensors. But we have to create only instance, because they are all separated by I2C multiplexer
CI2C::Handle I2C_HANDLE_VL6180X; // Each footplate has got six VL6180X sensor sensors. But we have to create only instance, because they are all separated by I2C multiplexer

// Because the data is transfered in the background (using interrupts), we have to offer a global buffer for the ISR, where the ISR can store the received values:
byte I2C_GY271_ReceiveBuffer[10]; // All bytes, received over I2C from the GY271 sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
byte I2C_VL6180X_ReceiveBuffer[10]; // XXX All bytes, received over I2C from the VL6180X sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
tGY271_Values *I2C_GY271_Value_Pointer;  // This is a pointer to the currently active GY271 sensor. When data is received from this sensor, the function "I2C_GY271_OnDataReceived" will fill this structure with the values from the sensor.
tVL6180X_Values *I2C_VL6180X_Value_Pointer;  // This is a pointer to the currently active VL6180X sensor. When data is received from this sensor, the function "I2C_VL6180X_OnDataReceived" will fill this structure with the values from the sensor.
byte cZero[1]={0}; // This is a global variable containing zero. Needed for the function "nI2C->Write" later on.
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
  Serial.write (&message.id,sizeof(message));
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
  message.data16[0]=Errornumber;
  message.data16[1]=ErrorSource;
  RS232_SendMessage (FrameID,message);
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
  delay (1000);                               // Wait for a second, so that HC05 has enough time to switch into AT mode (if button is pressed)
  Serial.println ("AT");                      // Check if HC05 is in AT mode
  delay (1000);                               // Wait for a bit
  if (Serial.available() >= 2)                // There is an answer
  { // Bluetooth module seems to be in AT Mode
    Serial.println ("AT+UART=38400,0,0");     // Change baud rate to 38400
    delay (2000);
    Serial.println ("AT+NAME=VR-Crosswalk");  // Change its name to VR-Crosswalk
    while (1){}; // STOP                      // STOP arduino, need to be restarted
  }
}

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
  ACAN2515Settings settings (8UL * 1000UL * 1000UL, 250UL * 1000UL); // Crystal of MCR2515 is 8MHZ, CAN Bus speed has to be 250kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode; // Tell the CAN bus module, that we want to communicate in real (with ODRIVE)
  const uint16_t errorCode = can.begin (settings, NULL) ; // ISR is null, we dont use interrupt for the CAN bus.
  if (errorCode != 0) 
  {
    RS232_SendError (RS232_FRAMEID_ERROR_CANBUS,errorCode,0); // Tell the host computer, that there is an issue with the CAN bus.
    while (1) { }; // STOP, arduino needs to be restarted
  }
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
  nI2C->SetTimeoutMS(5); // there is 5ms timeout for any I2C communication
  // For each I2C device on the bus, we have to create a handle.
  // The handles will be used to read and write later on over the I2C bus.  
  // There are four TCA9548A I2C multiplexer in total, two on each footplate:
  Footplate_Left.I2C_HANDLE_TCA9548A_Upper  = nI2C->RegisterDevice(0x70, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x70
  Footplate_Left.I2C_HANDLE_TCA9548A_Lower  = nI2C->RegisterDevice(0x71, 1, CI2C::Speed::FAST); // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x71
  Footplate_Right.I2C_HANDLE_TCA9548A_Upper = nI2C->RegisterDevice(0x74, 1, CI2C::Speed::FAST);   // The I2C adress of this TCA9548A I2C multiplexer is, with correct DIP switch setting on the footplate: 0x74
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
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, 3); // Enable Channel 0 and 1
	I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, 12); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, 48); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  // Left footplate, lower TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, 3); // Enable Channel 0 and 1
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, 12); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Lower, 48); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  // Right footplate, upper TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, 3); // Enable Channel 0 and 1
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, 12); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Upper, 48); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  // Right footplate, lower TCA9548A I2C multiplexer
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, 3); // Enable Channel 0 and 1
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, 12); // Enable Channel 2 and 3
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
  I2C_Select_TCA9548A (Footplate_Right.I2C_HANDLE_TCA9548A_Lower, 48); // Enable Channel 4 and 5
  I2C_GY271_Init (); // Initialize this GY-271/HMC5883L sensor on the left footplate
}

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
  if (ChannelsToActivate>63) // There are only 0...7 channels for each TCA9548A I2C multiplexer. But only six of them (0..5) are being used. Keep in mind this is a bitmask. So value 3 will enable channel 0 and channel 1.
  { // Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_CODE, 1, 0); // Tell the host computer that there is an internal error in the arduino source code, because this should never happen. Looks like the programmer did a mistake.
    while (1) { }; // STOP ardurino
  }
  // Disable the current TCA9548A I2C multiplexer, if needed:
  if (I2C_Current_TCA9548A.device_address!=HandleOfTCA9548AToActivate.device_address)
  {
    if (bStatus=nI2C->Write (I2C_Current_TCA9548A, &cZero[0], 1)!=0) // Tell this TCA9548A to disable all channels.
    { // Something went wrong
      RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, 25); // XXX Tell the host computer that there is something wrong
      while (1) { }; // STOP ardurino
    }
  }
  // Now activate the new TCA9548A multiplexer with the correct channel:
  if (bStatus=nI2C->Write (HandleOfTCA9548AToActivate, ChannelsToActivate, 1)!=0) 
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, 26); // XXX Tell the host computer that there is an issue
    while (1) { }; // STOP ardurino
  }
  I2C_Current_TCA9548A=HandleOfTCA9548AToActivate;
}

// -----------------------------------------------------------------
// I2C_GY271_Write
// -----------------------------------------------------------------
// Will write a value to a given register of the GY-271/HMC5883L
// magnetic field sensor.
// -----------------------------------------------------------------
void I2C_GY271_Write (byte Register, byte Value) 
{
  byte bStatus;
  if (bStatus=nI2C->Write (I2C_HANDLE_GY271, Register, &Value, (uint32_t)1)!=0) 
  { // Something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C, bStatus, 26); // XXX Tell the host computer that there is an issue
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
void I2C_GY271_Read_Values (tGY271_Values *pGY271, unsigned int ErrorSource) 
{
  byte bStatus;
  I2C_GY271_Value_Pointer = pGY271; // The ISR function ("I2C_GY271_OnDataReceived") needs to know where to put the received values. So copy the pointer to this global variable.
  pGY271->Valid=0;                  // Set the valid variable in this bufferplace to false, so that ardurino wont use these values for any calculation.
  if (bStatus=nI2C->Write (I2C_HANDLE_GY271, &cZero[0], 1)!=0)  // Tell this GY271 sensor we want to read beginning of register zero (there are the measured values stored)
  { // There is an issue with nI2C, send error message to host and stop the arduino
    RS232_SendError (RS232_FRAMEID_ERROR_I2C,bStatus,ErrorSource); // Inform the host computer that there is an error with I2C
    while (1) { }; // STOP Arduino
  }
  // Tell the "nI2C" library that the next data, which is received over the I2C bus, comes from this GY271 sensor:
  if (bStatus=nI2C->Read (I2C_HANDLE_GY271, &I2C_GY271_ReceiveBuffer[0], (uint32_t)6, I2C_GY271_OnDataReceived)!=0) // Tell the library where to store the received values and which ISR to call when data is received
  { // There is an issue with this library:
    RS232_SendError (RS232_FRAMEID_ERROR_I2C,bStatus,ErrorSource); // Inform the host computer that there is an error
    while (1) { }; // STOP Arduino    
  }
}

// -----------------------------------------------------------------
// I2C_VL6180X_Read_Values
// -----------------------------------------------------------------
// This function is asking one VL6180X sensor for its values.
// The parameter "pVL6180X" contains a pointer to one of the
// bufferplaces for each VL6180X. Doesnt matter if it is footplate
// left or right.
// Before this function is called, it is important to use
// the function "I2C_Select_VL6180X" before.
// -----------------------------------------------------------------
void I2C_VL6180X_Read_Values (tVL6180X_Values *pVL6180X, unsigned int ErrorSource) 
{
  // Needs to be implemented
}

// -----------------------------------------------------------------
// setup
// -----------------------------------------------------------------
// Will initialize all components.
// -----------------------------------------------------------------
void setup() 
{
  RS232_setup ();
  CAN_setup ();
  I2C_setup ();
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
  byte ErrorSource;
  if (bStatus!=0) // is there an issue with the received data from the current GY271 sensor?
  { // something went wrong
    RS232_SendError (RS232_FRAMEID_ERROR_I2C,bStatus,ErrorSource); // XXX ErrorSource is here unknown! Inform the host computer that there is an error
    while (1) { }; // Stop the arduino
  }
  // Looks like to be fine:
  // Copy the received value from the global buffer to local variables (I know this is stupid):  
  x = (int)(int16_t)(I2C_GY271_ReceiveBuffer[0] | I2C_GY271_ReceiveBuffer[1] << 8);
  y = (int)(int16_t)(I2C_GY271_ReceiveBuffer[2] | I2C_GY271_ReceiveBuffer[3] << 8);
  z = (int)(int16_t)(I2C_GY271_ReceiveBuffer[4] | I2C_GY271_ReceiveBuffer[5] << 8);
  s = I2C_GY271_ReceiveBuffer[6];
  // Now, copy these values from the local variables to the buffer of the current GY271 sensor:
  I2C_GY271_Value_Pointer->X = x;
  I2C_GY271_Value_Pointer->Y = y;
  I2C_GY271_Value_Pointer->Z = z;
/* XXX The programmer has to think about this one:
 *  a) it is important?
 *  b) do we really need this one?
 *  c) what should we do in such a case?
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
  I2C_GY271_Value_Pointer->Valid=1; // Tell the arduino, that we now can calculate with these values, because they are valid
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
  if(Serial.available() >= sizeof(RS232Message))
  { // Data received from the host computer
    if (Serial.readBytes(&message.id,sizeof(message))==sizeof(RS232Message))
    { // Message is read
      
    }
    else
    { // Something went wrong
      RS232_SendError (RS232_FRAMEID_ERROR_RS232, 1,0); // tell the host computer that there is something wrong. Should never be execute.
    }
  }
}

// -----------------------------------------------------------------
// CAN_loop
// -----------------------------------------------------------------
// Polling the CAN bus, checking if ODRIVE has something send to
// this arduino. This function is also being used to ask the ODRIVE
// some stuff. This is an endless loop.
// -----------------------------------------------------------------
void CAN_loop ()
{
  CANMessage message;
  // Check if data is received from ODRIVE over the CAN-bus:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  can.poll () ;         // Because we do not use interrupt for the can-bus, we have to poll the can bus.
  if (can.available ()) 
  { // Something was received from the CAN-bus:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    can.receive (message);
    switch (message.id)
    {
      case  ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Heartbeat_Message:  // This one is send by ODRIVE by its own, 10 times a second
               // Message from AXIS0 with its current state and axis error:
               ODrive.Axis0.AxisError = message.data32[0];
               ODrive.Axis0.AxisCurrentState = message.data32[1];
               break;
      case  ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Heartbeat_Message:  // This one is send by ODRIVE by its own, 10 times a second
               // Message from AXIS1 with its current state and axis error:
               ODrive.Axis1.AxisError = message.data32[0];
               ODrive.Axis1.AxisCurrentState = message.data32[1];
               break;
      case ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_Encoder_Count: // We will receive this one only if we have asked for
               // Message from AXIS0 with its current encoder value:
               ODrive.Axis0.EncoderShadowCount = message.data32[0];
               ODrive.Axis0.EncoderCountinCPR  = message.data32[1];
               break;
      case ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_Encoder_Count:  // We will receive this one only if we have asked for
               // Message from AXIS1 with its current encoder value:
               ODrive.Axis1.EncoderShadowCount = message.data32[0];
               ODrive.Axis1.EncoderCountinCPR  = message.data32[1];
               break;
      case ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_VBus_Voltage:  // We will receive this one only if we have asked for
      case ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_VBus_Voltage:
               // Message from ODRIVE with its current power supply voltage:
               ODrive.VBus_Voltage = message.dataFloat[0];
               break;              
    }
    // Here comes the send/polling part to ask ODRIVE something:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    switch (CanBus.bPollIndex)  // We ask the ODRIVE each available parameter in a row, we ask all the time the same stuff:
    {
      case 0: // Ask for the current encoder position of AXIS0 (rotation axis)
              message.id = ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_Encoder_Count;
              message.rtr = true;
              message.len = 0;
              if (can.tryToSend (message)) CanBus.bPollIndex++; // Next stuff to ask for, in the next cycle
              break;
      case 1: // Ask for the current encoder position of AXIS1 (horizontal axis)
              message.id = ODRIVE_NODE_ID_AXIS1 + CAN_FRAMEID_Get_Encoder_Count;
              message.rtr = true;
              message.len = 0;
              if (can.tryToSend (message)) CanBus.bPollIndex++; // Next stuff to ask for, in the next cycle
              break;
      case 2: // Ask for the current voltage of the power supply from the ODRIVE
              message.id = ODRIVE_NODE_ID_AXIS0 + CAN_FRAMEID_Get_VBus_Voltage;
              message.rtr = true;
              message.len = 0;
              if (can.tryToSend (message)) CanBus.bPollIndex=0; // repeat stuff to ask for, in the next cycle
              break;
      default: // Ups, something went wrong, but we don't care. We just repeat the loop from the beginning (no need to raise an emergency stop)
              CanBus.bPollIndex=0;
              break;
    }
  }
}

// -----------------------------------------------------------------
// I2C_loop
// -----------------------------------------------------------------
// Call this function very often. It will communicate with each
// footplate and with each of its sensors.
// Once the data is collected from all sensors of all footplates,
// it will begin to calculate where all the axis have to move to.
// -----------------------------------------------------------------
// Keep in mind: each footplate has got 12 sensors and there are
// two TCA9548A I2C multiplexer for each footplate. Each TCA9548A
// has got 8 I2C channels, but only six of them are being used:
// Channel: Sensor
// 0:   GY-271/HMC5883L
// 1:   VL6180X
// 2:   VL6180X
// 3:   GY-271/HMC5883L
// 4:   GY-271/HMC5883L
// 5:   VL6180X
void I2C_loop ()
{
  static byte a=0;
  switch (a)
  {
    case 0: // Activate Channel 0 and 1 of the upper TCA9548A at left footplate at once:
      I2C_Select_TCA9548A (Footplate_Left.I2C_HANDLE_TCA9548A_Upper, 3); 
      a++;
      break;
    case 1: // Read the measure value of both sensors:
      I2C_GY271_Read_Values (&Footplate_Left.GY271[0],1); // ErrorSource is 1, just in case there is an issue later one
      I2C_VL6180X_Read_Values (&Footplate_Left.VL6180X[0],7); // // ErrorSource is 7, just in case there is an issue later one
      a++;
      break;
    case 2: // Wait until the value have been received:
      if ((Footplate_Left.GY271[0].Valid==true)&& // Yes, all values from both sensors are received, continue:
          (Footplate_Left.VL6180X[0].Valid==true))
      {
        a++;
      }
      break;
    case 3: // Switch to the next channel
      a=0; // XXX
      break;
  }
}


void loop() 
{
  CAN_loop ();
  RS232_loop ();
  I2C_loop ();
}
