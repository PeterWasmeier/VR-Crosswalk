// VR-Crosswalk Light
// ~~~~~~~~~~~~~~~~~~
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
// There are 24 I2C sensors and 4 TCA9548A I2C multiplexer connected to the I2C bus.
// So, in total there are 28 I2C devices on the I2C bus. The second parameter
// tell the host computer which I2C device has an issue.
// Each TCA9548A uses six channels. The sensor type, connected to a channel, is like this:
// Channel Sensor
// 0:      GY-271/HMC5883L
// 1:      VL6180X
// 2:      VL6180X
// 3:      GY-271/HMC5883L
// 4:      GY-271/HMC5883L
// 5:      VL6180X
// This second parameter is devided into upper and lower 8 bits:

const byte RS232_FRAMEID_ERROR_CODE = 4; // Message to host computer, that there is something wrong in the source code of this program.
// 1.Parameter (Errornumber):
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
//  1: Invalid parameter for function I2C_Select_TCA9548A, ChannelToActivate is too high. Stopping the arduino
//  2: The parameter Footplate for the function I2C_Select_GY271 is invalid. Arduino stopped.
//  3: The parameter device_address in function I2C_GetErrorSource is invalid.
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
} RS232Message;

// global variable stuff for SPI/CAN bus communication with ODRIVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ACAN2515 can (10, SPI, 255) ;     // For SPI communication: Chip select pin = 10, Last argument is 255 -> no interrupt pin is being used for can-bus communication

// Global variables for/from ODRIVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Each axis of ODRIVE has got its own node id.
// By default, Axis0 has got the node id zero.
// The other axis, Axis1, has got the node id one.
const unsigned int ODRIVE_NODE_ID_AXIS0 =  0;      // Upper 6 bit of the can-bus-frame-id is the NODE id of axis0
const unsigned int ODRIVE_NODE_ID_AXIS1 = (1) << 5; // Upper 6 bit of the can-bus-frame-id is the NODE id of axis1
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
  byte Distance;            // Current Distance value in mm
  byte SignalStength;       // Current SignalStrength
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

CI2C::Handle I2C_HANDLE_Current_TCA9548A;  // This variable contains the handle of the current active TCA9548A I2C multiplexer. There can be only one of them being active at the same time.
byte I2C_CHANNEL_Current_TCA9548A; // The channels which are currently activated/open on the current TCA9548A

// Because we use the library "nI2C", each I2C device (like GY271 or VL6180X) gets its own handle, which is later on needed to communicate with
CI2C::Handle I2C_HANDLE_GY271;   // Each footplate has got six GY271 magnet field sensors. But we have to create only instance, because they are all separated by I2C multiplexer
CI2C::Handle I2C_HANDLE_VL6180X; // Each footplate has got six VL6180X sensor sensors. But we have to create only instance, because they are all separated by I2C multiplexer

// Because the data is transfered in the background (using interrupts), we have to offer a global buffer for the ISR, where the ISR can store the received values:
typedef struct
{
  union {
    uint16_t data16[3];
    uint8_t data [6] = {0, 0, 0, 0, 0, 0};
  };
} tReceiveBuffer;
tReceiveBuffer I2C_GY271_ReceiveBuffer; // All bytes, received over I2C from the GY271 sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
tReceiveBuffer I2C_VL6180X_ReceiveBuffer; // XXX All bytes, received over I2C from the VL6180X sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
tGY271_Values *I2C_GY271_Value_Pointer;  // This is a pointer to the currently active GY271 sensor. When data is received from this sensor, the function "I2C_GY271_OnDataReceived" will fill this structure with the values from the sensor.
tVL6180X_Values *I2C_VL6180X_Value_Pointer;  // This is a pointer to the currently active VL6180X sensor. When data is received from this sensor, the function "I2C_VL6180X_OnDataReceived" will fill this structure with the values from the sensor.
byte cValue[256];
bool VL6180X_IsInitialized;

const byte TCA9548A_CHANNEL_0 = 1;
const byte TCA9548A_CHANNEL_1 = 2;
const byte TCA9548A_CHANNEL_2 = 4;
const byte TCA9548A_CHANNEL_3 = 8;
const byte TCA9548A_CHANNEL_4 = 16;
const byte TCA9548A_CHANNEL_5 = 32;

// -----------------------------------------------------------------
// setup
// -----------------------------------------------------------------
// Will initialize all components.
// -----------------------------------------------------------------
void setup()
{
  int i;
  for (i=0;i<256;i++)
  {
    cValue[i]=i;
  }
  RS232_setup ();
  CAN_setup ();
  I2C_setup ();
}


// -----------------------------------------------------------------
// loop
// -----------------------------------------------------------------
// The main program, looping all the time.
// -----------------------------------------------------------------
void loop()
{
  CAN_loop ();
  RS232_loop ();
  I2C_loop ();
}
