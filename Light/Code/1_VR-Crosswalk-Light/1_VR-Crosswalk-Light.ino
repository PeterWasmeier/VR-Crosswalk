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
  unsigned int ErrorCode;
  unsigned int SignalRate;       // Current SignalStrength
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

CI2C::Handle *I2C_HANDLE_Current_TCA9548A;  // This variable contains the handle of the current active TCA9548A I2C multiplexer. There can be only one of them being active at the same time.
byte I2C_CHANNEL_Current_TCA9548A; // The channels which are currently activated/open on the current TCA9548A

// Because we use the library "nI2C", each I2C device (like GY271 or VL6180X) gets its own handle, which is later on needed to communicate with
CI2C::Handle I2C_HANDLE_GY271;   // Each footplate has got six GY271 magnet field sensors. But we have to create only instance, because they are all separated by I2C multiplexer
CI2C::Handle I2C_HANDLE_VL6180X; // Each footplate has got six VL6180X sensor sensors. But we have to create only instance, because they are all separated by I2C multiplexer

// Because the data is transfered in the background (using interrupts), we have to offer a global buffer for the ISR, where the ISR can store the received values:
byte I2C_GY271_ReceiveBuffer[10]; // All bytes, received over I2C from the GY271 sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
byte I2C_VL6180X_ReceiveBuffer[10]; // XXX All bytes, received over I2C from the VL6180X sensors, will be placed into this buffer by the function "I2C_GY271_OnDataReceived"
tGY271_Values *I2C_GY271_Value_Pointer;  // This is a pointer to the currently active GY271 sensor. When data is received from this sensor, the function "I2C_GY271_OnDataReceived" will fill this structure with the values from the sensor.
tVL6180X_Values *I2C_VL6180X_Value_Pointer;  // This is a pointer to the currently active VL6180X sensor. When data is received from this sensor, the function "I2C_VL6180X_OnDataReceived" will fill this structure with the values from the sensor.
const byte cValue[256] ={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
                        0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
                        0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
                        0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
                        0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
                        0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,
                        0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
                        0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,
                        0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,
                        0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F,
                        0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,
                        0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,
                        0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,
                        0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,
                        0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,
                        0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF};

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
  RS232_setup ();
  //CAN_setup ();
  I2C_setup ();
  Serial.println ("Setup done");
}

// -----------------------------------------------------------------
// loop
// -----------------------------------------------------------------
// The main program, looping all the time.
// -----------------------------------------------------------------
void loop()
{
  //CAN_loop ();
  RS232_loop ();
  I2C_loop ();
}
