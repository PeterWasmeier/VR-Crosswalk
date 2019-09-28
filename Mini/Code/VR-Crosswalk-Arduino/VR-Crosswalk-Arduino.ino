#include <MobaTools.h>
#include <avrlibdefs.h>
#include <nI2C.h>
#include <nTWI.h>
#include <queue.h>
                                                       
/* ******************************************************************************************************** */
/* **************************************  PIN ASSIGNMENT ************************************************* */
/* ******************************************************************************************************** */
//         Name                       = PIN     PROPERTY DESCRIPTION
const char PIN_BASEPLATE_ENCODER_B    =  2;  // INT      Baseplate, Encoder, Channel B
const char PIN_BASEPLATE_ENCODER_A    =  3;  // PWM,INT  Baseplate, Encoder, Channel A
const char PIN_STEPPER_COIL2_PLUS     =  4;  //          Stepperdriver for wire pair: green/black (IN2). The stepper is always enabled.
const char PIN_BASEPLATE_MOTOR_PLUS   =  7;  //          Baseplate, Motor, Direction Plus/Clockwise
const char PIN_BASEPLATE_MOTOR_MINUS  =  8;  //          Baseplate, Motor, Direction Minus/Counter clockwise
const char PIN_FOOTPRINT_LEFT_SERVO   =  9;  // PWM      Footplate, Servo, Left, Position, 1ms..2ms=-90°..+90°
const char PIN_FOOTPRINT_RIGHT_SERVO  = 10;  // PWM      Footplate, Servo, Right, Position, 1ms..2ms=-90°..+90°
const char PIN_BASEPLATE_MOTOR_ENABLE = 11;  // PWM      Baseplate, Motor, Enable/PWM Signal
const char PIN_STEPPER_COIL1_PLUS     = 12;  //          Stepperdriver for wire pair: red/blue (IN4). The stepper is always enabled.
const char PIN_STEPPER_COIL1_MINUS    = 13;  //          Stepperdriver for wire pair: red/blue (IN3). The stepper is always enabled.
const char PIN_RESERVE1               = A0;  // A0      
const char PIN_STEPPER_COIL2_MINUS    = A1;  // A1       Stepperdriver for wire pair: green/black (IN1). The stepper is always enabled.

/* ******************************************************************************************************** */
/* **************************************  GLOBAL STRUCTURES  ********************************************* */
/* ******************************************************************************************************** */

typedef struct {                    // tGY271_XYZ, contains the measured values for each axis
  int X;                            // Current X value, measured by the GY271 sensor
  int Y;                            // Current Y value, measured by the GY271 sensor
  int Z;                            // Current Z value, measured by the GY271 sensor
} tGY271_XYZ;

typedef struct {                    // tGY271, some values for each GY271 sensor
  bool Valid;                       // Value in "Value" is valid.
  tGY271_XYZ Value;                 // Current Sensorvalue from GY271
} tGY271;

typedef struct {                    // tPosition, contains X and Y coordinates in mm.
  int X;                         // X Position in mm of something
  int Y;                         // Y Position in mm of something
} tPositionInt;

typedef struct {                    // tPosition, contains X and Y coordinates in mm.
  double X;                         // X Position in mm of something
  double Y;                         // Y Position in mm of something
} tPositionDouble;

typedef struct {                    // tFootplate, stuff for each footplate
  tGY271 GY271[6];                  // Each footplate has got 6 GY271 sensors
  tPositionInt Current;             // The current center of the footplate in mm
  tPositionInt Destination;         // The destination, where the footplate center has to move to, Units in absolute mm
  tPositionInt SensorOffset;        // Where is the sensor pointing to? It is an offset position in mm, including the current rotation of the footplate
  tPositionInt Offset;              // Offset of "SensorOffset"
  int Alpha;                        // Orientation of the footplate relative to the shoe, this is not the real orientation, 90 = The plate orientation is under the shoe. The orientation of footplate itself is stored in ""
  bool SensorOffsetValid;           // The value in SensorOffset is valid or not   
  CI2C::Handle I2C_TCA9548A_Upper;  // I2C Handle to the upper TCA9548A chip of this footplate
  CI2C::Handle I2C_TCA9548A_Lower;  // I2C Handle to the lower TCA9548A chip of this footplate
  double dServoPosition;
  int iServoPosition;
} tFootplate;

typedef struct {                    // tMotor, a lot of stuff for the DC motor
    bool bTurnedOn;                 // Motor is currently turned on    
    bool bEncoderPreviousChannelA;  // Previously value of the channel A of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderPreviousChannelB;  // Previously value of the channel B of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderInverted;          // TRUE, in case the channel A and B is inverted. Use the function "fMotor_loopCheckEncoderInverted" to figure out
    signed char bCurrentDirection;
    volatile bool bExecuteIsActive; // The function "fMotor_Execute" is already active/running
    volatile int iEncoderPosition; // Current position of the motor in units.
    double dP;                       // Position control, the Proportional value
    double dI;                       // Position control, the Integral value
    double dD;                       // Position control, the 
    int iLastError;                // Position control, the previous error distance
    unsigned long ulSampleLastTime; // Position control, time in microseconds when the function "fMotor_compute" was executed last time
    double dIntegral;                 // Position control, the sum of the current integral
    signed char bLastPIDValue;             // The current/previously calculated PID value from the function "fMotor_compute"
    signed char bLastPWMValue;             // The current/previously calculated PWM value from the function "fMotor_setPWM"
    signed char bLastDirection;     // Current or last direction of the motor. 0=unknown, -1=left, 1=right
    int iTargetPosition;           // Where the motor has to move to. This value is not in mm, it is in units of "lEncoderPosition"
} tMotor;

typedef struct {
  int iPreviousDestinationX;
  int iPreviousDestinationY;
  int iDestinationX;
  int iDestinationY;  
} tCNC;

typedef struct {
  signed char bMotor_LastPIDValue;
  int iMotor_TargetPosition;
  int iMotor_EncoderPosition;
  int iSteppermotor_CurrentPosition;
  int iSteppermotor_TargetPosition;
  int iServoLeftPosition;
  int iServoRightPosition;
  int iFootprintRight_Alpha;
} tLogging;

typedef struct {
  char cCommand[13];
  char cValue[5];
  byte bValueLength;
  byte bCommandLength;
  bool bValueActive;  
} tInterface;

/* ******************************************************************************************************** */
/* **************************************  GLOBAL VARIABLES  ********************************************** */
/* ******************************************************************************************************** */
bool bRequestAll;                   // PC/MFC wants to read all values
byte    cGY271_ReceiveBuffer[10];   // All bytes, received over I2C from the GY271 sensors, will be placed into this buffer
tGY271 *pGY271_Callback_Sensor;     // Pointer to a single "tFootplate.tGY271[x]" sensor, which is being readed right now over I2C, needed for the I2C callback function "fGY271_Callback"
CI2C::Handle I2C_GY271;             // Because al GY271 sensors have the same I2C address, we only need one single I2C handler for all of them
tFootplate FootprintLeft;           // The left footplate
tFootplate FootprintRight;          // The right footplate
byte bSTOP_CurrentTCA9548A;         // In case of an I2C error, we need to remember which TCA9548A was previously used on the I2C bus
byte bSTOP_CurrentTCA9548A_Port;    // In case of an I2C error, we need to remember which port of the current TCA9548A was previously used on the I2C bus
//Servo8 sServoLeft;                   // Servo of the left footprint
Servo8 sServoRight;                  // Servo of the right footprint
Stepper4 sStepper(400,HALFSTEP);
tMotor Motor;                       // The motor of the baseplate which will rotate the baseplate
tCNC CNC;
tLogging Logging;
tInterface Interface;
bool bGO_R_Command_Active;
bool bGO_R_Command_Once;
bool bGO_R_Command_Always;
int Steppermotor_iTargetPosition;
/* ******************************************************************************************************** */
/* **************************************  GLOBAL CONSTANTS  ********************************************** */
/* ******************************************************************************************************** */

const byte   cTCA9548A_SendBuffer[9]={0,1,2,4,8,16,32,64,128};  // We need a global variable to write to the I2C bus, this array is being used for TCA9548A
const byte   cZero[1]={0};               // We need a global variable to write to the I2C bus, this array is being used for GY271 to read X,Y,Z values
const byte   MOTOR_HIGH_SPEED = 90;   // The baseplate motor is a 6V motor. The power supply is 12V, so in order to get 6V the maximum value is 144, but we want the maximum motorspeed to be 90 PWM signal
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) }; // X Position in mm of each single GY271 sensor on the footplate
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 }; // Y Position in mm of each single GY271 sensor on the footplate
const double ENCODER_PULSES_PER_90DEGREE = 600.0;

const int SERVO_RIGHT_PLUS45  = 2100;
const int SERVO_RIGHT_MIDDLE  = 1566;
const int SERVO_RIGHT_MINUS45 = 1011;

const int SERVO_LEFT_PLUS45   = 2003;
const int SERVO_LEFT_MIDDLE   = 1613;
const int SERVO_LEFT_MINUS45  = 1208;

/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */
void DC_PIDMOTOR_EncoderInterrupt () {
  fMotor_EncoderInterrupt ();
}

void fFootprint_Init () {
  // Servos vorbereiten:
//  sServoLeft.attach (PIN_FOOTPRINT_LEFT_SERVO);
  sServoRight.attach (PIN_FOOTPRINT_RIGHT_SERVO);

//  sServoLeft.setSpeed(1,true);
  sServoRight.setSpeed(0);

  // Right footprint GY271 init:
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 0);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 3);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 4);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 0);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 3);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 4);
  FootprintRight.Offset.X=0;
  FootprintRight.Offset.Y=0;
  sServoRight.write (SERVO_RIGHT_MIDDLE);
//  sServoLeft.write (SERVO_LEFT_MIDDLE);
  FootprintRight.Destination.X=30;
  FootprintRight.Destination.Y=0;
  
}
