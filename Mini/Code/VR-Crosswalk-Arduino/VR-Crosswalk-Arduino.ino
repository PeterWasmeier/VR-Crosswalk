#include <Servo.h>
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
  /*bool SetOffset;                   // True in case the current sensor value must be used as an offset*/
  tGY271_XYZ Value;                 // Current Sensorvalue from GY271
  //tGY271_XYZ Offset;                // Offset value for each single GY271 sensor
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
  tPositionInt SensorOffset;        // The offset in mm of the shoe above the footplate
  tPositionInt Offset;              // Offset of "SensorOffset"
  volatile int Alpha;               // Orientation of the footplate relative to the shoe, this is not the real orientation, 90 = The plate orientation is under the shoe. The orientation of footplate itself is stored in ""
  bool SensorOffsetValid;           // The value in SensorOffset is valid or not   
  CI2C::Handle I2C_TCA9548A_Upper;  // I2C Handle to the upper TCA9548A chip of this footplate
  CI2C::Handle I2C_TCA9548A_Lower;  // I2C Handle to the lower TCA9548A chip of this footplate
  volatile int iServoPosition;
  volatile double dServoPosition;
} tFootplate;

typedef struct {                    // tSteppermotor_Steps, for each coil the steps in order to turn left/right
  bool bPIN_COIL1_PLUS;             // First coil, positive direction
  bool bPIN_COIL2_PLUS;             // First coil, negative direction
  bool bPIN_COIL1_MINUS;            // Second coil, positive direction
  bool bPIN_COIL2_MINUS;            // Second coil, negative direction
} tSteppermotor_Steps;

typedef struct {                    // tSteppermotor, some stuff for the stepper motor
  volatile int iCurrentPosition;   // Current position of the stepper motor in steps
  volatile int iTargetPosition;    // Destination position (in steps) where the motor has to move to
  bool bTurnedOn;                   // Stepper is currently turned on
  bool bAutoTurnOff;                // Turn off the stepper motor after 50ms when destination position is reached
  byte bStepIndex;                  // Index to the array "bSteppermotor_Steps[]", so keep in mind which coil is or was powered
  byte Acceleration;                // Unit is mm/sec
  unsigned long ulStart;            // Timestamp of the last step which was done. Needed to calculate the time until the next step has to be executed
  signed char bLastDirection;       // Current or last direction of the stepper motor. 0=unknown, -1=left, 1=right
  unsigned long ulRunningSince;             // The stepper is running in one direction since these seconds
} tSteppermotor;

typedef struct {                    // tMotor, a lot of stuff for the DC motor
    bool bTurnedOn;                 // Motor is currently turned on    
    bool bEncoderPreviousChannelA;  // Previously value of the channel A of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderPreviousChannelB;  // Previously value of the channel B of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderInverted;          // TRUE, in case the channel A and B is inverted. Use the function "fMotor_loopCheckEncoderInverted" to figure out
    volatile bool bInterruptEnabled;// The function "fMotor_Execute" is allowed to be executed over the interrupts of the encoder
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
//tFootplate FootprintLeft;           // The left footplate
tFootplate FootprintRight;          // The right footplate
byte bSTOP_CurrentTCA9548A;         // In case of an I2C error, we need to remember which TCA9548A was previously used on the I2C bus
byte bSTOP_CurrentTCA9548A_Port;    // In case of an I2C error, we need to remember which port of the current TCA9548A was previously used on the I2C bus
Servo sServoLeft;                   // Servo of the left footprint
Servo sServoRight;                  // Servo of the right footprint
tSteppermotor Steppermotor;         // The steppermotor of the baseplate which will move the footplates left/right
tMotor Motor;                       // The motor of the baseplate which will rotate the baseplate
tCNC CNC;
tLogging Logging;
tInterface Interface;
int iServoLeftPosition;
bool bGO_R_Command_Active;
bool bGO_R_Command_Once;
bool bGO_R_Command_Always;
/* ******************************************************************************************************** */
/* **************************************  GLOBAL CONSTANTS  ********************************************** */
/* ******************************************************************************************************** */

const byte    cTCA9548A_SendBuffer[9]={0,1,2,4,8,16,32,64,128};  // We need a global variable to write to the I2C bus, this array is being used for TCA9548A
const byte    cZero[1]={0};               // We need a global variable to write to the I2C bus, this array is being used for GY271 to read X,Y,Z values
const byte MOTOR_HIGH_SPEED = 90;   // The baseplate motor is a 6V motor. The power supply is 12V, so in order to get 6V the maximum value is 144, but we want the maximum motorspeed to be 90 PWM signal
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) }; // X Position in mm of each single GY271 sensor on the footplate
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 }; // Y Position in mm of each single GY271 sensor on the footplate
const tSteppermotor_Steps bSteppermotor_Steps[] = { { LOW, LOW, LOW, HIGH },
                                                    { LOW, LOW, HIGH, LOW },
                                                    { LOW, HIGH, LOW, LOW },
                                                    { HIGH, LOW, LOW, LOW } };
const double ENCODER_PULSES_PER_90DEGREE = 600.0;
/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */


// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect)  {  
  double dPrimaryCurrentPosition;
  fSteppermotor_Execute ();
  // Change the orientation of the footplate in a way, that the plate is below the shoe:
  //dPrimaryCurrentPosition=Motor.iEncoderPosition;
  //FootprintRight.dServoPosition = 1500 + (  ((90.0/ENCODER_PULSES_PER_90DEGREE) * dPrimaryCurrentPosition) * (600.0/45.0) ); // The servo will be -600=-45°, +600=+45° 
  if (bGO_R_Command_Always)
  {
  FootprintRight.dServoPosition = FootprintRight.dServoPosition + (FootprintRight.Alpha-90) * 0.1;  // Proportional Controller to go to 90°
  if (FootprintRight.dServoPosition>2100) FootprintRight.dServoPosition=2100;
  if (FootprintRight.dServoPosition<900) FootprintRight.dServoPosition=900;  
  FootprintRight.iServoPosition = FootprintRight.dServoPosition;
  sServoRight.writeMicroseconds (FootprintRight.iServoPosition);
  }
}

void DC_PIDMOTOR_EncoderInterrupt () {
  fMotor_EncoderInterrupt ();
}


void fFootprint_Init () {
  // Servos vorbereiten:
  sServoLeft.attach (PIN_FOOTPRINT_LEFT_SERVO);
  sServoLeft.writeMicroseconds (1500);
  sServoRight.attach (PIN_FOOTPRINT_RIGHT_SERVO);
  sServoRight.writeMicroseconds (1500+81);

  // VL6180X Sensoren initialisieren:
  // Left footprint VL6180X init:  
  /*fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Upper,1);
  fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Upper,2);
  fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Upper,5);
  fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Lower,1);
  fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Lower,2);
  fVL6180X_Init (FootprintLeft.I2C_TCA9548A_Lower,5);
  */
  // Right footprint GY271 init:
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 0);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 3);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Upper, 4);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 0);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 3);
  fGY271_Init (FootprintRight.I2C_TCA9548A_Lower, 4);
  FootprintRight.Offset.X=0;
  FootprintRight.Offset.Y=0;
  FootprintRight.iServoPosition=1500;
  FootprintRight.dServoPosition=1500;
  FootprintRight.Destination.X=30;
  FootprintRight.Destination.Y=0;
}
