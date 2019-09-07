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
  bool SetOffset;                   // True in case the current sensor value must be used as an offset
  tGY271_XYZ Value;                 // Current Sensorvalue from GY271
  tGY271_XYZ Offset;                // Offset value for each single GY271 sensor
} tGY271;

typedef struct {                    // tPosition, contains X and Y coordinates in mm.
  double X;                         // X Position in mm of something
  double Y;                         // Y Position in mm of something
} tPosition;

typedef struct {                    // tFootplate, stuff for each footplate
  tGY271 GY271[6];                  // Each footplate has got 6 GY271 sensors
  tPosition Current;                // The current center of the footplate in mm
  tPosition Destination;            // The destination, where the footplate has to move to in mm
  tPosition SensorOffset;           // The offset of the foot above the footplate in mm
  bool SensorOffsetValid;           // The value in SensorOffset is valid or not   
  CI2C::Handle I2C_TCA9548A_Upper;  // I2C Handle to the upper TCA9548A chip of this footplate
  CI2C::Handle I2C_TCA9548A_Lower;  // I2C Handle to the lower TCA9548A chip of this footplate
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
  unsigned int uiAcceleration;      // Acceleration in mm per seconds²
  unsigned long ulStart;            // Timestamp of the last step which was done. Needed to calculate the time until the next step has to be executed
  signed char bLastDirection;       // Current or last direction of the stepper motor. 0=unknown, -1=left, 1=right
} tSteppermotor;

typedef struct {                    // tMotor, a lot of stuff for the DC motor
    bool bTurnedOn;                 // Motor is currently turned on    
    bool bEncoderPreviousChannelA;  // Previously value of the channel A of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderPreviousChannelB;  // Previously value of the channel B of the encoder. Needed to figure out the current spinning direction of the motor.
    bool bEncoderInverted;          // TRUE, in case the channel A and B is inverted. Use the function "fMotor_loopCheckEncoderInverted" to figure out
    volatile bool bInterruptEnabled;// The function "fMotor_Execute" is allowed to be executed over the interrupts of the encoder
    volatile bool bExecuteIsActive; // The function "fMotor_Execute" is already active/running
    volatile int iEncoderPosition; // Current position of the motor in units.
    float fP;                       // Position control, the Proportional value
    float fI;                       // Position control, the Integral value
    float fD;                       // Position control, the 
    int iLastError;                // Position control, the previous error distance
    unsigned long ulSampleLastTime; // Position control, time in microseconds when the function "fMotor_compute" was executed last time
    long lIntegral;                 // Position control, the sum of the current integral
    long lLastPIDValue;             // The current/previously calculated PID value from the function "fMotor_compute"
    byte bLastPWMValue;             // The current/previously calculated PWM value from the function "fMotor_setPWM"
    signed char bLastDirection;     // Current or last direction of the motor. 0=unknown, -1=left, 1=right
    int iTargetPosition;           // Where the motor has to move to. This value is not in mm, it is in units of "lEncoderPosition"
} tMotor;

typedef struct {
  double dPreviousX;
  double dPreviousY;
  double dDestinationX;
  double dDestinationY;  
} tCNC;

typedef struct {
  byte bMotor_LastPWMValue;
  int iMotor_TargetPosition;
  int iMotor_EncoderPosition;
  int iSteppermotor_CurrentPosition;
  int iSteppermotor_TargetPosition;
  tPosition FootplateRight_SensorOffset;
} tLogging;

typedef struct {
  char cCommand[10];
  char cValue[10];
  byte bValueLength;
  byte bCommandLength;
  bool bValueActive;  
} tInterface;

/* ******************************************************************************************************** */
/* **************************************  GLOBAL VARIABLES  ********************************************** */
/* ******************************************************************************************************** */

byte    cGY271_ReceiveBuffer[10];   // All bytes, received over I2C from the GY271 sensors, will be placed into this buffer
tGY271 *pGY271_Callback_Sensor;     // Pointer to a single "tFootplate.tGY271[x]" sensor, which is being readed right now over I2C, needed for the I2C callback function "fGY271_Callback"
CI2C::Handle I2C_GY271;             // Because al GY271 sensors have the same I2C address, we only need one single I2C handler for all of them
tFootplate FootprintLeft;           // The left footplate
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

/* ******************************************************************************************************** */
/* **************************************  GLOBAL CONSTANTS  ********************************************** */
/* ******************************************************************************************************** */

const byte    cTCA9548A_SendBuffer[9]={0,1,2,4,8,16,32,64,128};  // We need a global variable to write to the I2C bus, this array is being used for TCA9548A
const byte    cZero[1]={0};               // We need a global variable to write to the I2C bus, this array is being used for GY271 to read X,Y,Z values
const byte MOTOR_HIGH_SPEED = 90;   // The baseplate motor is a 6V motor. The power supply is 12V, so in order to get 6V the maximum value is 144, but we want the maximum motorspeed to be 90 PWM signal
const byte STEPPER_ACCELERATION = 10; // 10 Units/second
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) }; // X Position in mm of each single GY271 sensor on the footplate
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 }; // Y Position in mm of each single GY271 sensor on the footplate
const tSteppermotor_Steps bSteppermotor_Steps[] = { { HIGH, LOW, LOW, LOW },
                                                    { LOW, HIGH, LOW, LOW },
                                                    { LOW, LOW, HIGH, LOW },
                                                    { LOW, LOW, LOW, HIGH } };

/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */


// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect)  {  
  fSteppermotor_Execute ();
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
}
