#include <Servo.h>
#include <avrlibdefs.h>
#include <nI2C.h>
#include <nTWI.h>
#include <queue.h>

double dFootX=30.0;
double dFootY=0;

const byte MOTOR_HIGH_SPEED           = 90;            // Der Motor darf maximal 6V erhalten
                                                       // Es liegen am Motortreiber "L298N" 12V extern an. Er besitzt 1,4V Verlust
                                                       // D.h. ein 
                                                       // PWM mit   0 = 0V
                                                       // PWM mit 144 = 5,986V
                                                       // PWM mit 255 = 10,6V (12V - 1,4 Verlust)
const char PIN_BASEPLATE_ENCODER_B    = 2;   // INT     Plattform Encoder: INT Input
const char PIN_BASEPLATE_ENCODER_A    = 3;   // PWM,INT Plattform Encoder: INT Input
const char PIN_STEPPER_COIL2_PLUS     = 4;   //         Stepper Driver for wire pair: Green/Black (IN2), The stepper is always enabled
const char PIN_BASEPLATE_MOTOR_PLUS   = 7;   //         Plattform Motor: Richtung Plus/Clockwise
const char PIN_BASEPLATE_MOTOR_MINUS  = 8;   //         Plattform Motor: Richtung Minus/Counter clockwise
const char PIN_FOOTPRINT_LEFT_SERVO   =  9;  // PWM     Trittplatte links: Servo position rechts, 1ms..2ms=-90°..+90°
const char PIN_FOOTPRINT_RIGHT_SERVO  = 10;   // PWM     Trittplatte rechts: Servo position links, 1ms..2ms=-90°..+90°
const char PIN_BASEPLATE_MOTOR_ENABLE = 11;  // PWM490
const char PIN_STEPPER_COIL1_PLUS     = 12;  //         Stepper Driver for wire pair: red/blue (IN4), The stepper is always enabled
const char PIN_STEPPER_COIL1_MINUS    = 13;  //         Stepper Driver for wire pair: red/blue (IN3), The stepper is always enabled
const char PIN_RESERVE1               = A0;  // A0      
const char PIN_STEPPER_COIL2_MINUS    = A1;  // A1      Stepper Driver for wire pair: Green/Black (IN1), The stepper is always enabled

/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */

typedef struct {
  int X;
  int Y;
  int Z;
} tGY271_Offset;

typedef struct {
  boolean SetOffset;
  tGY271_Offset Value;
  tGY271_Offset Offset;
  bool Valid;
  bool Started;
} tGY271;

typedef struct {
  double X;
  double Y;
} tPosition;

typedef struct {
  long fx;                  // X Position der Trittplatte im Raum
  long fy;                  // Y Position der Trittplatte im Raum
  long b;                   // gewünschter Abstand der Trittplatte zum Drehzentrum
  float deltaAlpha;         // gewünschter Delta-Dreh-Winkel der Plattform
  float alpha;              // gewünschter Dreh-Winkel der Plattform
  float deltaBeta;          // gewünschter Delta-Dreh-Winkel der Plattform
  int   ServoMicroseconds;  // gewünschter Dreh-Winkel der Trittplattform in Servo-gerechter Zahl (1000 ... 2000 = -90° ... +90°) 
  float beta;               // gewünschter Dreh-Winkel der Trittplattform
  tGY271 GY271[6];          // Sensorvalues from all of the GY271 sensors
  tPosition Center;         // The current center of the foot above the footplate
  bool CenterValid;
  CI2C::Handle I2C_TCA9548A_Upper;
  CI2C::Handle I2C_TCA9548A_Lower;
} tFootprint;


byte    cGY271_ReceiveBuffer[10]={0};
tGY271 *pGY271_Callback_Sensor;
CI2C::Handle I2C_GY271;
tFootprint FootprintLeft;
tFootprint FootprintRight;
byte bSTOP_CurrentTCA9548A;
byte bSTOP_CurrentTCA9548A_Port;
byte    cTCA9548A_SendBuffer[]={0,1,2,4,8,16,32,64,128};
byte    cZero[1]={0};
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) };
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 };

void fTCA9548A_Disable_I2C_BusDevice (CI2C::Handle bModule) {
  byte bStatus;
  bSTOP_CurrentTCA9548A = bModule.device_address;
  bSTOP_CurrentTCA9548A_Port=0;
  if (bStatus=nI2C->Write (bModule, &cTCA9548A_SendBuffer[0], 1)!=0) fSTOP_I2C (bModule.device_address,bStatus,"fTCA9548A_Disable_I2C_BusDevice");
}

void fSTOP_I2C (byte bI2CAddress, byte bI2CResult, char *pFunction) {
  Serial.println ("SYSTEM STOPPED BECAUSE OF I2C ERROR.");
  Serial.print   ("Current TCA9548A: 0x");  Serial.println (bSTOP_CurrentTCA9548A,HEX);
  Serial.print   ("TCA9548A Port   : 0x");  Serial.println (bSTOP_CurrentTCA9548A_Port);
  Serial.print   ("I2C destination : 0x");  Serial.println   (bI2CAddress,HEX);
  Serial.print   ("Message         : ");
  switch (bI2CResult)
  {
    case 0: Serial.println ("success"); break;
    case 1: Serial.println ("busy"); break;
    case 2: Serial.println ("timeout"); break;
    case 3: Serial.println ("data too long to fit in transmit buffer"); break;
    case 4: Serial.println ("memory allocation failure"); break;
    case 5: Serial.println ("attempted illegal transition of state"); break;
    case 6: Serial.println ("received NACK on transmit of address"); break;
    case 7: Serial.println ("received NACK on transmit of data"); break;
    case 8: Serial.println ("illegal start or stop condition on bus"); break;
    case 9: Serial.println ("lost bus arbitration to other master"); break;
    case 100: Serial.println ("Value overflow error at GY271"); break;
  }
  Serial.print   ("Functionname    : ");
  Serial.println (pFunction);
  while (1==1) 
  {
    delay(1);
    Serial.flush();
  };
}

void fTCA9548A_Select_I2C_BusDevice (CI2C::Handle bModule, byte bPort) {
  static CI2C::Handle PreviousModule;
  byte bStatus;
  if ((PreviousModule.device_address>0)&&(PreviousModule.device_address!=bModule.device_address))
  { // Disable the previous Module:
    fTCA9548A_Disable_I2C_BusDevice (PreviousModule);
  }
  PreviousModule = bModule;
  bSTOP_CurrentTCA9548A = bModule.device_address;
  bSTOP_CurrentTCA9548A_Port=1<<bPort;
  if (bStatus=nI2C->Write (bModule, &cTCA9548A_SendBuffer[bPort+1], 1)!=0) fSTOP_I2C (bModule.device_address,bStatus,"fTCA9548A_Select_I2C_BusDevice");
}

void fGY271_Callback (const uint8_t bStatus) {
  int x, y, z;
  long lx, ly, lz;
  byte s;
  if (bStatus!=0)
  {
    fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Callback");
  }  
  x = (int)(int16_t)(cGY271_ReceiveBuffer[0] | cGY271_ReceiveBuffer[1] << 8);
  y = (int)(int16_t)(cGY271_ReceiveBuffer[2] | cGY271_ReceiveBuffer[3] << 8);
  z = (int)(int16_t)(cGY271_ReceiveBuffer[4] | cGY271_ReceiveBuffer[5] << 8);
  s = cGY271_ReceiveBuffer[6];
  if (pGY271_Callback_Sensor->SetOffset)
  {
    pGY271_Callback_Sensor->Offset.X = x;
    pGY271_Callback_Sensor->Offset.Y = y;
    pGY271_Callback_Sensor->Offset.Z = z;
    pGY271_Callback_Sensor->SetOffset = 0;
  }
  lx = x - pGY271_Callback_Sensor->Offset.X;
  ly = y - pGY271_Callback_Sensor->Offset.Y;
  lz = z - pGY271_Callback_Sensor->Offset.Z;
  pGY271_Callback_Sensor->Value.X = lx;
  pGY271_Callback_Sensor->Value.Y = ly;
  pGY271_Callback_Sensor->Value.Z = lz;
/*
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
  pGY271_Callback_Sensor->Valid=1;
}

void fGY271_Read_Value (tGY271 *pGY271) {
  byte bStatus;
  pGY271_Callback_Sensor = pGY271;
  pGY271->Valid=0;
  pGY271->Started=1;
  if (bStatus=nI2C->Write (I2C_GY271, &cZero[0], 1)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Read_Value");
  if (bStatus=nI2C->Read (I2C_GY271, &cGY271_ReceiveBuffer[0], (uint32_t)6, fGY271_Callback)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Read_Value");
}

bool fCalculate_Intersection (double a1, double b1, double S1x, double S1y, double a2, double b2, double S2x, double S2y, tPosition *Position) {
  double C1;
  double C2;
  double M1;
  double X;
  double Y;
  double divisor;
  if ((S1x==0)&&(S2x==0)) return false; // We can not calculate, otherwise we will have a "Division by zero"
  if (S1x==0)                           // The first line is vertical, so use this method to calculate:
  {
    C2 = b2 - (S2y/S2x) * (a2 - a1);
    X  = a1;
    Y  = C2;
  }
  else
  if (S2x==0)                           // The second line is vertical, so use this method to calculate:
  {
    C1 = b1 - (S1y/S1x) * (a1 - a2);
    X  = a2;
    Y  = C1;    
  }
  else                                  // None of the lines is vertical, so use this method to calculate:
  {
    divisor = (S1y/S1x) - (S2y/S2x);
    if (divisor==0) return false;
    C1 = b1 - ((S1y / S1x) * a1);
    M1 = (S1y / S1x);
    X = ( b2 - (S2y*a2/S2x) - ( b1 - (S1y*a1/S1x) ) ) / ( divisor );
    Y = M1 * X + C1;
  }
  // Return the Result:
  Position->X = X;
  Position->Y = Y;
  return true;
}

double fCalculate_Distance (double x1, double y1, double x2, double y2) {
  return sqrt( ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)) );
}

bool fCalculate_GY271_CenterOfFootplate (tFootprint *Footplate) {
  tPosition Intersection_S0S1;
  tPosition Intersection_S0S2;
  tPosition Intersection_S1S2;
  tPosition Intersection_S3S4;
  tPosition Intersection_S3S5;
  tPosition Intersection_S4S5;
  tPosition NorthPole;
  tPosition SouthPole;

  // UPPER FOOTPLATE:
  
  // Calculate the intersection between 1.Sensor versus 2.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[0], GY271_Sensor_Y[0], Footplate->GY271[0].Value.X, Footplate->GY271[0].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[1], GY271_Sensor_Y[1], Footplate->GY271[1].Value.X, Footplate->GY271[1].Value.Y,  // versus 2.Sensor
                               &Intersection_S0S1)==false) return false;
  
  // Calculate the intersection between 1.Sensor versus 3.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[0], GY271_Sensor_Y[0], Footplate->GY271[0].Value.X, Footplate->GY271[0].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[2], GY271_Sensor_Y[2], Footplate->GY271[2].Value.X, Footplate->GY271[2].Value.Y,  // versus 2.Sensor
                               &Intersection_S0S2)==false) return false;
  
  // Calculate the intersection between 2.Sensor versus 3.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[1], GY271_Sensor_Y[1], Footplate->GY271[1].Value.X, Footplate->GY271[1].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[2], GY271_Sensor_Y[2], Footplate->GY271[2].Value.X, Footplate->GY271[2].Value.Y,  // versus 2.Sensor
                               &Intersection_S1S2)==false) return false;
                               
  // LOWER FOOTPLATE:
  
  // Calculate the intersection between 4.Sensor versus 5.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[3], GY271_Sensor_Y[3], Footplate->GY271[3].Value.X, Footplate->GY271[3].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[4], GY271_Sensor_Y[4], Footplate->GY271[4].Value.X, Footplate->GY271[4].Value.Y,  // versus 2.Sensor
                               &Intersection_S3S4)==false) return false;
  
  // Calculate the intersection between 4.Sensor versus 6.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[3], GY271_Sensor_Y[3], Footplate->GY271[3].Value.X, Footplate->GY271[3].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[5], GY271_Sensor_Y[5], Footplate->GY271[5].Value.X, Footplate->GY271[5].Value.Y,  // versus 2.Sensor
                               &Intersection_S3S5)==false) return false;
  
  // Calculate the intersection between 5.Sensor versus 6.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[4], GY271_Sensor_Y[4], Footplate->GY271[4].Value.X, Footplate->GY271[4].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[5], GY271_Sensor_Y[5], Footplate->GY271[5].Value.X, Footplate->GY271[5].Value.Y,  // versus 2.Sensor
                               &Intersection_S4S5)==false) return false;

  // UPPER FOOTPLATE:
  
  double Distance_S0S1_S0S2;
  double Distance_S0S1_S1S2;
  double Distance_S0S2_S1S2;
  double Distance_S3S4_S3S5;
  double Distance_S3S4_S4S5;
  double Distance_S3S5_S4S5;

  // Calculate the distance between each intersection:

  // UPPER FOOTPLATE:
  
  Distance_S0S1_S0S2 = fCalculate_Distance ( Intersection_S0S1.X, Intersection_S0S1.Y,
                                             Intersection_S0S2.X, Intersection_S0S2.Y);
  Distance_S0S1_S1S2 = fCalculate_Distance ( Intersection_S0S1.X, Intersection_S0S1.Y,
                                             Intersection_S1S2.X, Intersection_S1S2.Y);
  Distance_S0S2_S1S2 = fCalculate_Distance ( Intersection_S0S2.X, Intersection_S0S2.Y,
                                             Intersection_S1S2.X, Intersection_S1S2.Y);
  // LOWER FOOTPLATE:
  Distance_S3S4_S3S5 = fCalculate_Distance ( Intersection_S3S4.X, Intersection_S3S4.Y,
                                             Intersection_S3S5.X, Intersection_S3S5.Y);
  Distance_S3S4_S4S5 = fCalculate_Distance ( Intersection_S3S4.X, Intersection_S3S4.Y,
                                             Intersection_S4S5.X, Intersection_S4S5.Y);
  Distance_S3S5_S4S5 = fCalculate_Distance ( Intersection_S3S5.X, Intersection_S3S5.Y,
                                             Intersection_S4S5.X, Intersection_S4S5.Y);
  // Calculate the "north-pole":
  if ((Distance_S0S1_S0S2<Distance_S0S1_S1S2) && (Distance_S0S1_S0S2<Distance_S0S2_S1S2))
  { // Distance_S0S1_S0S2 is the smalest one:
    NorthPole.X = (Intersection_S0S1.X + Intersection_S0S2.X) / 2;
    NorthPole.Y = (Intersection_S0S1.Y + Intersection_S0S2.Y) / 2;
  }
  else
  if ((Distance_S0S1_S1S2<Distance_S0S1_S0S2) && (Distance_S0S1_S1S2<Distance_S0S2_S1S2))
  { // Distance_S0S2 is the smalest one:
    NorthPole.X = (Intersection_S0S1.X + Intersection_S1S2.X) / 2;
    NorthPole.Y = (Intersection_S0S1.Y + Intersection_S1S2.Y) / 2;
  }
  else
  if ((Distance_S0S2_S1S2<Distance_S0S1_S0S2) && (Distance_S0S2_S1S2<Distance_S0S1_S1S2))
  { // Distance_S1S2 is the smalest one:
    NorthPole.X = (Intersection_S0S2.X + Intersection_S1S2.X) / 2;
    NorthPole.Y = (Intersection_S0S2.Y + Intersection_S1S2.Y) / 2;
  }

  // Calculate the "south-pole":

  if ((Distance_S3S4_S3S5<Distance_S3S4_S4S5) && (Distance_S3S4_S3S5<Distance_S3S5_S4S5))
  { // Distance_S3S4 is the smalest one:
    SouthPole.X = (Intersection_S3S4.X + Intersection_S3S5.X) / 2;
    SouthPole.Y = (Intersection_S3S4.Y + Intersection_S3S5.Y) / 2;
  }
  else
  if ((Distance_S3S4_S4S5<Distance_S3S4_S3S5) && (Distance_S3S4_S4S5<Distance_S3S5_S4S5))
  { // Distance_S3S5 is the smalest one:
    SouthPole.X = (Intersection_S3S4.X + Intersection_S4S5.X) / 2;
    SouthPole.Y = (Intersection_S3S4.Y + Intersection_S4S5.Y) / 2;
  }
  else
  if ((Distance_S3S5_S4S5<Distance_S3S4_S3S5) && (Distance_S3S5_S4S5<Distance_S3S4_S4S5))
  { // Distance_S4S5 is the smalest one:
    SouthPole.X = (Intersection_S3S5.X + Intersection_S4S5.X) / 2;
    SouthPole.Y = (Intersection_S3S5.Y + Intersection_S4S5.Y) / 2;
  }    

  // CALCULATE CENTER OF FOOTPLATE:

  Footplate->Center.X = (NorthPole.X + SouthPole.X) / 2;
  Footplate->Center.Y = (NorthPole.Y + SouthPole.Y) / 2;
  
  Footplate->CenterValid=1;
  
  return true;
}

void fGY271_Read_Values () {
  static byte bIndex=0;
  // RIGHT FOOTPRINT:
  // ~~~~~~~~~~~~~~~~
  switch (bIndex)
  {
    case 0:   FootprintRight.CenterValid=0;                                           // The current data is not yet valid
              fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 0);  // select right footprint, upper, GY271 channel #0
              fGY271_Read_Value (&FootprintRight.GY271[0]);                           // Request values
              bIndex++;
              break;
    case 1:   if (FootprintRight.GY271[0].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 3);// select right footprint, upper, GY271 channel #3
                fGY271_Read_Value (&FootprintRight.GY271[1]);                         // Request values
                bIndex++;
              }
              break;
    case 2:   if (FootprintRight.GY271[1].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper, 4);// select right footprint, upper, GY271 channel #4
                fGY271_Read_Value (&FootprintRight.GY271[2]);                         // Request values
                bIndex++;
              }
              break;
    case 3:   if (FootprintRight.GY271[2].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 0);// select right footprint, lower, GY271 channel #0
                fGY271_Read_Value (&FootprintRight.GY271[3]);                         // Request values
                bIndex++;
              }
              break;
    case 4:   if (FootprintRight.GY271[3].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 3);// select right footprint, lower, GY271 channel #3
                fGY271_Read_Value (&FootprintRight.GY271[4]);                         // Request values
                bIndex++;
              }
              break;
    case 5:   if (FootprintRight.GY271[4].Valid)
              {
                fTCA9548A_Select_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower, 4);// select right footprint, lower, GY271 channel #4
                fGY271_Read_Value (&FootprintRight.GY271[5]);                         // Request values
                bIndex++;
              }
              break;
    case 6:   if (FootprintRight.GY271[5].Valid)
              {
                fCalculate_GY271_CenterOfFootplate (&FootprintRight);                 // Calculate the center of the footplate
                bIndex=0;
              }
  }
}

/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
class tCNC_Calculator {
  private:
    long lSourcePositionPrimaryAxis;
    long lSourcePositionSecondaryAxis;
    long lTargetPositionPrimaryAxis;
    long lTargetPositionSecondaryAxis;
    long lDistancePrimaryAxis;
    long lDistanceSecondaryAxis;
    
  public:
    tCNC_Calculator ();
    void setSourcePositionPrimaryAxis (long lSourcePosition);
    void setSourcePositionSecondaryAxis (long lSourcePosition);
    void setTargetPositionPrimaryAxis (long lTargetPosition);
    void setTargetPositionSecondaryAxis (long lTargetPosition);
    long setCurrentPositionPrimaryAxis (long lCurrentPosition);
    long getSourcePositionPrimaryAxis ();
    long getSourcePositionSecondaryAxis ();
    long getTargetPositionPrimaryAxis ();
    long getTargetPositionSecondaryAxis ();
};

long tCNC_Calculator::getSourcePositionPrimaryAxis () { return lSourcePositionPrimaryAxis; };
long tCNC_Calculator::getSourcePositionSecondaryAxis () { return lSourcePositionSecondaryAxis; };
long tCNC_Calculator::getTargetPositionPrimaryAxis () { return lTargetPositionPrimaryAxis; };
long tCNC_Calculator::getTargetPositionSecondaryAxis () { return lTargetPositionSecondaryAxis; };

void tCNC_Calculator::setSourcePositionPrimaryAxis (long lSourcePosition) {
  lSourcePositionPrimaryAxis=lSourcePosition;
  lDistancePrimaryAxis=lTargetPositionPrimaryAxis-lSourcePositionPrimaryAxis;
}

void tCNC_Calculator::setTargetPositionPrimaryAxis (long lTargetPosition) {
  lTargetPositionPrimaryAxis=lTargetPosition;
  lDistancePrimaryAxis=lTargetPositionPrimaryAxis-lSourcePositionPrimaryAxis;
}

void tCNC_Calculator::setSourcePositionSecondaryAxis (long lSourcePosition) {
  lSourcePositionSecondaryAxis=lSourcePosition;
  lDistanceSecondaryAxis=lTargetPositionSecondaryAxis-lSourcePositionSecondaryAxis;
}

void tCNC_Calculator::setTargetPositionSecondaryAxis (long lTargetPosition) {
  lTargetPositionSecondaryAxis=lTargetPosition;
  lDistanceSecondaryAxis=lTargetPositionSecondaryAxis-lSourcePositionSecondaryAxis;
}

long tCNC_Calculator::setCurrentPositionPrimaryAxis (long lCurrentPosition) {
  double dDistancePrimary;
  double dDistanceSecondary;  
  double dFactor;
  double dAlreadyDrivenPrimary;
  long lAlreadyDrivenPrimary;
  double dSourcePositionSecondaryAxis;
  double dNewPositionSecondaryAxis;
  double dTargetPositionSecondaryAxis;
  long lNewPositionSecondaryAxis;
//  double dDistanceToCenter;
//  dDistanceToCenter = sqrt (fFootX*fFootX+fFootY*fFootY);
  
  if (lDistancePrimaryAxis==0)
    return lTargetPositionSecondaryAxis;
  lAlreadyDrivenPrimary = lCurrentPosition - lSourcePositionPrimaryAxis;
  // Convert all to double:
  dAlreadyDrivenPrimary = lAlreadyDrivenPrimary;
  dDistancePrimary=lDistancePrimaryAxis;
  dDistanceSecondary=lDistanceSecondaryAxis;
  dSourcePositionSecondaryAxis = lSourcePositionSecondaryAxis;
  // Calculate destination value for secondary axis:
  dFactor = dDistanceSecondary/dDistancePrimary;
  dNewPositionSecondaryAxis = dSourcePositionSecondaryAxis+(dAlreadyDrivenPrimary * dFactor);
  lNewPositionSecondaryAxis = dNewPositionSecondaryAxis;
  return lNewPositionSecondaryAxis;
}

tCNC_Calculator::tCNC_Calculator (void) {
  //
}

/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */

class tDC_STEPPERMOTOR {
  
  typedef struct {
    bool bPIN_COIL1_PLUS;
    bool bPIN_COIL2_PLUS;
    bool bPIN_COIL1_MINUS;
    bool bPIN_COIL2_MINUS;
  } tSteps;
  
  typedef struct {
    byte bPIN_COIL1_PLUS;
    byte bPIN_COIL2_PLUS;
    byte bPIN_COIL1_MINUS;
    byte bPIN_COIL2_MINUS;
    unsigned long ulExecuteEveryMicros;
    long lTargetPosition;
    bool bAutoTurnOff;
    long lCurrentPosition;
  } tConfiguration;
  
  private:
    tConfiguration Configuration;
    unsigned long ulStart;
    bool bTurnedOn;
    byte bStepIndex;
    tSteps Steps [4];
    
  public:
    void TurnOn ();
    void TurnOff ();
    void Execute ();
    void setTargetPosition (long lTargetPosition, bool bAutoTurnOff);
    long getTargetPosition ();
    void setCurrentPosition (long lNewPosition);
    long getCurrentPosition ();
    tDC_STEPPERMOTOR (byte bPIN_COIL1_PLUS, byte bPIN_COIL1_MINUS, byte bPIN_COIL2_PLUS, byte bPIN_COIL2_MINUS, unsigned long ulExecuteEveryMicros);
};

tDC_STEPPERMOTOR::tDC_STEPPERMOTOR(byte bPIN_COIL1_PLUS, byte bPIN_COIL1_MINUS, byte bPIN_COIL2_PLUS, byte bPIN_COIL2_MINUS, unsigned long ulExecuteEveryMicros) {
  bTurnedOn=false;
  Configuration.bPIN_COIL1_PLUS  = bPIN_COIL1_PLUS;
  Configuration.bPIN_COIL2_PLUS  = bPIN_COIL2_PLUS;
  Configuration.bPIN_COIL1_MINUS = bPIN_COIL1_MINUS;
  Configuration.bPIN_COIL2_MINUS = bPIN_COIL2_MINUS;
  Configuration.ulExecuteEveryMicros = ulExecuteEveryMicros;
  Steps[3].bPIN_COIL1_PLUS=HIGH; Steps[3].bPIN_COIL2_PLUS=LOW;  Steps[3].bPIN_COIL1_MINUS=LOW;  Steps[3].bPIN_COIL2_MINUS=LOW;
  Steps[2].bPIN_COIL1_PLUS=LOW;  Steps[2].bPIN_COIL2_PLUS=HIGH; Steps[2].bPIN_COIL1_MINUS=LOW;  Steps[2].bPIN_COIL2_MINUS=LOW;
  Steps[1].bPIN_COIL1_PLUS=LOW;  Steps[1].bPIN_COIL2_PLUS=LOW;  Steps[1].bPIN_COIL1_MINUS=HIGH; Steps[1].bPIN_COIL2_MINUS=LOW;
  Steps[0].bPIN_COIL1_PLUS=LOW;  Steps[0].bPIN_COIL2_PLUS=LOW;  Steps[0].bPIN_COIL1_MINUS=LOW;  Steps[0].bPIN_COIL2_MINUS=HIGH;
  Configuration.lCurrentPosition=0;
  Configuration.lTargetPosition=0;
  bStepIndex=0;
  pinMode (Configuration.bPIN_COIL1_PLUS, OUTPUT);
  pinMode (Configuration.bPIN_COIL2_PLUS, OUTPUT);
  pinMode (Configuration.bPIN_COIL1_MINUS, OUTPUT);
  pinMode (Configuration.bPIN_COIL2_MINUS, OUTPUT);
  ulStart=micros ();
}

void tDC_STEPPERMOTOR::TurnOn () {
  if (bTurnedOn==false)
  {
    digitalWrite (Configuration.bPIN_COIL1_PLUS,  Steps[bStepIndex].bPIN_COIL1_PLUS);
    digitalWrite (Configuration.bPIN_COIL2_PLUS,  Steps[bStepIndex].bPIN_COIL2_PLUS);
    digitalWrite (Configuration.bPIN_COIL1_MINUS, Steps[bStepIndex].bPIN_COIL1_MINUS);
    digitalWrite (Configuration.bPIN_COIL2_MINUS, Steps[bStepIndex].bPIN_COIL2_MINUS);
    ulStart = micros ();
    bTurnedOn=true;
    Serial.println ("StepperOn");
  }
}

void tDC_STEPPERMOTOR::TurnOff () {
  if (bTurnedOn==true)
  {
    bTurnedOn=false;
    digitalWrite (Configuration.bPIN_COIL1_PLUS,LOW);
    digitalWrite (Configuration.bPIN_COIL2_PLUS,LOW);
    digitalWrite (Configuration.bPIN_COIL1_MINUS,LOW);
    digitalWrite (Configuration.bPIN_COIL2_MINUS,LOW);  
    Serial.println ("StepperOff");
  }
}

void tDC_STEPPERMOTOR::setTargetPosition (long lTargetPosition, bool bAutoTurnOff) {
  if (lTargetPosition<0) lTargetPosition=0;
  Configuration.bAutoTurnOff = bAutoTurnOff;
  Configuration.lTargetPosition = lTargetPosition; 
}

long tDC_STEPPERMOTOR::getTargetPosition () {
  return Configuration.lTargetPosition;
}

void tDC_STEPPERMOTOR::setCurrentPosition (long lNewPosition) {
  if (lNewPosition<0) lNewPosition=0;
  Configuration.lCurrentPosition = lNewPosition;
}

long tDC_STEPPERMOTOR::getCurrentPosition () {
  return Configuration.lCurrentPosition;
}

void tDC_STEPPERMOTOR::Execute () {
  unsigned long ulStop;
  unsigned long ulDuration;

  if (bTurnedOn==false) return;

  ulStop = micros ();
  if (ulStop<ulStart)
    ulDuration = (0xFFFFFFFF - ulStart) + ulStop; // Overflow
  else
    ulDuration = ulStop - ulStart;

  if (ulDuration<Configuration.ulExecuteEveryMicros) return;
  
  if (Configuration.lTargetPosition!=Configuration.lCurrentPosition)
  {
    if (Configuration.lTargetPosition>Configuration.lCurrentPosition)
    {
      Configuration.lCurrentPosition++;
      if (bStepIndex<3)
        bStepIndex++;
      else
        bStepIndex=0;
    }
    else
    {
      Configuration.lCurrentPosition--;
      if (bStepIndex>0)
        bStepIndex--;
      else
        bStepIndex=3;
    }
    digitalWrite (Configuration.bPIN_COIL1_PLUS,  Steps[bStepIndex].bPIN_COIL1_PLUS);
    digitalWrite (Configuration.bPIN_COIL2_PLUS,  Steps[bStepIndex].bPIN_COIL2_PLUS);
    digitalWrite (Configuration.bPIN_COIL1_MINUS, Steps[bStepIndex].bPIN_COIL1_MINUS);
    digitalWrite (Configuration.bPIN_COIL2_MINUS, Steps[bStepIndex].bPIN_COIL2_MINUS);
    ulStart = micros ();
  }
  else
  { 
    if (Configuration.bAutoTurnOff)
    {
      TurnOff ();
    }
  }
}

/* ******************************************************************************************************** */
/* ******************************************************************************************************** */
/* ******************************************************************************************************** */

class tDC_PIDMOTOR {
  
  typedef struct {
    float fP;
    float fI;
    float fD;    
    long lLastError;
    unsigned long ulSampleLastTime;
    volatile long lIntegral;
    long lLastValue;
    float iTerm;
    float fsampleTime;
  } tPIDParameter;
  
  typedef struct {
    long lMaxValue;
    byte bPreviousValue;
    byte bPIN_MOTOR_PWM;
  } tPWMParameter;

  typedef struct {
    byte bPIN_MOTOR_A;
    byte bPIN_MOTOR_B;
    bool bTurnedOn;    
    long lPreviousDirection;
    long lTargetPosition;
  } tMotorParameter;
  
  typedef struct {
    byte bPIN_ENCODER_CHANNEL_A;
    byte bPIN_ENCODER_CHANNEL_B;    
    volatile long lEncoderPosition;
    bool bEncoderPreviousChannelA;
    bool bEncoderPreviousChannelB;
    bool bEncoderInverted;
  } tEncoderParameter;
  
  private:
    tEncoderParameter EncoderParameter;
    tPWMParameter     PWMParameter;
    tMotorParameter   MotorParameter;

    volatile bool bInterruptEnabled;
    volatile bool bExecuteIsActive;
    long compute();

  public:
    tPIDParameter     PIDParameter;
    tDC_PIDMOTOR (byte bPIN_MOTOR_PWM, long bPWM_Max, byte bPIN_MOTOR_A, byte bPIN_MOTOR_B, byte bPIN_ENCODER_CHANNEL_A, byte bPIN_ENCODER_CHANNEL_B);
    void Execute();
    byte setPWM(long PWM_Width);
    void setTargetPosition (long lTargetPosition);
    long getTargetPosition ();
    void setEncoderInverted (bool bInverted);
    void setCurrentPosition (long lNewPosition);
    long getCurrentPosition();
    void setDirection(long lDirection);
    void setPID(float pValue, float iValue, float dValue);
    float getPID_P(void);
    float getPID_I(void);
    float getPID_D(void);
    void EncoderInterrupt ();
    void TurnOn ();
    void TurnOff ();
    bool loopAutoTune(void);
    bool loopCheckEncoderInverted (void);
};

void tDC_PIDMOTOR::TurnOn () {
  if (MotorParameter.bTurnedOn==false)
  {
    PIDParameter.lIntegral=0;
    analogWrite (PWMParameter.bPIN_MOTOR_PWM,PWMParameter.bPreviousValue);
    PIDParameter.ulSampleLastTime = micros();
    MotorParameter.bTurnedOn=true;
    Serial.println ("MotorOn");
  }
}

void tDC_PIDMOTOR::TurnOff () {
  if (MotorParameter.bTurnedOn==true)
  {
    MotorParameter.bTurnedOn=false;
    analogWrite (PWMParameter.bPIN_MOTOR_PWM,0);
    Serial.println ("MotorOff");
  }
}

void tDC_PIDMOTOR::setTargetPosition (long lTargetPosition) {
  if (MotorParameter.lTargetPosition!=lTargetPosition)
  {
    PIDParameter.lIntegral=0;
    MotorParameter.lTargetPosition=lTargetPosition;
  }
}

long tDC_PIDMOTOR::getTargetPosition () {
  return MotorParameter.lTargetPosition;
}

void tDC_PIDMOTOR::setEncoderInverted (bool bInverted) {
  EncoderParameter.bEncoderInverted=bInverted;
}

void tDC_PIDMOTOR::setCurrentPosition (long lNewPosition) {
  EncoderParameter.lEncoderPosition=lNewPosition; 
}

byte tDC_PIDMOTOR::setPWM(long lNewPWMValue) {
  byte bNewPWMValue;
  lNewPWMValue=abs(lNewPWMValue);
  if (lNewPWMValue>PWMParameter.lMaxValue)
  {
    lNewPWMValue=PWMParameter.lMaxValue;
  }
  bNewPWMValue=lNewPWMValue;
  if (bNewPWMValue!=PWMParameter.bPreviousValue)
  {
    PWMParameter.bPreviousValue=bNewPWMValue;
    if (MotorParameter.bTurnedOn==true)
    {
      analogWrite (PWMParameter.bPIN_MOTOR_PWM,PWMParameter.bPreviousValue);
    }
  }
  return PWMParameter.bPreviousValue;
}

long tDC_PIDMOTOR::getCurrentPosition () {
  return EncoderParameter.lEncoderPosition;
}

void tDC_PIDMOTOR::setDirection (long lDirection) {
  if (MotorParameter.lPreviousDirection!=lDirection)
  {
    if (lDirection>0)
    {
      digitalWrite (MotorParameter.bPIN_MOTOR_A,HIGH);
      digitalWrite (MotorParameter.bPIN_MOTOR_B,LOW);
    }
    else
    if (lDirection<0)
    {
      digitalWrite (MotorParameter.bPIN_MOTOR_A,LOW);
      digitalWrite (MotorParameter.bPIN_MOTOR_B,HIGH);    
    }
    else
    {
      digitalWrite (MotorParameter.bPIN_MOTOR_A,LOW);
      digitalWrite (MotorParameter.bPIN_MOTOR_B,LOW);    
    }
    MotorParameter.lPreviousDirection=lDirection;
  }
}

long tDC_PIDMOTOR::compute() {
  unsigned long ulcurTime;
  unsigned long ulsampleTime;
  float fsampleTime;
  long lcurError;
  float fcurError;
  float pTerm;
  float iTerm;
  float dTerm;
  long lDiff;
  float fDiff;

  ulcurTime = micros();
  if (ulcurTime<PIDParameter.ulSampleLastTime)
    ulsampleTime = (0xFFFFFFFF - PIDParameter.ulSampleLastTime) + ulcurTime;
  else
    ulsampleTime = ulcurTime - PIDParameter.ulSampleLastTime;
  if (ulsampleTime<1000) return PIDParameter.lLastValue; // Less than 1ms, do nothing
  
  PIDParameter.ulSampleLastTime=ulcurTime;
  fsampleTime = ulsampleTime;
  fsampleTime = fsampleTime / 1000000; // fsampleTime is now in seconds
  
  lcurError = MotorParameter.lTargetPosition - EncoderParameter.lEncoderPosition;
  fcurError = lcurError;
  
  pTerm = PIDParameter.fP * fcurError;
  if (bInterruptEnabled==true)
  {
    if (abs(lcurError)<10)
    {
      PIDParameter.lIntegral  = PIDParameter.lIntegral + lcurError;
      if (PIDParameter.lIntegral>1000000) PIDParameter.lIntegral=1000000;
      if (PIDParameter.lIntegral<-1000000) PIDParameter.lIntegral=-1000000;
      iTerm = PIDParameter.fI*PIDParameter.lIntegral*fsampleTime;
      PIDParameter.iTerm = iTerm;
      PIDParameter.fsampleTime = fsampleTime;
    }
    else
      iTerm=0;
  }
  else
    iTerm=0;
  if (bInterruptEnabled==true)
  {
    lDiff = lcurError - PIDParameter.lLastError;
    fDiff = lDiff;
    dTerm = PIDParameter.fD * fDiff / fsampleTime;
    PIDParameter.lLastError=lcurError;
  }
  else
    dTerm=0;
  PIDParameter.lLastValue=round(pTerm+iTerm+dTerm);
  return PIDParameter.lLastValue;
}

tDC_PIDMOTOR::tDC_PIDMOTOR(byte bPIN_MOTOR_PWM, long lPWM_Max, byte bPIN_MOTOR_A, byte bPIN_MOTOR_B, byte bPIN_ENCODER_CHANNEL_A, byte bPIN_ENCODER_CHANNEL_B) {
  // War bisher gut geeignet
  //PIDParameter.fP = 6;
  //PIDParameter.fI = 1.21;
  //PIDParameter.fD = 0.08;

  // Bisher am besten auch mit 500er Schritten:
  PIDParameter.fP = 6.4;
  PIDParameter.fI = 1.49; // 1.21;
  PIDParameter.fD = 0.49;//  0.63;
  PIDParameter.lIntegral = 0;
  PIDParameter.lLastError = 0;
  PWMParameter.lMaxValue = lPWM_Max;
  PWMParameter.bPIN_MOTOR_PWM = bPIN_MOTOR_PWM;  
  EncoderParameter.bEncoderInverted=false;
  EncoderParameter.bPIN_ENCODER_CHANNEL_A=bPIN_ENCODER_CHANNEL_A;
  EncoderParameter.bPIN_ENCODER_CHANNEL_B=bPIN_ENCODER_CHANNEL_B;
  MotorParameter.bPIN_MOTOR_A=bPIN_MOTOR_A;
  MotorParameter.bPIN_MOTOR_B=bPIN_MOTOR_B;
  
  pinMode (PWMParameter.bPIN_MOTOR_PWM, OUTPUT);
  analogWrite(PWMParameter.bPIN_MOTOR_PWM,0);
  pinMode (MotorParameter.bPIN_MOTOR_A, OUTPUT);
  pinMode (MotorParameter.bPIN_MOTOR_B, OUTPUT);
  digitalWrite (MotorParameter.bPIN_MOTOR_A,LOW);
  digitalWrite (MotorParameter.bPIN_MOTOR_B,LOW);
  
  pinMode (EncoderParameter.bPIN_ENCODER_CHANNEL_A, INPUT_PULLUP);
  pinMode (EncoderParameter.bPIN_ENCODER_CHANNEL_B, INPUT_PULLUP);
  //digitalWrite(EncoderParameter.bPIN_ENCODER_CHANNEL_A, HIGH); //turn pullup resistor on
  //digitalWrite(EncoderParameter.bPIN_ENCODER_CHANNEL_B, HIGH); //turn pullup resistor on
  EncoderParameter.bEncoderPreviousChannelA = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_A);
  EncoderParameter.bEncoderPreviousChannelB = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_B);
  bExecuteIsActive=false;
  bInterruptEnabled=true;
}

void tDC_PIDMOTOR::setPID(float pValue, float iValue, float dValue) {
  PIDParameter.fP = pValue;
  PIDParameter.fI = iValue;
  PIDParameter.fD = dValue;
}

float tDC_PIDMOTOR::getPID_P(void) {return PIDParameter.fP; }
float tDC_PIDMOTOR::getPID_I(void) {return PIDParameter.fI; }
float tDC_PIDMOTOR::getPID_D(void) {return PIDParameter.fD; }

void tDC_PIDMOTOR::Execute () {
  long lpwmWidth;
  if (bExecuteIsActive==true) return;
  if (MotorParameter.bTurnedOn==false) return;
  bExecuteIsActive=true;
  lpwmWidth = compute();
  setDirection(lpwmWidth);
  setPWM(lpwmWidth);
  bExecuteIsActive=false;
}

bool tDC_PIDMOTOR::loopCheckEncoderInverted () {
  int ipwm = 0; 
  long lposition = 0;
  bool bResult;
  TurnOff ();
  bInterruptEnabled=false;
  delay (1300);
  setEncoderInverted (false);
  setCurrentPosition (0);
  setDirection(1);
  setPWM(0);
  TurnOn ();
  while(abs(lposition)<20){
    ipwm=setPWM(ipwm+10);
    lposition = getCurrentPosition();
    delay(1);
  }
  TurnOff ();
  if(lposition < 0)
  {
    setEncoderInverted (true);
    bResult=true;
  }
  else
    bResult=false;
  bInterruptEnabled=true;
  return bResult;
}

bool tDC_PIDMOTOR::loopAutoTune(void) {
  unsigned long ulStartMicros=0;
  unsigned long ulStopMicros=0;
  unsigned long ulDurationMicros=0;
  unsigned long ulWaveStartMicros=0;
  unsigned long ulWaveStopMicros=0;
  unsigned long ulWaveDurationMicros=0;
  unsigned long ulWaveDurationMicrosSum=0;
  float fWaveDurationAverage;
  long lPreviousEncoderPosition;
  long lCurrentEncoderPosition;
  long lWaveMinEncoderPosition;
  long lWaveMaxEncoderPosition;
  long lPOINT1;
  long lPOINT2;
  int iWaveCounts=0;
  int iWaveSumCounts=0;
  byte bCurrentDirection; // 1=left, 2=right
  byte bPreviousDirection; // 1=left, 2=right
  double Tu;
  
  bInterruptEnabled=false;
  TurnOff ();
  setPWM(0);
  delay (1000);
  lPOINT1=90;
  lPOINT2=100;
  PIDParameter.fP=0;
  PIDParameter.fI=0;
  PIDParameter.fD=0;
  setCurrentPosition (lPOINT1);
  setTargetPosition (lPOINT2);
  lCurrentEncoderPosition = EncoderParameter.lEncoderPosition;
  lPreviousEncoderPosition= lCurrentEncoderPosition;
  lWaveMinEncoderPosition=lCurrentEncoderPosition;
  lWaveMaxEncoderPosition=lCurrentEncoderPosition;
  iWaveCounts=0;
  iWaveSumCounts=0;
  
  TurnOn ();
  ulStartMicros=micros ();
  
  while (1)
  {
    Execute ();
    lCurrentEncoderPosition=EncoderParameter.lEncoderPosition;
    ulStopMicros=micros();
    if (ulStopMicros<ulStartMicros)
      ulDurationMicros = (0xFFFFFFFF - ulStartMicros) + ulStopMicros; // Overflow
    else
      ulDurationMicros = ulStopMicros-ulStartMicros;
      
    if (lCurrentEncoderPosition!=lPreviousEncoderPosition)
    { // It has been moved somehow in any direction
      ulStartMicros=micros ();
      if (lCurrentEncoderPosition<lWaveMinEncoderPosition)
        lWaveMinEncoderPosition=lCurrentEncoderPosition;
      else
      if (lCurrentEncoderPosition>lWaveMaxEncoderPosition)
        lWaveMaxEncoderPosition=lCurrentEncoderPosition;
      // Check if it is "waving":
      if ((lWaveMinEncoderPosition<MotorParameter.lTargetPosition)&&(lWaveMaxEncoderPosition>MotorParameter.lTargetPosition))
      {
        // It is waving
        if (lCurrentEncoderPosition-lPreviousEncoderPosition>0)
        {
          // It is moving right now to the right
          bCurrentDirection=2;
        }
        else
        {
          // It is moving right now to the left
          bCurrentDirection=1;
        }
        if (bCurrentDirection!=bPreviousDirection)
        {
          // Direction has been changed
          ulWaveStopMicros=micros();
          if (ulWaveStopMicros<ulWaveStartMicros)
            ulWaveDurationMicros = (0xFFFFFFFF - ulWaveStartMicros) + ulWaveStopMicros; // Overflow
          else
            ulWaveDurationMicros = ulWaveStopMicros-ulWaveStartMicros;
          ulWaveStartMicros=micros ();
          iWaveCounts++;
          if (iWaveCounts>19) // Ignore the first 20 waves
          {
            ulWaveDurationMicrosSum = ulWaveDurationMicrosSum + ulWaveDurationMicros;
            iWaveSumCounts++;
          }
          // Start a new measurement of the wavelength:
          lWaveMinEncoderPosition=lCurrentEncoderPosition;
          lWaveMaxEncoderPosition=lCurrentEncoderPosition;
          if (iWaveSumCounts>39)
          { // We have 40 real waves without changing Kp Parameter
            TurnOff ();
            setPWM (0);
            fWaveDurationAverage=ulWaveDurationMicrosSum/iWaveSumCounts;

            Tu = (fWaveDurationAverage*2) / 1000000; // Average in Seconds
            // classic PID according to Ziegler–Nichols method
            //PIDParameter.fP = 0.6 * PIDParameter.fP;
            //PIDParameter.fI = Tu / 2; /* Ki = 0,5 * Tu */
            //PIDParameter.fD = Tu / 8; /* Kd = 0,125 * Tu */

            // no overshoot according to Ziegler–Nichols method
            //PIDParameter.fP = 0.2 * PIDParameter.fP;
            //PIDParameter.fI = Tu / 2;
            //PIDParameter.fD = Tu / 3;

            // these ones works good for the used DC motor:
            PIDParameter.fP = PIDParameter.fP / 3.3;
            PIDParameter.fI = Tu / 0.13;
            PIDParameter.fD = Tu / 0.25;

            bInterruptEnabled=true;
            break;
          }
          bPreviousDirection=bCurrentDirection;
        }
      }
    }
    else
    { // There was no movement detected:
      if (ulDurationMicros>60000)
      { // No movement since 60ms
        if (lCurrentEncoderPosition==MotorParameter.lTargetPosition)
        {
          // Position is reached since 60ms.
          // But we want it waving, so choose another target position:
          TurnOff ();
          setPWM (0);
          delay (1300);
          iWaveCounts=0;
          ulWaveDurationMicrosSum=0;
          iWaveSumCounts=0;          
          lWaveMinEncoderPosition=lCurrentEncoderPosition;
          lWaveMaxEncoderPosition=lCurrentEncoderPosition;
          if (MotorParameter.lTargetPosition==lPOINT1)
            setTargetPosition (lPOINT2);
          else
            setTargetPosition (lPOINT1);
          TurnOn();
          ulStartMicros=micros ();
        }
        else
        {   // No movement since 60ms, and Targetposition not reached, increase Kp:
          TurnOff ();
          setPWM (0);
          PIDParameter.fP=PIDParameter.fP+1;
          delay (1300);
          setCurrentPosition (lPOINT1);
          setTargetPosition (lPOINT2);
          lCurrentEncoderPosition = EncoderParameter.lEncoderPosition;
          lPreviousEncoderPosition= lCurrentEncoderPosition;         
          TurnOn ();
          iWaveCounts=0;
          ulWaveDurationMicrosSum=0;
          iWaveSumCounts=0;          
          lWaveMinEncoderPosition=lCurrentEncoderPosition;
          lWaveMaxEncoderPosition=lCurrentEncoderPosition;
          ulStartMicros=micros ();
        }
      }
    }
    lPreviousEncoderPosition=lCurrentEncoderPosition;
  }
}

void tDC_PIDMOTOR::EncoderInterrupt() {
  bool bEncoderChannelA;
  bool bEncoderChannelB;
  if (EncoderParameter.bEncoderInverted)
  {
    bEncoderChannelB = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_A);
    bEncoderChannelA = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_B);    
  }
  else
  {
    bEncoderChannelA = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_A);
    bEncoderChannelB = digitalRead (EncoderParameter.bPIN_ENCODER_CHANNEL_B);
  }
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==HIGH))
  {
    if ((EncoderParameter.bEncoderPreviousChannelA==HIGH)&&(EncoderParameter.bEncoderPreviousChannelB==HIGH))
      EncoderParameter.lEncoderPosition++;
    else
      EncoderParameter.lEncoderPosition--;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==HIGH))
  {
    if ((EncoderParameter.bEncoderPreviousChannelA==LOW)&&(EncoderParameter.bEncoderPreviousChannelB==HIGH))
      EncoderParameter.lEncoderPosition--;
    else
      EncoderParameter.lEncoderPosition++;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==LOW))
  {
    if ((EncoderParameter.bEncoderPreviousChannelA==HIGH)&&(EncoderParameter.bEncoderPreviousChannelB==HIGH))
      EncoderParameter.lEncoderPosition--;
    else
      EncoderParameter.lEncoderPosition++;
  }
  else
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==LOW))
  {
    if ((EncoderParameter.bEncoderPreviousChannelA==HIGH)&&(EncoderParameter.bEncoderPreviousChannelB==LOW))
      EncoderParameter.lEncoderPosition--;
    else
      EncoderParameter.lEncoderPosition++;
  }
  EncoderParameter.bEncoderPreviousChannelA = bEncoderChannelA;
  EncoderParameter.bEncoderPreviousChannelB = bEncoderChannelB;
  if ((bInterruptEnabled==true)&&(bExecuteIsActive==false)) Execute ();
}

/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */

tDC_PIDMOTOR DC_PIDMOTOR(PIN_BASEPLATE_MOTOR_ENABLE, MOTOR_HIGH_SPEED, PIN_BASEPLATE_MOTOR_PLUS, PIN_BASEPLATE_MOTOR_MINUS, PIN_BASEPLATE_ENCODER_A, PIN_BASEPLATE_ENCODER_B);
tDC_STEPPERMOTOR DC_STEPPERMOTOR(PIN_STEPPER_COIL1_PLUS,PIN_STEPPER_COIL1_MINUS,PIN_STEPPER_COIL2_PLUS,PIN_STEPPER_COIL2_MINUS,6000);
tCNC_Calculator CNC_Calculator;
Servo sServoLeft;
Servo sServoRight;

// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect)  {  
  DC_STEPPERMOTOR.Execute ();
}

void DC_PIDMOTOR_EncoderInterrupt () {
  DC_PIDMOTOR.EncoderInterrupt ();
}

/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */

void setup(){
  sServoLeft.attach (PIN_FOOTPRINT_LEFT_SERVO);
  sServoLeft.writeMicroseconds (1500);
  sServoRight.attach (PIN_FOOTPRINT_RIGHT_SERVO);
  sServoRight.writeMicroseconds (1500+81);
  Serial.begin(115200);
  while(!Serial)
  {
    delay (1);
  };
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  DC_PIDMOTOR.TurnOff ();
  DC_STEPPERMOTOR.TurnOff ();

  dFootX=30.0;
  dFootY=0;
  
  // Millisecond timer interrupt
  //OCR0A = 0x7D;
  //TIMSK0 |= _BV(OCIE0A); 
}

void fCalculate (double fFootX, double fFootY, double *dDistanceToCenter, double *dRotationCenter) {
  // Distance from Center to Foot (stepper motor)
  *dDistanceToCenter = sqrt (fFootX*fFootX+fFootY*fFootY);
  // How much to rotate the baseplate?
  *dRotationCenter = asin (fFootY / *dDistanceToCenter); // Result will be -PI .... +PI
}

bool fControl ()
{
  static double dPreviousFootX;
  static double dPreviousFootY;
  static bool bInitialized=false;
  double dRotationCenter;
  double dDistanceToCenter;
  long lRotationCenter;
  long lDistanceToCenter;
  if (bInitialized==false)
  {
    bInitialized=true;
    dPreviousFootX=dFootX;
    dPreviousFootY=dFootY;
  }
  if ((dPreviousFootX==dFootX)&&(dPreviousFootY==dFootY)) return false;

  dPreviousFootX=dFootX;
  dPreviousFootY=dFootY;
  
  fCalculate (dFootX,dFootY,&dDistanceToCenter,&dRotationCenter);
  dRotationCenter = (dRotationCenter / (PI/2.0) ) * 600.0;            // Convert from -PI/2 ... +PI/2 to dRotationCenter (-600...+600)
  lRotationCenter = dRotationCenter;
  if (lRotationCenter>400) lRotationCenter=400;
  if (lRotationCenter<-400) lRotationCenter=-400;
  DC_PIDMOTOR.setTargetPosition (lRotationCenter);
  DC_PIDMOTOR.TurnOn ();
  
  // Convert from dDistanceToCenter (mm) to lDistanceToCenter (Steps)
  lDistanceToCenter = (dDistanceToCenter-30.0) * 5;                   // Because one step is about 0,2mm. So, 1mm to drive is 5 steps
  if (lDistanceToCenter<0) lDistanceToCenter=0;
  if (lDistanceToCenter>400) lDistanceToCenter=400;                   // No more than 400 steps (80mm)

  CNC_Calculator.setSourcePositionPrimaryAxis (DC_PIDMOTOR.getCurrentPosition ());
  CNC_Calculator.setTargetPositionPrimaryAxis (DC_PIDMOTOR.getTargetPosition ());

  CNC_Calculator.setSourcePositionSecondaryAxis (DC_STEPPERMOTOR.getCurrentPosition ());
  CNC_Calculator.setTargetPositionSecondaryAxis (lDistanceToCenter);
  
  // Display Primary Axis:
  Serial.print (CNC_Calculator.getSourcePositionPrimaryAxis ()); Serial.print ("\t");
  Serial.print (CNC_Calculator.getTargetPositionPrimaryAxis ()); Serial.print ("\t");
  // Display Secondary Axis:
  Serial.print (CNC_Calculator.getSourcePositionSecondaryAxis ()); Serial.print ("\t");
  Serial.println (CNC_Calculator.getTargetPositionSecondaryAxis ());

  
  return true;
}

void loop(){
  bool bShowPID=false;
  static long lpEP;
  static bool mMove;
  static long lMove;
  static bool bMove;
  static int iStellwinkel=0;
  static unsigned long ulIntervall;
  static unsigned long ulLastIntervall;
  int iServoLeftPosition;
  int iServoRightPosition;
  double dServoRightPosition;
  long lEP;
  char c;
  double dEP;
  long lll;

  //fGY271_Read_Values ();
  fControl();
  lll=CNC_Calculator.setCurrentPositionPrimaryAxis (DC_PIDMOTOR.getCurrentPosition());
  if (lll!=DC_STEPPERMOTOR.getCurrentPosition())
  {
    DC_STEPPERMOTOR.TurnOn ();
    DC_STEPPERMOTOR.setTargetPosition (lll, true);
  }
  DC_STEPPERMOTOR.Execute ();
  DC_PIDMOTOR.Execute();

  if (Serial.available()) {
    c=Serial.read();
    switch (c)
    {
      case '$': dFootX=40; dFootY=30;
                break;
      case '%': dFootX=40; dFootY=-30;
                break;
      case 'q': Serial.print (DC_PIDMOTOR.PIDParameter.lIntegral); Serial.print (' '); 
                Serial.print (DC_PIDMOTOR.PIDParameter.fsampleTime*1000); Serial.print (' ');
                break;
      case 'b': dFootX=dFootX-10;
                break;
      case 'n': dFootX=dFootX+10;
                break;
      case 'j': dFootY=dFootY+10;
                break;
      case 'm': dFootY=dFootY-10;
                break;
      case 's':
                DC_STEPPERMOTOR.setCurrentPosition (0);
                DC_STEPPERMOTOR.setTargetPosition (4,false);
                DC_STEPPERMOTOR.TurnOn();
                delay (500);
                DC_STEPPERMOTOR.setTargetPosition (0,false);
                delay (5000);
                DC_STEPPERMOTOR.TurnOff ();
                break;
      case 'I':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I()+0.01,DC_PIDMOTOR.getPID_D());
                bShowPID=true;
                break;
      case 'i':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I()-0.01,DC_PIDMOTOR.getPID_D());
                bShowPID=true;
                break;      
      case 'P':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P()+0.1,DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D());
                bShowPID=true;
                break;      
      case 'p':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P()-0.1,DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D());
                bShowPID=true;
                break;      
      case 'D':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D()+0.01);
                bShowPID=true;
                break;      
      case 'd':
                DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D()-0.01);
                bShowPID=true;
                break;      
      case '0': DC_PIDMOTOR.TurnOff();
                DC_STEPPERMOTOR.TurnOff();
                bShowPID=true;
                break;      
      case '1': 
                DC_PIDMOTOR.setTargetPosition (10);
                DC_PIDMOTOR.TurnOn ();
                break;
      case '2':
                DC_PIDMOTOR.setTargetPosition (20);
                DC_PIDMOTOR.TurnOn ();
                break;
      case '3':
                DC_PIDMOTOR.setTargetPosition (30);
                DC_PIDMOTOR.TurnOn ();
                break;
      case '5': 
                DC_PIDMOTOR.setTargetPosition (300);
                DC_PIDMOTOR.TurnOn ();
                break;
      case 'e':
                if (DC_PIDMOTOR.loopCheckEncoderInverted ()==true)
                  Serial.println ("Encoder inverted.");
                else
                  Serial.println ("Encoder not inverted.");
                break;
      case 't':
                DC_PIDMOTOR.loopAutoTune();
                bShowPID=true;
                break;
    }
    if (bShowPID==true)
    {
      bShowPID=false;
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D()); 
      Serial.print("H: "); Serial.println(DC_STEPPERMOTOR.getCurrentPosition());     
      Serial.print("R: "); Serial.println(DC_PIDMOTOR.getCurrentPosition());     
    }
  }
  
  lEP = DC_PIDMOTOR.getCurrentPosition ();  
  dEP = lEP;
  iServoLeftPosition = 1500;
  dServoRightPosition = 1500 + (  ((180.0/600.0) * dEP) * (600.0/90.0) );
  iServoRightPosition = dServoRightPosition;
  if (iServoRightPosition>2100) iServoRightPosition=2100;
  if (iServoRightPosition<900) iServoRightPosition=900;  
  if (iServoLeftPosition>2100) iServoLeftPosition=2100;
  if (iServoLeftPosition<900) iServoLeftPosition=900;  
  sServoLeft.writeMicroseconds (iServoLeftPosition);
  sServoRight.writeMicroseconds (iServoRightPosition+81);
  
  if (lEP!=lpEP)
  {
    DC_PIDMOTOR.getCurrentPosition ();
    Serial.print (DC_PIDMOTOR.getCurrentPosition ()); Serial.print ("\t"); // Primary Axis ist
    Serial.print (DC_STEPPERMOTOR.getCurrentPosition ()); // Secondary Axis ist
    
    Serial.print(", Stepper Current="); Serial.print(DC_STEPPERMOTOR.getCurrentPosition());     
    Serial.print(", Target="); Serial.print (DC_STEPPERMOTOR.getTargetPosition());
    Serial.print(", Motor Current="); Serial.print(DC_PIDMOTOR.getCurrentPosition());     
    Serial.print(", Target="); Serial.print (DC_PIDMOTOR.getTargetPosition());
    Serial.print(", dFootX="); Serial.print (dFootX);
    Serial.print(", dFootY="); Serial.println (dFootY);
    lpEP=lEP;
  }
}
