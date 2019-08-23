#include <avrlibdefs.h>
#include <nI2C.h>
#include <nTWI.h>
#include <queue.h>
#include <Servo.h>

/* ****************************************** D I V E R S E - K O N S T A N T E N *******************************************
   
  ************************************************************************************************************************** */
typedef struct
{
  bool PIN_STEPPER_COIL1_PLUS;
  bool PIN_STEPPER_COIL2_PLUS;
  bool PIN_STEPPER_COIL1_MINUS;
  bool PIN_STEPPER_COIL2_MINUS;
} tStepperPins;
/*
//                                          PIN_STEPPER_COIL1_PLUS, PIN_STEPPER_COIL2_PLUS, PIN_STEPPER_COIL1_MINUS,  PIN_STEPPER_COIL2_MINUS
const tStepperPins HorizontalAxisSteps[] = {{LOW,                   LOW,                    LOW,                      HIGH},
                                            {LOW,                   LOW,                    HIGH,                     LOW},
                                            {LOW,                   HIGH,                   LOW,                      LOW},
                                            {HIGH,                  LOW,                    LOW,                      LOW}};
*/                                    
//                                          PIN_STEPPER_COIL1_PLUS, PIN_STEPPER_COIL2_PLUS, PIN_STEPPER_COIL1_MINUS,  PIN_STEPPER_COIL2_MINUS
const tStepperPins HorizontalAxisSteps[] = {{HIGH,                  LOW,                    LOW,                      LOW},
                                            {LOW,                   HIGH,                   LOW,                      LOW},
                                            {LOW,                   LOW,                    HIGH,                     LOW},
                                            {LOW,                   LOW,                    LOW,                      HIGH}};

// Position of the GY271 sensors on each footplate:
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) };
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 };

const char MOTOR_BREAK_SPEED         = 110;
const char MOTOR_LOW_SPEED           = 0;
const char MOTOR_HIGH_SPEED          = 80;             // Der Motor darf maximal 6V erhalten
                                                       // Es liegen am Motortreiber "L298N" 12V extern an. Er besitzt 1,4V Verlust
                                                       // D.h. ein 
                                                       // PWM mit   0 = 0V
                                                       // PWM mit 144 = 5,986V
                                                       // PWM mit 255 = 10,6V (12V - 1,4 Verlust)

/* ********************************************** P I N - A S S I G N M E N T ***********************************************
   ************************************************************************************************************************** */
const char PIN_USB_RX                = 0;   // USB BUS RX
const char PIN_USB_TX                = 1;   // USB BUS TX
const char PIN_BASEPLATE_ENCODER_A   = 2;   // INT     Plattform Encoder: INT Input
const char PIN_BASEPLATE_ENCODER_B   = 3;   // PWM,INT Plattform Encoder: INT Input
const char PIN_STEPPER_COIL2_PLUS    = 4;   //         Stepper Driver for wire pair: Green/Black (IN2), The stepper is always enabled
const char PIN_RESERVE_1             = 5;   // PWM
const char PIN_BASEPLATE_MOTOR_ENABLE= 6;   // PWM     Plattform Motor: PWM Signal für Geschwindigkeit (Achtung: der Motor verträgt 6V)
const char PIN_BASEPLATE_MOTOR1_PLUS = 7;   //         Plattform Motor: Richtung Plus/Clockwise
const char PIN_BASEPLATE_MOTOR1_MINUS= 8;   //         Plattform Motor: Richtung Minus/Counter clockwise
const char PIN_FOOTPRINT_RIGHT_SERVO = 9;   // PWM     Trittplatte links: Servo position links, 1ms..2ms=-90°..+90°
const char PIN_FOOTPRINT_LEFT_SERVO  = 10;  // PWM     Trittplatte rechts: Servo position rechts, 1ms..2ms=-90°..+90°
const char PIN_RESERVE_2             = 11;  // PWM
const char PIN_STEPPER_COIL1_PLUS    = 12;  //         Stepper Driver for wire pair: red/blue (IN4), The stepper is always enabled
const char PIN_STEPPER_COIL1_MINUS   = 13;  //         Stepper Driver for wire pair: red/blue (IN3), The stepper is always enabled
const char PIN_REFERENCE_SWITCH      = A0;  // A0      
const char PIN_STEPPER_COIL2_MINUS   = A1;  // A1      Stepper Driver for wire pair: Green/Black (IN1), The stepper is always enabled
const char PIN_BASEPLATE_MOTOR2_PLUS = A2;  // A2
const char PIN_BASEPLATE_MOTOR2_MINUS= A3;  // A3
const char PIN_I2C_BUS_SDA           = A4;  // A4      I2C BUS: SDA
const char PIN_I2C_BUS_SCL           = A5;  // A5      I2C BUS: SCL

/* ****************************************************** T Y P E S *********************************************************
   ************************************************************************************************************************** */
typedef struct {
  double X;
  double Y;
} tPosition;

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
  byte Range;
  byte Status;
} tVL6180X;

typedef struct {
  int CurrentPosition;
  int DestinationPosition;  
} tStepper;

/* tFootprint
   ~~~~~~~~~~
   Diese Struktur enthält alle Parameter einer Trittplatte.
   Siehe Dokument "VRTM - Crosswalk - Formeln um einen Fuß zu folgen"
*/
typedef struct {
  long fx;                  // X Position der Trittplatte im Raum
  long fy;                  // Y Position der Trittplatte im Raum
  long b;                   // gewünschter Abstand der Trittplatte zum Drehzentrum
  float deltaAlpha;         // gewünschter Delta-Dreh-Winkel der Plattform
  float alpha;              // gewünschter Dreh-Winkel der Plattform
  float deltaBeta;          // gewünschter Delta-Dreh-Winkel der Plattform
  int   ServoMicroseconds;  // gewünschter Dreh-Winkel der Trittplattform in Servo-gerechter Zahl (1000 ... 2000 = -90° ... +90°) 
  float beta;               // gewünschter Dreh-Winkel der Trittplattform
  Servo sServo;             // Ist der Motor, der die Trittplatte dreht
  tGY271 GY271[6];          // Sensorvalues from all of the GY271 sensors
  tVL6180X VL6180X[6];      // Sensorvalues from all of the VL6180X sensors
  tPosition Center;         // The current center of the foot above the footplate
  bool CenterValid;
  CI2C::Handle I2C_TCA9548A_Upper;
  CI2C::Handle I2C_TCA9548A_Lower;
} tFootprint;

/* tBasePlate
   Diese Struktur enthält alle Parameter die für die Plattform
   notwendig sind.
 */
typedef struct {
  volatile int iCurrentPosition;      // aktueller Pulszähler der Plattform. Eine Umdrehung sind 600x4 = 2400 Pulse. Gezählt wird von 0...1199 Pulse. Das entspricht 0..359 Grad
  volatile char cCurrentState;        // aktueller "State" von PIN A und PIN B des Encoders. Wird f�r den Interrupt benötigt
  volatile int iDestinationPosition;  // Zielposition der Plattform. Die Motoren werden die Plattform in Position bringen.
  volatile int iPreviousDestinationPosition;
  volatile tStepper HorizontalAxis;
  volatile long lCurrentSpeed;
  volatile long lPreviousSpeed;
  volatile byte bMoving;
} tBasePlate;

/* *********************************************  G L O B A L    V A R I A B L E S ******************************************
   ************************************************************************************************************************** */
volatile byte PID_KI=0;

volatile byte bPWM;
byte bPreviousPWM;
long lPreviousSpeed;
byte HorizontalAxisIndex;
byte bSTOP_CurrentTCA9548A;
byte bSTOP_CurrentTCA9548A_Port;

// I2C Send and receive buffer:
byte    cGY271_ReceiveBuffer[10]={0};
byte    cTCA9548A_SendBuffer[]={0,1,2,4,8,16,32,64,128};
byte    cZero[1]={0};
byte bInterrupt;
tGY271 *pGY271_Callback_Sensor;

tBasePlate BasePlate;
tFootprint FootprintLeft;
tFootprint FootprintRight;
double fDistanceFactor;
CI2C::Handle I2C_GY271;
CI2C::Handle I2C_VL6180X;

unsigned long ulStart;
unsigned long ulStop;
unsigned long ulCycles;
/* ***************************************************  B A S E P L A T E  **************************************************
   ************************************************************************************************************************** 
   Dieser Abschnitt enthält alle Funktionen für die sich drehende Plattform.
   Auch mit dabei ist die Ansteuerung der Motoren um die Plattform zu drehen.
   Damit sich die Plattform dreht, und die Motoren angesteuert werden, ist zunächst
   die Funktion "fBasePlate_Init" aufzurufen. Anschließend ist lediglich
   die folgende Variable auf die anzufahrende Zielposition zu setzen:

   BasePlate.iDestinationPosition = 0..1199, wobei dies 0..359° entspricht

   Soll die Plattform langsam anfahren und auch bremsen kurz vor der Zielposition,
   so ist diese Variable zu setzen: ACC_BASEPLASTE_DISTANCE

   Wenn die zu fahrende Strecke der Plattform kleiner ist als die Anzahl der Pulse (0..1199)
   in ACC_BASEPLASTE_DISTANCE, so werden die Motoren per PWM langsam gestartet bzw. abgebremst.

   Ist die zurück zu legende Strecke größer als ACC_BASEPLASTE_DISTANCE, so wird gleich
   von Begin an die Motoren zu 100% angesteuert.

   Ist kein Abbremsen oder langsames Beschleunigen gewünscht, so ist die Konstante
   ACC_BASEPLASTE_DISTANCE auf 0 zu setzen.
  
 */

/* Function: fBasePlate_Move
   Description:
    Diese Funktion dreht die eigentliche Plattform. Sie wird von 
    der Interrupt Funktion fBasePlate_Interrupt aufgerufen.
    Die Motoren werden so angesteuert, dass die gewünschte 
    Zielposition "iDestinationPosition" automatisch angefahren wird.
    Diese Funktion wird vom Interrupt des Encoders automatisch 
    aufgerufen. Siehe Funktion: "fBasePlate_Interrupt"
    Es ist also nicht notwendig, diese Funktion manuell/direkt 
    aufzurufen.

   PARAMETERS:  none
   USES:        BasePlate.iDestinationPosition
                BasePlate.iCurrentPosition
   MODIFYS:     PIN_BASEPLATE_MOTOR_MINUS
                PIN_BASEPLATE_MOTOR_PLUS
   DURATION:    ?
   USED BY:     fBasePlate_Interrupt     
   
 */

void fBasePlate_Move () {
  int iDeltaMinus;
  int iDeltaPlus;
  if ((BasePlate.iDestinationPosition!=BasePlate.iCurrentPosition) && (BasePlate.iPreviousDestinationPosition!=BasePlate.iDestinationPosition)) { 
    // die Achse muss sich in Bewegung setzen
    // Zunächst ermitteln, ob sich diese links oder rechtsherum drehen muss.
    // Problem: es gibt einen Überlauf von 1199 auf 0 und umgekehrt.
    if (BasePlate.iDestinationPosition > BasePlate.iCurrentPosition) { 
      iDeltaPlus = BasePlate.iDestinationPosition - BasePlate.iCurrentPosition;
      iDeltaMinus = BasePlate.iCurrentPosition + (1199 - BasePlate.iDestinationPosition);
    }
    else {
      iDeltaMinus = BasePlate.iCurrentPosition - BasePlate.iDestinationPosition;
      iDeltaPlus = BasePlate.iDestinationPosition + (1199 - BasePlate.iCurrentPosition);
    }
    if (iDeltaPlus<iDeltaMinus) { 
      // die Plattform muss sich rechts-herum drehen
      bPWM=PID_KI;
      digitalWrite (PIN_BASEPLATE_MOTOR1_MINUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR1_PLUS, HIGH);
      digitalWrite (PIN_BASEPLATE_MOTOR2_MINUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR2_PLUS, HIGH);
      analogWrite (PIN_BASEPLATE_MOTOR_ENABLE, bPWM);
      BasePlate.bMoving=1;
      return;
    }
    else 
    if (iDeltaMinus<iDeltaPlus) {
      // die Plattform muss sich links-herum drehen
      bPWM=PID_KI;
      digitalWrite (PIN_BASEPLATE_MOTOR1_PLUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR1_MINUS, HIGH);
      digitalWrite (PIN_BASEPLATE_MOTOR2_PLUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR2_MINUS, HIGH);
      analogWrite (PIN_BASEPLATE_MOTOR_ENABLE, bPWM);
      BasePlate.bMoving=1;
      return;
    }
  }
  // Plattform ist bereits in Position, also die Bremse betätigen:
  BasePlate.bMoving=0;
  BasePlate.iPreviousDestinationPosition = BasePlate.iDestinationPosition;
  digitalWrite (PIN_BASEPLATE_MOTOR1_MINUS, HIGH);
  digitalWrite (PIN_BASEPLATE_MOTOR1_PLUS, HIGH);
  digitalWrite (PIN_BASEPLATE_MOTOR2_MINUS, HIGH);
  digitalWrite (PIN_BASEPLATE_MOTOR2_PLUS, HIGH);
  analogWrite (PIN_BASEPLATE_MOTOR_ENABLE, MOTOR_BREAK_SPEED);      
}


/* Function: fBasePlate_GetCurrentState
   Description:
     Diese Funktion ermittelt den aktuellen Zustand (1..4) der Plattform
     bzw. in welcher der vier Stati sich die Plattform befindet.
     Wird später der vorherige "State" mit dem aktuellen "State"
     verglichen, kann die Drehrichtung der Plattform ermittelt werden.
     
   PARAMETERS:  none
   USES:        PIN_BASEPLATE_ENCODER_A
                PIN_BASEPLATE_ENCODER_B
   MODIFYS:     BasePlate.cCurrentState
   DURATION:    ?
   USED BY:     fBasePlate_Interrupt
                fBasePlate_Init
 
 */
void fBasePlate_GetCurrentState () {
  bool Channel_A;
  bool Channel_B;
  Channel_A = digitalRead (PIN_BASEPLATE_ENCODER_A);
  Channel_B = digitalRead (PIN_BASEPLATE_ENCODER_B);
  if (Channel_A == HIGH) {
    if (Channel_B == LOW)
      BasePlate.cCurrentState = 1;  // State=1: PINA = 1, PINB = 0
    else
      BasePlate.cCurrentState = 2;  // State=2: PINA = 1, PINB = 1
  }
  else { // Channel_A==LOW
    if (Channel_B == HIGH)
      BasePlate.cCurrentState = 3;  // State=3: PINA = 0, PINB = 1
    else
      BasePlate.cCurrentState = 4;  // State=4: PINA = 0, PINB = 0
  }  
}

/* Function: fBasePlate_Interrupt
   Description:
    Wird vom Encoder aufgerufen wenn sich dieser dreht.
    Daraus wird die aktuelle IST Position der Platforrm
    ermittelt.
    Es ändert sich entweder der Zustand von PIN A oder B.
    Sobald sich die Spannung an einem der beiden PINs
    ändert, wird diese Funktion aufgerufen.
    Bei jeden Interrupts kann sich "State" wie folgt
    ändern:
      STATE: 1 2 3 4 1 2 3 4
      ======================
      PIN A: 1 1 0 0 1 1 0 0
      PIN B: 0 1 1 0 0 1 1 0
    Beispiel:
    Ist zum Zeitpunkt des Interrupts der PINA=1 und
    PINB=1, so handelt es sich um "State=2".
    Verglichen mit dem vorherigen State, kann
    ermittelt werden ob sich der Encoder links oder
    rechts herum dreht.
    Gezählt wird von 0..1199 was 0..359 Grad entspricht.
    Dieser Interrupt ruft auch die Funktion
    fBasePlate_MotorMove auf, um sofort die 
    Motoren entsprechend zu korrigieren.
   
   PARAMETERS:  none
   USES:        BasePlate.cCurrentState
                fBasePlate_Move
                fBasePlate_GetCurrentState
   MODIFYS:     BasePlate.iCurrentPosition
   DURATION:    ?
   USED BY:     PIN_BASEPLATE_ENCODER_A
                PIN_BASEPLATE_ENCODER_B
*/
void fBasePlate_Interrupt () {
  char cPreviousState;
  
  BasePlate.lCurrentSpeed++;
  
  cPreviousState = BasePlate.cCurrentState;
  fBasePlate_GetCurrentState ();
  if (cPreviousState < BasePlate.cCurrentState) {
    // Encoder dreht sich links herum
    if (BasePlate.iCurrentPosition > 0)
      BasePlate.iCurrentPosition--;         // Encoder dreht sich links
    else
      BasePlate.iCurrentPosition = 1199;    // Überlauf, zurück zu 1199 da es in den Minusbereich nicht gehen darf
  }
  else { 
    // Encoder dreht sich (hoffentlich) rechts herum
    if (BasePlate.iCurrentPosition < 1199)
      BasePlate.iCurrentPosition++;         // Encoder dreht sich rechts
    else
      BasePlate.iCurrentPosition = 0;       // Überlauf, zurück zu 0, da hier nicht höher als 1199 gezählt werden darf
  }
  fBasePlate_Move ();                       // die Motoren ansteuern
}
 
/* Function: fBasePlate_Init
   Description:
    Diese Funktion ist aufzurufen damit der Encoder initialisiert wird.
    Hier werden die PINs als Eingang deklariert, der aktuelle "State"
    des Encoders ermittelt als auch die Interrupts des Encoders aktiviert.

   PARAMETERS:  none
   USES:        PIN_BASEPLATE_ENCODER_A
                PIN_BASEPLATE_ENCODER_B
                fBasePlate_GetCurrentState
   MODIFYS:     PIN_BASEPLATE_ENCODER_A
                PIN_BASEPLATE_ENCODER_B
                PIN_BASEPLATE_MOTOR_PLUS
                PIN_BASEPLATE_MOTOR_MINUS
                BasePlate.iCurrentPosition
                BasePlate.iDestinationPosition
   DURATION:    ?
   USED BY:     setup
*/
void fBasePlate_Init () {
  Serial.println ("fBasePlate_Init");
  pinMode (PIN_BASEPLATE_ENCODER_A, INPUT_PULLUP);
  pinMode (PIN_BASEPLATE_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_BASEPLATE_MOTOR1_PLUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR1_MINUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR2_PLUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR2_MINUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR_ENABLE, OUTPUT);
  pinMode (PIN_REFERENCE_SWITCH, INPUT_PULLUP);
  digitalWrite (PIN_BASEPLATE_MOTOR1_PLUS, LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR1_MINUS, LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR2_PLUS, LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR2_MINUS, LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR_ENABLE, LOW);
  analogWrite (PIN_BASEPLATE_MOTOR_ENABLE, 0);
  pinMode (PIN_STEPPER_COIL1_PLUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL1_MINUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL2_PLUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL2_MINUS, OUTPUT);
  digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
  digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
  digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
  digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);
  fBasePlate_GetCurrentState ();
  BasePlate.HorizontalAxis.CurrentPosition=600;
  BasePlate.iCurrentPosition = 600;
  BasePlate.iDestinationPosition = 600;
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), fBasePlate_Interrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), fBasePlate_Interrupt, CHANGE);
}

/* ***************************************************  F O O T P R I N T *************************************************** 
   **************************************************************************************************************************
   Dieser Abschnitt enthält alle Funktionen die zur Ansteuerung einer Trittplatte notwendig sind.
 */


boolean fVL6180X_Synchron_ReadRegister8  (unsigned int uiRegister, byte *bValue) {
/*
  Wire.beginTransmission (0x29);
  Wire.write (uiRegister>>8);
  Wire.write (uiRegister);
  if (fI2C_CheckForError(Wire.endTransmission ())==false) 
  {
    Serial.print ("fVL6180X_Synchron_ReadRegister8, Wire.endTransmission failed: uiRegister=0x");
    Serial.println (uiRegister,HEX);
    fSTOP ();
    return false;
  }
  if (Wire.requestFrom (0x29,1)==1)
  {
    *bValue = Wire.read ();
    return true;
  }
  Serial.print ("fVL6180X_Synchron_ReadRegister8, Wire.endTransmission failed: uiRegister=0x");
  Serial.println (uiRegister,HEX);
  fSTOP ();
  */
  return false;
}

unsigned int fVL6180X_Synchron_ReadRegister16  (int bI2CAddress, unsigned int uiRegister) {
  unsigned int uiValue;
/*
  Wire.beginTransmission (bI2CAddress);
  Wire.write (uiRegister>>8);
  Wire.write (uiRegister);
  fI2C_CheckForError(Wire.endTransmission ());
  Wire.requestFrom (bI2CAddress,2);
  uiValue   = Wire.read ();
  uiValue <<= 8;
  uiValue  |= Wire.read();
  */
  return (uiValue);
}

boolean fGY271_Synchron_WriteRegister8 (byte uiRegister, byte bValue) {
/*
  Wire.beginTransmission (0x0D);
  Wire.write (uiRegister);
  Wire.write (bValue);
  if (fI2C_CheckForError(Wire.endTransmission ())==false)
   {
    Serial.print ("fGY271_Synchron_WriteRegister8, Wire.endTransmission failed, uiRegister=0x");
    Serial.print (uiRegister,HEX);
    Serial.print (", bValue=0x");
    Serial.println (bValue,HEX);
    return false;
  }
  */
  return true; 
}

void fVL6180X_Write (int uiRegister, byte bValue) {
  byte bStatus;
  if (bStatus=nI2C->Write (I2C_VL6180X, uiRegister, &bValue,1)!=0) fSTOP_I2C (I2C_VL6180X.device_address,bStatus,"fVL6180X_Write");
}

void fGY271_Write (byte bRegister, byte bValue) {
  byte bStatus;
  if (bStatus=nI2C->Write (I2C_GY271, bRegister, &bValue, (uint32_t)1)!=0) fSTOP_I2C (I2C_GY271.device_address,bStatus,"fGY271_Write");
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

bool fVL6180X_Synchron_ReadRanges () {
  
  // RIGHT FOOTPRINT:
  // ~~~~~~~~~~~~~~~~
  /*
  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 1)==false) return false; // select right footprint, upper, VL6180X channel #1
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[0])==false) return false;      // read range
  
  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 2)==false) return false; // select right footprint, upper, VL6180X channel #2
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[1])==false) return false;      // read range

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 5)==false) return false; // select right footprint, upper, VL6180X channel #5
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[2])==false) return false;      // read range

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 1)==false) return false; // select right footprint, lower, VL6180X channel #1
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[3])==false) return false;      // read range
  
  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 2)==false) return false; // select right footprint, lower, VL6180X channel #2
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[4])==false) return false;      // read range

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 5)==false) return false; // select right footprint, lower, VL6180X channel #5
  if (fVL6180X_Synchron_ReadRange (&FootprintRight.VL6180X[5])==false) return false;      // read range
  */
  return true;
}

bool fVL6180X_Synchron_ReadRange (tVL6180X *VL6180X) {
  byte bValue=0;
  while ((bValue & 1) != 1)
  {
    if (fVL6180X_Synchron_ReadRegister8 (0x04d, &bValue)==false) return false;      // VL6180X_REG_RESULT_RANGE_STATUS: wait for device to be ready for range measurement
  }
  if (bValue & 0x01)
  {
    bValue=0;
    fVL6180X_Write (0x18, 0x01);         // VL6180X_REG_SYSRANGE_START: start a range measurement
    while ((bValue & 4) !=4)
    {
      if (fVL6180X_Synchron_ReadRegister8 (0x04f, &bValue)==false) return false;    // VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO
    }
    if (bValue & 0x04)
    {
      if (fVL6180X_Synchron_ReadRegister8 (0x062, &bValue)==false) return false;    // VL6180X_REG_RESULT_RANGE_VAL read range in mm
      VL6180X->Range = bValue;
      fVL6180X_Write (0x015, 0x07);      // VL6180X_REG_SYSTEM_INTERRUPT_CLEAR
      // Read Status:
      if (fVL6180X_Synchron_ReadRegister8 (0x04d, &bValue)==false) return false;    // VL6180X_REG_RESULT_RANGE_STATUS read status
      VL6180X->Status=bValue>>4;
      if (bValue == 0)                                                              // No Error? 
      {
        /*
          #define VL6180X_ERROR_NONE         0   ///< Success!
          #define VL6180X_ERROR_SYSERR_1     1   ///< System error
          #define VL6180X_ERROR_SYSERR_5     5   ///< Sysem error
          #define VL6180X_ERROR_ECEFAIL      6   ///< Early convergence estimate fail
          #define VL6180X_ERROR_NOCONVERGE   7   ///< No target detected
          #define VL6180X_ERROR_RANGEIGNORE  8   ///< Ignore threshold check failed
          #define VL6180X_ERROR_SNR          11  ///< Ambient conditions too high
          #define VL6180X_ERROR_RAWUFLOW     12  ///< Raw range algo underflow
          #define VL6180X_ERROR_RAWOFLOW     13  ///< Raw range algo overflow
          #define VL6180X_ERROR_RANGEUFLOW   14  ///< Raw range algo underflow
          #define VL6180X_ERROR_RANGEOFLOW   15  ///< Raw range algo overflow
          
          1-5 System error System error detected (can only happen on power on). No measurement possible.
          6   Early convergence estimate ECE check failed
          7   Max convergence System did not converge before the specified max. convergence time limit
          8   Range ignore Ignore threshold check failed
          9-10 Not used -
          11  Signal to Noise (SNR) Ambient conditions too high. Measurement not valid
          12/14 Range underflow, Range value < 0
                If the target is very close (0-10mm) and the offset
                is not correctly calibrated it could lead to a small
                negative value
          13/15 Range overflow
                Range value out of range. This occurs when the
                target is detected by the device but is placed at a
                high distance (> 200mm) resulting in internal
                variable overflow.
          16  Ranging_Filtered(1)
              Distance filtered by Wrap Around Filter (WAF).
              Occurs when a high reflectance target is
              detected between 600mm to 1.2m
          17 Not used -
          18 Data_Not_Ready
             Error returned by VL6180x_RangeGetMeasurementIfReady() when ranging data is not ready.
        */
        return true;
      }
    }
  }
  return true; // xxx
}

void fTCA9548A_Disable_I2C_BusDevice (CI2C::Handle bModule) {
  byte bStatus;
  bSTOP_CurrentTCA9548A = bModule.device_address;
  bSTOP_CurrentTCA9548A_Port=0;
  if (bStatus=nI2C->Write (bModule, &cTCA9548A_SendBuffer[0], 1)!=0) fSTOP_I2C (bModule.device_address,bStatus,"fTCA9548A_Disable_I2C_BusDevice");
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

void fGY271_Init (CI2C::Handle bModule, byte bPort) { 
  Serial.println ("fGY271_Init");
  fTCA9548A_Select_I2C_BusDevice (bModule, bPort);
  fGY271_Write(0x0A, 0x80);  // Soft Reset
  fGY271_Write(0x0B, 0x01);  // Define Set/Reset period
  //fGY271_Write(0x09, 0x1D);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
  fGY271_Write(0x09, 0x15);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 50Hz, set continuous measurement mode
  //fGY271_Write(0x09, 0x05);  // Define OSR = 512, Full Scale Range = 2 Gauss, ODR = 50Hz, set continuous measurement mode
}

void fVL6180X_Init (CI2C::Handle bModule, byte bPort) { 
  byte bValue;
  fTCA9548A_Select_I2C_BusDevice (bModule, bPort);
  fVL6180X_Write(0x0207, 0x01);
  fVL6180X_Write(0x0208, 0x01);
  fVL6180X_Write(0x0096, 0x00);
  fVL6180X_Write(0x0097, 0xfd);
  fVL6180X_Write(0x00e3, 0x00);
  fVL6180X_Write(0x00e4, 0x04);
  fVL6180X_Write(0x00e5, 0x02);
  fVL6180X_Write(0x00e6, 0x01);
  fVL6180X_Write(0x00e7, 0x03);
  fVL6180X_Write(0x00f5, 0x02);
  fVL6180X_Write(0x00d9, 0x05);
  fVL6180X_Write(0x00db, 0xce);
  fVL6180X_Write(0x00dc, 0x03);
  fVL6180X_Write(0x00dd, 0xf8);
  fVL6180X_Write(0x009f, 0x00);
  fVL6180X_Write(0x00a3, 0x3c);
  fVL6180X_Write(0x00b7, 0x00);
  fVL6180X_Write(0x00bb, 0x3c);
  fVL6180X_Write(0x00b2, 0x09);
  fVL6180X_Write(0x00ca, 0x09);
  fVL6180X_Write(0x0198, 0x01);
  fVL6180X_Write(0x01b0, 0x17);
  fVL6180X_Write(0x01ad, 0x00);
  fVL6180X_Write(0x00ff, 0x05);
  fVL6180X_Write(0x0100, 0x05);
  fVL6180X_Write(0x0199, 0x05);
  
  fVL6180X_Write(0x01a6, 0x1b);
  fVL6180X_Write(0x01ac, 0x3e);
  fVL6180X_Write(0x01a7, 0x1f);
  fVL6180X_Write(0x0030, 0x00);

  // Recommended : Public registers - See data sheet for more detail
  fVL6180X_Write(0x0011, 0x10);       // Enables polling for 'New Sample ready' when measurement completes
  fVL6180X_Write(0x010a, 0x30);       // Set the averaging sample period (compromise between lower noise and increased execution time)
  fVL6180X_Write(0x003f, 0x46);       // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
  fVL6180X_Write(0x0031, 0xFF);       // sets the # of range measurements after which auto calibration of system is performed
  fVL6180X_Write(0x0040, 0x63);       // Set ALS integration time to 100ms
  fVL6180X_Write(0x002e, 0x01);       // perform a single temperature calibration of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  fVL6180X_Write(0x001b, 0x09);       // Set default ranging inter-measurement period to 100ms
  fVL6180X_Write(0x003e, 0x31);       // Set default ALS inter-measurement period to 500ms
  fVL6180X_Write(0x0014, 0x24);       // Configures interrupt on 'New Sample Ready threshold event'

  fVL6180X_Write(0x0016, 0x00);       // VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET
}

/* Function: fFootprint_Calculate
   Description:
    Diese Funktion muss aufgerufen werden, um die neue Position einer
    Trittplatte im Raum zu berechnen. Hierzu werden die Formeln aus
    dem Dokument "VRTM - Crosswalk - Formeln um einen Fuß zu folgen"
    genutzt.
    Als Parameter ist der Zeiger auf eine Trittplatte zu übergeben.
    Innerhalb dieser Struktur muss DeltaX1, DeltaY1, DeltaX2, DeltaY2
    bereits gesetzt sein mit den Werten der zwei Kameras.
    Die Laufzeit dieser Funktion beträgt <528 Mikrosekunden
    (etwa 0,0005 Sekunden bei 16Mhz)

   PARAMETERS:  pointer to left or right "tFootprint"
   USES:        Footprint->x1
                Footprint->y1
                Footprint->DeltaX1
                Footprint->DeltaY1
                Footprint->x2
                Footprint->y2
                Footprint->DeltaX2
                Footprint->DeltaY2
   MODIFYS:     Footprint->x1
                Footprint->y1
                Footprint->x2
                Footprint->y2
                Footprint->fx
                Footprint->fy
                Footprint->b
                Footprint->deltaAlpha
                Footprint->alpha
                Footprint->deltaBeta
                Footprint->beta
   DURATION:    ?
   USED BY:     loop
*/
/*
void fFootprint_Calculate (tFootprint* Footprint) {
  float alpha;
  float beta;
  //Footprint->Camera1.x = Footprint->Camera1.x + Footprint->Camera1.Delta_X;
  //Footprint->Camera1.y = Footprint->Camera1.y + Footprint->Camera1.Delta_Y;
  //Footprint->Camera2.x = Footprint->Camera2.x - Footprint->Camera2.Delta_X;
  //Footprint->Camera2.y = Footprint->Camera2.y - Footprint->Camera2.Delta_Y;
  //Footprint->fx = (Footprint->Camera1.x + Footprint->Camera2.x) / 2;
  //Footprint->fy = (Footprint->Camera1.y + Footprint->Camera2.y) / 2;
  Footprint->b  = sqrt ( ( Footprint->fx * Footprint->fx ) + ( Footprint->fy * Footprint->fy ) );
  alpha = asin ( float(Footprint->fy) / float(Footprint->b) ) * (180 / PI);
  Footprint->deltaAlpha = alpha - Footprint->alpha;
  Footprint->alpha = alpha;
  //beta  = asin ( float( Footprint->Camera1.x - Footprint->Camera2.x ) / float(Footprint->m) ) * (180 / PI);
  Footprint->deltaBeta = beta - Footprint->beta;
  Footprint->beta = beta;
  Footprint->ServoMicroseconds = 1500 + ((500.0/90.0) * beta);
}
*/

/* Function: fFootprint_Init
   Description:
     Initialisiert die gesamte Trittplatte. */
void fFootprint_Init () {
  // Servos vorbereiten:
  FootprintLeft.sServo.attach (PIN_FOOTPRINT_LEFT_SERVO);
  FootprintRight.sServo.attach (PIN_FOOTPRINT_RIGHT_SERVO);
  FootprintLeft.sServo.writeMicroseconds (1500);
  FootprintRight.sServo.writeMicroseconds (1500);

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

void fSTOP (char *pMessage, char *pFunction) {
  Serial.println ("SYSTEM STOPPED BECAUSE OF INTERNAL ERROR.");
  Serial.print   ("Message : ");  Serial.println (pMessage);
  Serial.print   ("Function: ");  Serial.println (pFunction);
  while (1==1) 
  {
    delay(1);
    Serial.flush();
  };
}

/* fCalculate_Intersection
   ***********************
   Calculate the intersection of two lines.
   a1 = X Position of the Sensor #1
   b1 = Y Position of the Sensor #1
   a2 = X Position of the Sensor #2
   b2 = Y Position of the Sensor #2
   S1x= X Direction where the sensor #1 is pointing to
   S1y= Y Direction where the sensor #1 is pointing to
   S2x= X Direction where the sensor #2 is pointing to
   S2y= Y Direction where the sensor #2 is pointing to
   RESULT:
   *x = X coordinate where the two lines cross each other
   *y = y coordinate where the two lines cross each other
   return value: TRUE = The result is valid
                 FALSE = Ignore the result, error in calculation */
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

/* fCalculate_Distance
   ~~~~~~~~~~~~~~~~~~~
   Calculate the distance between two points */
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

void fReceiveCommandsFromPC () {
  char cCommandFromPC;
  // Look for commands from PC:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (Serial.available())
  {
    cCommandFromPC=Serial.read ();
    switch (cCommandFromPC)
    {
      case 's':
                digitalWrite (PIN_STEPPER_COIL1_PLUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL1_PLUS);
                digitalWrite (PIN_STEPPER_COIL2_PLUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL2_PLUS);
                digitalWrite (PIN_STEPPER_COIL1_MINUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL1_MINUS);
                digitalWrite (PIN_STEPPER_COIL2_MINUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL2_MINUS);
                delay (500);
                digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
                digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
                digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
                digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);
                break;
      case '0': //
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;
      case '5': BasePlate.iDestinationPosition=BasePlate.iDestinationPosition+1;
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;
      case '6': BasePlate.iDestinationPosition = 600;
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;
      case '7': BasePlate.iDestinationPosition = 615;
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;
      case '8': BasePlate.iDestinationPosition = 630;
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;
      case '9': BasePlate.iDestinationPosition = 660;
                Serial.print ("bPWM="); Serial.print (bPWM);
                Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
                Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
                Serial.print (", BasePlate.lCurrentSpeed="); Serial.println (BasePlate.lCurrentSpeed);
                break;

    }
  }
  
  if ((bPreviousPWM!=bPWM)||(lPreviousSpeed!=BasePlate.lCurrentSpeed)||(BasePlate.bMoving==1))
  {
    Serial.print ("bPWM="); Serial.print (bPWM);
    Serial.print (", PID_KI="); Serial.print (PID_KI);
    Serial.print (", BasePlate.iCurrentPosition="); Serial.print (BasePlate.iCurrentPosition);
    Serial.print (", BasePlate.iDestinationPosition="); Serial.print (BasePlate.iDestinationPosition);      
    Serial.print (", BasePlate.lPreviousSpeed="); Serial.println (BasePlate.lPreviousSpeed);
    bPreviousPWM=bPWM;
    lPreviousSpeed=BasePlate.lCurrentSpeed;
  }
}

void fSendDataToPC () {
  long CX;
  long CY;
  
  if (Serial.availableForWrite()>60)
  {
    Serial.print ("*");
    /*
    Serial.print (FootprintRight.VL6180X[0].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[0].Status); Serial.print ("\t"); 
    Serial.print (FootprintRight.VL6180X[1].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[1].Status); Serial.print ("\t");
    Serial.print (FootprintRight.VL6180X[2].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[2].Status); Serial.print ("\t");
    Serial.print (FootprintRight.VL6180X[3].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[3].Status); Serial.print ("\t");
    Serial.print (FootprintRight.VL6180X[4].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[4].Status); Serial.print ("\t");
    Serial.print (FootprintRight.VL6180X[5].Range); Serial.print ("\t"); Serial.print (FootprintRight.VL6180X[5].Status); Serial.print ("\t");
    Serial.print (ulCycles);
    Serial.println ("#");        
    */
    /*
    Serial.print (FootprintRight.GY271[0].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[0].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[0].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[1].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[1].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[1].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[2].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[2].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[2].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[3].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[3].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[3].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[4].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[4].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[4].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[5].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[5].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[5].Value.Z); Serial.print ("\t");
    CX = FootprintRight.Center.X;
    CY = FootprintRight.Center.Y;
    Serial.print (CX); Serial.print ("\t"); Serial.print (CY); Serial.print ("\t");
    Serial.print (BasePlate.iCurrentPosition); Serial.print (", ");
    Serial.print (JoystickX); Serial.print (", ");
    Serial.print (JoystickY); Serial.print (", ");
    Serial.print (JoystickB); Serial.print (", ");
    Serial.println ("#");  
   */
   }  
}


/* ***********************************************  M A I N P R O G R A M M  ************************************************
   ************************************************************************************************************************** */
void setup() {
  byte bStatus;
  // Serial stuff:
  Serial.begin(115000);
  while (!Serial) {                   // wait for serial port to open on native usb devices
    delay(1);
  }

  // I2C stuff:
  nI2C->SetTimeoutMS(25);
  FootprintLeft.I2C_TCA9548A_Upper  = nI2C->RegisterDevice(0x70, 1, CI2C::Speed::FAST);
  FootprintLeft.I2C_TCA9548A_Lower  = nI2C->RegisterDevice(0x71, 1, CI2C::Speed::FAST);
  FootprintRight.I2C_TCA9548A_Upper = nI2C->RegisterDevice(0x74, 1, CI2C::Speed::FAST);
  FootprintRight.I2C_TCA9548A_Lower = nI2C->RegisterDevice(0x75, 1, CI2C::Speed::FAST);
  I2C_GY271                         = nI2C->RegisterDevice(0x0D, 1, CI2C::Speed::FAST);
//  I2C_VL6180X                       = nI2C->RegisterDevice(0x29, 2, CI2C::Speed::FAST);
//  fTCA9548A_Disable_I2C_BusDevice (FootprintLeft.I2C_TCA9548A_Upper);
//  fTCA9548A_Disable_I2C_BusDevice (FootprintLeft.I2C_TCA9548A_Lower);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower);
  
  // Init other stuff:
  fFootprint_Init ();
  fBasePlate_Init ();

  // Millisecond timer interrupt
  OCR0A = 0x7D;
  TIMSK0 |= _BV(OCIE0A); 
  Serial.println ("Ready.");   
}

void loop() {
  
  // Read values from Sensors:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~
  //fGY271_Read_Values (); 

  //Serial.println (digitalRead (PIN_REFERENCE_SWITCH));
  //fSendDataToPC ();
  if (BasePlate.bMoving==0)
  {
    fBasePlate_Move ();  
  }
  fReceiveCommandsFromPC ();  
}

/* ***********************************************  I N T E R R U P T  ***********************************************
   ******************************************************************************************************************* */

// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect)  {  
  static int x;

  if (BasePlate.bMoving==1)
  {
    if (BasePlate.lCurrentSpeed<10)
    {
      if (PID_KI<MOTOR_HIGH_SPEED) PID_KI++;
    }
    if (BasePlate.lCurrentSpeed>20)
    {
      if (PID_KI>0) PID_KI--;
    }
    if (BasePlate.lCurrentSpeed!=0)
    {
      BasePlate.lPreviousSpeed=BasePlate.lCurrentSpeed;
    }
  }
  BasePlate.lCurrentSpeed=0;
  
  // STEPPER STUFF:
  // ~~~~~~~~~~~~~~
  if (digitalRead (PIN_REFERENCE_SWITCH)==HIGH) // Reference Switch active?
  { // Turn off motor
    digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);
    BasePlate.HorizontalAxis.CurrentPosition=0;    
  }
  x++;
  if (x<8) return;
  x=0;
  if (BasePlate.HorizontalAxis.CurrentPosition!=BasePlate.HorizontalAxis.DestinationPosition)
  {
    if (BasePlate.HorizontalAxis.DestinationPosition>BasePlate.HorizontalAxis.CurrentPosition)
    {
      BasePlate.HorizontalAxis.CurrentPosition++;
      if (HorizontalAxisIndex<3)
      {
        HorizontalAxisIndex++;
      }
      else
      {
        HorizontalAxisIndex=0;
      }
    }
    else
    {
      BasePlate.HorizontalAxis.CurrentPosition--;
      if (HorizontalAxisIndex>0)
      {
        HorizontalAxisIndex--;
      }
      else
      {
        HorizontalAxisIndex=3;
      }
    }
    digitalWrite (PIN_STEPPER_COIL1_PLUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL1_PLUS);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL2_PLUS);
    digitalWrite (PIN_STEPPER_COIL1_MINUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL1_MINUS);
    digitalWrite (PIN_STEPPER_COIL2_MINUS,HorizontalAxisSteps[HorizontalAxisIndex].PIN_STEPPER_COIL2_MINUS);
  }
  else
  { // Turn off motor
    digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);  
  }
}
