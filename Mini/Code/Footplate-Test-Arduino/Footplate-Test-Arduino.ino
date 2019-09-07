#include <Wire.h>
#include <Servo.h>

/* ****************************************** D I V E R S E - K O N S T A N T E N *******************************************
   ************************************************************************************************************************** */

// Position of the GY271 sensors on each footplate:
const double GY271_Sensor_X[] = {         -(40.64/2),          0.0,          (40.64/2),        -(40.64/2),          0.0,         (40.64/2) };
const double GY271_Sensor_Y[] = { -(143.149/2)+21.23, -(143.149/2), -(143.149/2)+21.23, (143.149/2)-22.86,  (143.149/2), (143.149/2)-22.86 };

const char MOTOR_LOW_SPEED           = 30;
const char MOTOR_HIGH_SPEED          = 70;             // Der Motor darf maximal 6V erhalten
                                                       // Es liegen am Motortreiber "L298N" 12V extern an. Er besitzt 1,4V Verlust
                                                       // D.h. ein 
                                                       // PWM mit   0 = 0V
                                                       // PWM mit 144 = 5,986V
                                                       // PWM mit 255 = 10,6V (12V - 1,4 Verlust)
const char LEFT_FOOTPRINT_I2C_UPPER  = 0x70;           // left footprint I2C address of the upper "TCA9548A I2C Multiplexer"
const char LEFT_FOOTPRINT_I2C_LOWER  = 0x71;           // left footprint I2C address of the lower "TCA9548A I2C Multiplexer"
const char RIGHT_FOOTPRINT_I2C_UPPER = 0x74;           // right footprint I2C address of the upper "TCA9548A I2C Multiplexer"
const char RIGHT_FOOTPRINT_I2C_LOWER = 0x75;           // right footprint I2C address of the lower "TCA9548A I2C Multiplexer"

/* ********************************************** P I N - A S S I G N M E N T ***********************************************
   ************************************************************************************************************************** */
const char PIN_USB_RX                = 0;   // USB BUS RX
const char PIN_USB_TX                = 1;   // USB BUS TX
const char PIN_BASEPLATE_ENCODER_A   = 2;   // INT     Plattform Encoder: INT Input
const char PIN_BASEPLATE_ENCODER_B   = 3;   // PWM,INT Plattform Encoder: INT Input
const char PIN_RESERVE_0             = 4;   // 
const char PIN_RESERVE_1             = 5;   // PWM
const char PIN_BASEPLATE_ENABLE      = 6;   // PWM     Plattform Motor: PWM Signal für Geschwindigkeit (Achtung: der Motor verträgt 6V)
const char PIN_BASEPLATE_MOTOR_PLUS  = 7;   //         Plattform Motor: Richtung Plus
const char PIN_BASEPLATE_MOTOR_MINUS = 8;   //         Plattform Motor: Richtung Minus
const char PIN_FOOTPRINT_LEFT_SERVO  = 9;   // PWM     Trittplatte links: Servo position links, 1ms..2ms=-90°..+90°
const char PIN_FOORPRINT_RIGHT_SERVO = 10;  // PWM     Trittplatte rechts: Servo position rechts, 1ms..2ms=-90°..+90°
const char PIN_STEPPER_COIL1_PLUS    = 11;  // PWM     Stepper IN4, The stepper is always enabled
const char PIN_STEPPER_COIL1_MINUS   = 12;  //         Stepper IN3, The stepper is always enabled
const char PIN_STEPPER_COIL2_PLUS    = 13;  //         Stepper IN2, The stepper is always enabled
const char PIN_STEPPER_COIL2_MINUS   = 14;  // A0      Stepper IN1, The stepper is always enabled
const char PIN_RESERVE_2             = 15;  // A1
const char PIN_RESERVE_3             = 16;  // A2
const char PIN_RESERVE_4             = 17;  // A3
const char PIN_I2C_BUS_SDA           = 18;  // A4      I2C BUS: SDA
const char PIN_I2C_BUS_SCL           = 19;  // A5      I2C BUS: SCL

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
  float Heading;
  int Strength;
} tGY271;

typedef struct {
  byte Range;
  byte Status;
} tVL6180X;


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
  tGY271 GY271[6];
  tVL6180X VL6180X[6];
  tPosition Center;
} tFootprint;

/* tBasePlate
   Diese Struktur enthält alle Parameter die für die Plattform
   notwendig sind.
 */
typedef struct {
  volatile int iCurrentPosition;      // aktueller Pulszähler der Plattform. Eine Umdrehung sind 600x4 = 2400 Pulse. Gezählt wird von 0...1199 Pulse. Das entspricht 0..359 Grad
  volatile char cCurrentState;        // aktueller "State" von PIN A und PIN B des Encoders. Wird f�r den Interrupt benötigt
  volatile int iDestinationPosition;  // Zielposition der Plattform. Die Motoren werden die Plattform in Position bringen.
} tBasePlate;

/* *********************************************  G L O B A L    V A R I A B L E S ******************************************
   ************************************************************************************************************************** */

volatile tBasePlate BasePlate;
volatile tFootprint FootprintLeft;
volatile tFootprint FootprintRight;
double fDistanceFactor;

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
  byte bPWM;
  bPWM = MOTOR_HIGH_SPEED;
  if (BasePlate.iDestinationPosition!=BasePlate.iCurrentPosition) { 
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
      analogWrite (PIN_BASEPLATE_ENABLE, bPWM);
      digitalWrite (PIN_BASEPLATE_MOTOR_MINUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR_PLUS, HIGH);
    }
    else { 
      // die Plattform muss sich links-herum drehen
      analogWrite (PIN_BASEPLATE_ENABLE, bPWM);
      digitalWrite (PIN_BASEPLATE_MOTOR_PLUS, LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR_MINUS, HIGH);
    }
  }
  else { 
    // Plattform ist bereits in Position, also die Bremse betätigen:
    digitalWrite (PIN_BASEPLATE_MOTOR_MINUS, LOW);
    digitalWrite (PIN_BASEPLATE_MOTOR_PLUS, LOW);
    analogWrite (PIN_BASEPLATE_ENABLE, 255);
  }
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
  bool a;
  bool b;
  a = digitalRead (PIN_BASEPLATE_ENCODER_A);
  b = digitalRead (PIN_BASEPLATE_ENCODER_B);
  if (a == HIGH) {
    if (b == LOW)
      BasePlate.cCurrentState = 1;  // State=1: PINA = 1, PINB = 0
    else
      BasePlate.cCurrentState = 2;  // State=2: PINA = 1, PINB = 1
  }
  else { // a==LOW
    if (b == HIGH)
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
boolean fBasePlate_Init () {
  pinMode (PIN_BASEPLATE_ENCODER_A, INPUT_PULLUP);
  pinMode (PIN_BASEPLATE_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_BASEPLATE_MOTOR_PLUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR_MINUS, OUTPUT);
  pinMode (PIN_BASEPLATE_ENABLE, OUTPUT);
  digitalWrite (PIN_BASEPLATE_MOTOR_PLUS, LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR_MINUS, LOW);
  digitalWrite (PIN_BASEPLATE_ENABLE, LOW);
  analogWrite (PIN_BASEPLATE_ENABLE, 0);
  fBasePlate_GetCurrentState ();
  BasePlate.iCurrentPosition = 0;
  BasePlate.iDestinationPosition = 0;
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), fBasePlate_Interrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), fBasePlate_Interrupt, CHANGE);
  return true;
}

/* ***************************************************  F O O T P R I N T *************************************************** 
   **************************************************************************************************************************
   Dieser Abschnitt enthält alle Funktionen die zur Ansteuerung einer Trittplatte notwendig sind.
 */

boolean fI2C_CheckForError (byte error) {
  if (error==0) return true;
  return false;
}

boolean fTCA9548A_Disable_I2C_BusDevice (byte bModule) {
  Wire.beginTransmission (bModule);
  Wire.write (0);
  if (fI2C_CheckForError(Wire.endTransmission ())==false) 
  {
    Serial.print ("Failed: fTCA9548A_Disable_I2C_BusDevice, bModule=0x");
    Serial.println (bModule,HEX);
    fSTOP ();
    return false;  
  }
  return true;
}

boolean fTCA9548A_Select_I2C_BusDevice (byte bModule, byte bPort) {
  static byte PreviousModule=0;
  if ((PreviousModule>0)&&(PreviousModule!=bModule))
  { // Disable the previous Module:
    if (fTCA9548A_Disable_I2C_BusDevice (PreviousModule)==false) 
    {
      Serial.print ("Failed: fTCA9548A_Select_I2C_BusDevice, bModule=0x");
      Serial.print (bModule,HEX);
      Serial.print (", bPort=0x");
      Serial.println (bPort,HEX);
      fSTOP();
      return false;
    }
  }
  PreviousModule = bModule;
  Wire.beginTransmission (bModule);
  Wire.write (1<<bPort);
  if (fI2C_CheckForError(Wire.endTransmission ())==false) 
  {
    Serial.print ("Failed: fTCA9548A_Select_I2C_BusDevice, bModule=0x");
    Serial.print (bModule,HEX);
    Serial.print (", bPort=0x");
    Serial.println (bPort,HEX);
    fSTOP();
    return false;
  }
  // Enable the correct channel on this TCA9548A
  Wire.beginTransmission (bModule);
  Wire.write (1<<bPort);
  if (fI2C_CheckForError(Wire.endTransmission ())==false) 
  {
    Serial.print ("Failed: fTCA9548A_Select_I2C_BusDevice, bModule=0x");
    Serial.print (bModule,HEX);
    Serial.print (", bPort=0x");
    Serial.println (bPort,HEX);
    fSTOP();
    return false;
  }
  return true;  
}

boolean fVL6180X_Synchron_ReadRegister8  (unsigned int uiRegister, byte *bValue) {
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
  return false;
}

unsigned int fVL6180X_Synchron_ReadRegister16  (int bI2CAddress, unsigned int uiRegister) {
  unsigned int uiValue;
  Wire.beginTransmission (bI2CAddress);
  Wire.write (uiRegister>>8);
  Wire.write (uiRegister);
  fI2C_CheckForError(Wire.endTransmission ());
  Wire.requestFrom (bI2CAddress,2);
  uiValue   = Wire.read ();
  uiValue <<= 8;
  uiValue  |= Wire.read();
  return (uiValue);
}

boolean fGY271_Synchron_WriteRegister8 (byte uiRegister, byte bValue) {
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
  return true; 
}

boolean fVL6180X_Synchron_WriteRegister8 (byte uiRegister, byte bValue) {
  Wire.beginTransmission (0x29);
  Wire.write (uiRegister>>8);
  Wire.write (uiRegister);
  Wire.write (bValue);
  if (fI2C_CheckForError(Wire.endTransmission ())==false)
  {
    Serial.print ("fVL6180X_Synchron_WriteRegister8, Wire.endTransmission failed, uiRegister=0x");
    Serial.print (uiRegister,HEX);
    Serial.print (", bValue=0x");
    Serial.println (bValue,HEX);
    fSTOP();
    return false;
  }
  return true;
}

bool fGY271_Synchron_Read_Heading (tGY271 *GY271) {
  int x, y, z;
  long lx, ly, lz, Strength;
  byte s;
  Wire.beginTransmission (0x0D); // 7-bit address of QMC5883L compass
  Wire.write (0x00);             // Look for values
  if (fI2C_CheckForError(Wire.endTransmission ())==false) return false;
  if (Wire.requestFrom (0x0D,7)==7)
  {
    x = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    y = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    z = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    s = Wire.read ();
    if (GY271->SetOffset)
    {
      GY271->Offset.X = x;
      GY271->Offset.Y = y;
      GY271->Offset.Z = z;
      GY271->SetOffset = 0;
    }
    lx = x - GY271->Offset.X;
    ly = y - GY271->Offset.Y;
    lz = z - GY271->Offset.Z;
    GY271->Value.X = lx;
    GY271->Value.Y = ly;
    GY271->Value.Z = lz;
    Strength = (lx*lx)+(ly*ly);
    GY271->Strength = sqrt (Strength);
    if (((s & 0x02)==0)&&((x!=0)||(y!=0))) { // no overflow?
      // If compass module lies flat on the ground with no tilt,
      // just x and y are needed for calculation
      GY271->Heading=atan2(x,y) * 180.0/PI;
      if(GY271->Heading < 0) GY271->Heading+=360; // N=0/360, E=90, S=180, W=270
      return true;
    }
    else
    {
      fRaiseError (7); // Overflow
    }
  }
  else
  {
    fRaiseError (6); // Insuffient data
  }
  return false;
}

bool fGY271_Synchron_Read_Headings () {
  float static Heading;

  // LEFT FOOTPRINT:
  // ~~~~~~~~~~~~~~~
  /*
  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_UPPER, 0)==false) return false;  // select left footprint, upper, GY271 channel #0
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[0])==false) return false;        // read heading
  
  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_UPPER, 3)==false) return false;  // select left footprint, upper, GY271 channel #3
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[1])==false) return false;        // read heading

  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_UPPER, 4)==false) return false;  // select left footprint, upper, GY271 channel #4
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[2])==false) return false;        // read heading

  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_LOWER, 0)==false) return false;  // select left footprint, lower, GY271 channel #0
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[3])==false) return false;        // read heading

  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_LOWER, 3)==false) return false;  // select left footprint, lower, GY271 channel #3
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[4])==false) return false;        // read heading

  if (fTCA9548A_Select_I2C_BusDevice (LEFT_FOOTPRINT_I2C_LOWER, 4)==false) return false;  // select left footprint, lower, GY271 channel #4
  if (fGY271_Synchron_Read_Heading (&FootprintLeft.GY271[5])==false) return false;        // read heading
*/
  // RIGHT FOOTPRINT:
  // ~~~~~~~~~~~~~~~~

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 0)==false) return false; // select right footprint, upper, GY271 channel #0
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[0])==false) return false;       // read heading

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 3)==false) return false; // select right footprint, upper, GY271 channel #3
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[1])==false) return false;       // read heading

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER, 4)==false) return false; // select right footprint, upper, GY271 channel #4
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[2])==false) return false;       // read heading

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 0)==false) return false; // select right footprint, lower, GY271 channel #0
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[3])==false) return false;       // read heading

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 3)==false) return false; // select right footprint, lower, GY271 channel #3
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[4])==false) return false;       // read heading

  if (fTCA9548A_Select_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER, 4)==false) return false; // select right footprint, lower, GY271 channel #4
  if (fGY271_Synchron_Read_Heading (&FootprintRight.GY271[5])==false) return false;       // read heading
  
  return true;
}

bool fVL6180X_Synchron_ReadRanges () {
  
  // RIGHT FOOTPRINT:
  // ~~~~~~~~~~~~~~~~
  
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
    if (fVL6180X_Synchron_WriteRegister8 (0x18, 0x01)==false) return false;         // VL6180X_REG_SYSRANGE_START: start a range measurement
    while ((bValue & 4) !=4)
    {
      if (fVL6180X_Synchron_ReadRegister8 (0x04f, &bValue)==false) return false;    // VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO
    }
    if (bValue & 0x04)
    {
      if (fVL6180X_Synchron_ReadRegister8 (0x062, &bValue)==false) return false;    // VL6180X_REG_RESULT_RANGE_VAL read range in mm
      VL6180X->Range = bValue;
      if (fVL6180X_Synchron_WriteRegister8 (0x015, 0x07)==false) return false;      // VL6180X_REG_SYSTEM_INTERRUPT_CLEAR
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

boolean fGY271_Init (byte bModule, byte bPort) { 
  if (fTCA9548A_Select_I2C_BusDevice (bModule, bPort)==false) return false;
  if (fGY271_Synchron_WriteRegister8 (0x0A, 0x80)==false) return false;  // Soft Reset
  if (fGY271_Synchron_WriteRegister8 (0x0B, 0x01)==false) return false;  // Define Set/Reset period
  //if (fGY271_Synchron_WriteRegister8 (0x09, 0x1D)==false) return false;  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
  if (fGY271_Synchron_WriteRegister8 (0x09, 0x15)==false) return false;  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 50Hz, set continuous measurement mode
  //if (fGY271_Synchron_WriteRegister8 (0x09, 0x05)==false) return false;  // Define OSR = 512, Full Scale Range = 2 Gauss, ODR = 50Hz, set continuous measurement mode
  return true;
}

bool fVL6180X_Init (byte bModule, byte bPort) { 
  byte bValue;
  if (fTCA9548A_Select_I2C_BusDevice (bModule, bPort)==false) return false;
  if (fVL6180X_Synchron_ReadRegister8(0x00,&bValue)==false) return false; // Read model type
  if (bValue!=0xB4)
  {
    Serial.print ("Failed: fVL6180X_Init. VL6180X wrong model type. bModule=0x");
    Serial.print (bModule, HEX);
    Serial.print (", bPort=0x");
    Serial.print (bPort,HEX);
    Serial.print (", VL6180X model type=0x");
    Serial.print (bValue,HEX);
    Serial.println (" (expected value=0xB4)");
    fSTOP ();
    return false;
  }
  if (fVL6180X_Synchron_WriteRegister8(0x0207, 0x01)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0208, 0x01)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0096, 0x00)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0097, 0xfd)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00e3, 0x00)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00e4, 0x04)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00e5, 0x02)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00e6, 0x01)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00e7, 0x03)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00f5, 0x02)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00d9, 0x05)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00db, 0xce)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00dc, 0x03)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00dd, 0xf8)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x009f, 0x00)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00a3, 0x3c)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00b7, 0x00)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00bb, 0x3c)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00b2, 0x09)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00ca, 0x09)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0198, 0x01)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x01b0, 0x17)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x01ad, 0x00)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x00ff, 0x05)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0100, 0x05)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0199, 0x05)==false) return false;
  
  if (fVL6180X_Synchron_WriteRegister8(0x01a6, 0x1b)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x01ac, 0x3e)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x01a7, 0x1f)==false) return false;
  if (fVL6180X_Synchron_WriteRegister8(0x0030, 0x00)==false) return false;

  // Recommended : Public registers - See data sheet for more detail
  if (fVL6180X_Synchron_WriteRegister8(0x0011, 0x10)==false) return false;       // Enables polling for 'New Sample ready' when measurement completes
  if (fVL6180X_Synchron_WriteRegister8(0x010a, 0x30)==false) return false;       // Set the averaging sample period (compromise between lower noise and increased execution time)
  if (fVL6180X_Synchron_WriteRegister8(0x003f, 0x46)==false) return false;       // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
  if (fVL6180X_Synchron_WriteRegister8(0x0031, 0xFF)==false) return false;       // sets the # of range measurements after which auto calibration of system is performed
  if (fVL6180X_Synchron_WriteRegister8(0x0040, 0x63)==false) return false;       // Set ALS integration time to 100ms
  if (fVL6180X_Synchron_WriteRegister8(0x002e, 0x01)==false) return false;       // perform a single temperature calibration of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  if (fVL6180X_Synchron_WriteRegister8(0x001b, 0x09)==false) return false;       // Set default ranging inter-measurement period to 100ms
  if (fVL6180X_Synchron_WriteRegister8(0x003e, 0x31)==false) return false;       // Set default ALS inter-measurement period to 500ms
  if (fVL6180X_Synchron_WriteRegister8(0x0014, 0x24)==false) return false;       // Configures interrupt on 'New Sample Ready threshold event'

  if (fVL6180X_Synchron_WriteRegister8(0x0016, 0x00)==false) return false;       // VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET
  return true;
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
     Initialisiert die gesamte Trittplatte.
 */
boolean fFootprint_Init () {
  // Servos vorbereiten:
  FootprintLeft.sServo.attach (PIN_FOOTPRINT_LEFT_SERVO);
  FootprintRight.sServo.attach (PIN_FOORPRINT_RIGHT_SERVO);
//  FootprintLeft.sServo.writeMicroseconds (1500);  // Put Servo in Position 0° (Values are: 1000...2000 = -90° ... +90°)
//  FootprintRight.sServo.writeMicroseconds (1500); // Put Servo in Position 0° (Values are: 1000...2000 = -90° ... +90°)
  // VL6180X Sensoren initialisieren:
  // Left footprint VL6180X init:
/*  fVL6180X_Init (LEFT_FOOTPRINT_I2C_UPPER,1);
  fVL6180X_Init (LEFT_FOOTPRINT_I2C_UPPER,2);
  fVL6180X_Init (LEFT_FOOTPRINT_I2C_UPPER,5);
  fVL6180X_Init (LEFT_FOOTPRINT_I2C_LOWER,1);
  fVL6180X_Init (LEFT_FOOTPRINT_I2C_LOWER,2);
  fVL6180X_Init (LEFT_FOOTPRINT_I2C_LOWER,5);
  // Left footprint GY271 init:
  fGY271_Init (LEFT_FOOTPRINT_I2C_UPPER, 0);
  fGY271_Init (LEFT_FOOTPRINT_I2C_UPPER, 3);
  fGY271_Init (LEFT_FOOTPRINT_I2C_UPPER, 4);
  fGY271_Init (LEFT_FOOTPRINT_I2C_LOWER, 0);
  fGY271_Init (LEFT_FOOTPRINT_I2C_LOWER, 3);
  fGY271_Init (LEFT_FOOTPRINT_I2C_LOWER, 4);
  */
  // Right footprint VL6180X init:
  /*
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_UPPER,1)==false) return false;
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_UPPER,2)==false) return false;
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_UPPER,5)==false) return false;
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_LOWER,1)==false) return false;
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_LOWER,2)==false) return false;
  if (fVL6180X_Init (RIGHT_FOOTPRINT_I2C_LOWER,5)==false) return false;
  */
  // Right footprint GY271 init:
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_UPPER, 0)==false) return false;
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_UPPER, 3)==false) return false;
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_UPPER, 4)==false) return false;
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_LOWER, 0)==false) return false;
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_LOWER, 3)==false) return false;
  if (fGY271_Init (RIGHT_FOOTPRINT_I2C_LOWER, 4)==false) return false;
  return true;
}

void fSTOP ()
{
  Serial.println ("System stopped.");
  while (1==1) 
  {
    delay(100);
  };
}
void fRaiseError (byte errornumber) {
  if (Serial.availableForWrite()>3)
  {
    Serial.print ("E");
    Serial.print (errornumber,HEX);
  }
}

// fCalculate_Intersection
// ***********************
// Calculate the intersection of two lines.
// a1 = X Position of the Sensor #1
// b1 = Y Position of the Sensor #1
// a2 = X Position of the Sensor #2
// b2 = Y Position of the Sensor #2
// S1x= X Direction where the sensor #1 is pointing to
// S1y= Y Direction where the sensor #1 is pointing to
// S2x= X Direction where the sensor #2 is pointing to
// S2y= Y Direction where the sensor #2 is pointing to
// RESULT:
// *x = X coordinate where the two lines cross each other
// *y = y coordinate where the two lines cross each other
// return value: TRUE = The result is valid
//               FALSE = Ignore the result, error in calculation
bool fCalculate_Intersection (double a1, double b1, double S1x, double S1y,
                              double a2, double b2, double S2x, double S2y,
                              tPosition *Position)
{
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

// fCalculate_Distance
// ~~~~~~~~~~~~~~~~~~~
// Calculate the distance between two points
double fCalculate_Distance (double x1, double y1, double x2, double y2)
{
  return sqrt( ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)) );
}

bool fCalculate_GY271_CenterOfFootplate (tFootprint *Footplate)
{
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
  return true;
}
/* ***********************************************  M A I N P R O G R A M M  ************************************************
   ************************************************************************************************************************** */

void setup() {

  Serial.begin(115000);
  Wire.begin ();        // I2C Bus initialisieren

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  delay (1000);  

  //fTCA9548A_Disable_I2C_BusDevice (LEFT_FOOTPRINT_I2C_UPPER);
  //fTCA9548A_Disable_I2C_BusDevice (LEFT_FOOTPRINT_I2C_LOWER);
  
  if (fTCA9548A_Disable_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER)==false)
  {
    Serial.println ("Failed: fTCA9548A_Disable_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_UPPER)");
    fSTOP ();
  }
  if (fTCA9548A_Disable_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER)==false)
  {
    Serial.println ("Failed: fTCA9548A_Disable_I2C_BusDevice (RIGHT_FOOTPRINT_I2C_LOWER)");
    fSTOP ();
  }
  if (fBasePlate_Init ()==false)    // Plattform initialisieren
  {
    Serial.println ("Failed: fBasePlate_Init ()");
    fSTOP ();
  }
  if (fFootprint_Init ()==false) 
  {
    Serial.println ("Failed: fFootprint_Init ()");
    fSTOP ();
  }

  // Millisecond timer interrupt
  OCR0A = 0x7D;
  TIMSK0 |= _BV(OCIE0A); 
  
}


void fReceiveCommandsFromPC ()
{
  int i;
  char cCommandFromPC;
  // Look for commands from PC:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (Serial.available())
  {
    cCommandFromPC=Serial.read ();
    if (cCommandFromPC=='1') // Reset right Footprint
    {
      FootprintRight.GY271[0].SetOffset=1;
      FootprintRight.GY271[1].SetOffset=1;
      FootprintRight.GY271[2].SetOffset=1;
      FootprintRight.GY271[3].SetOffset=1;
      FootprintRight.GY271[4].SetOffset=1;
      FootprintRight.GY271[5].SetOffset=1;
    }
    if (cCommandFromPC=='2') // Reset Offset
    {
      for (i=0;i<6;i++)
      {
        FootprintRight.GY271[i].Offset.X=0;
        FootprintRight.GY271[i].Offset.Y=0;
        FootprintRight.GY271[i].Offset.Z=0;
      }
    }
    if (cCommandFromPC=='3') // rotate baseplate +10 Units
    {
    }
    if (cCommandFromPC=='4') // rotate baseplate -10 units
    {
    }
  }
}

void fSendDataToPC ()
{
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
    Serial.print (FootprintRight.GY271[0].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[0].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[0].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[1].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[1].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[1].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[2].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[2].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[2].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[3].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[3].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[3].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[4].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[4].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[4].Value.Z); Serial.print ("\t");
    Serial.print (FootprintRight.GY271[5].Value.X); Serial.print ("\t"); Serial.print (FootprintRight.GY271[5].Value.Y); Serial.print ("\t"); Serial.print (FootprintRight.GY271[5].Value.Z); Serial.print ("\t");
    CX = FootprintRight.Center.X;
    CY = FootprintRight.Center.Y;
    Serial.print (CX); Serial.print ("\t"); Serial.print (CY); Serial.print ("\t");
    Serial.println ("#");  
  }  
}

void loop() {

  // Read values from Sensors:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~
  if (fGY271_Synchron_Read_Headings ()==false)
  {
    Serial.println ("Failed: fGY271_Synchron_Read_Headings ()");
    fSTOP ();
  }

  // Calculate stuff:
  // ~~~~~~~~~~~~~~~~
  fCalculate_GY271_CenterOfFootplate (&FootprintRight);  

  // Measure the cycle time:
  // ~~~~~~~~~~~~~~~~~~~~~~~
  ulStop=millis();  
  if ((ulStop-ulStart>100))
  {
    fSendDataToPC ();
    ulStart = millis();
  }

  fReceiveCommandsFromPC ();
  
}


// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect) 
{
    static uint16_t elapsed_ms = 0;    
    // Only process every 500ms
    if (elapsed_ms++ > 500)
    {
        elapsed_ms = 0; // Reset
    }
}
