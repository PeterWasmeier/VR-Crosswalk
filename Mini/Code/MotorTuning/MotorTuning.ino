const byte MOTOR_HIGH_SPEED          = 90;            // Der Motor darf maximal 6V erhalten
                                                       // Es liegen am Motortreiber "L298N" 12V extern an. Er besitzt 1,4V Verlust
                                                       // D.h. ein 
                                                       // PWM mit   0 = 0V
                                                       // PWM mit 144 = 5,986V
                                                       // PWM mit 255 = 10,6V (12V - 1,4 Verlust)
const char PIN_BASEPLATE_ENCODER_B   = 2;   // INT     Plattform Encoder: INT Input
const char PIN_BASEPLATE_ENCODER_A   = 3;   // PWM,INT Plattform Encoder: INT Input
const char PIN_BASEPLATE_MOTOR_PLUS = 7;   //         Plattform Motor: Richtung Plus/Clockwise
const char PIN_BASEPLATE_MOTOR_MINUS= 8;   //         Plattform Motor: Richtung Minus/Counter clockwise
const char PIN_BASEPLATE_MOTOR_ENABLE= 11;  // PWM490


/* PID tuning state machine */

class tDC_PIDMOTOR {
  
  typedef struct {
    float fP;
    float fI;
    float fD;    
    long lLastError;
    unsigned long ulSampleLastTime;
    volatile long lIntegral;
    long lLastValue;
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
    tPIDParameter     PIDParameter;
    tPWMParameter     PWMParameter;
    tMotorParameter   MotorParameter;

    volatile bool bInterruptEnabled;
    volatile bool bExecuteIsActive;
    long compute();

  public:
    tDC_PIDMOTOR (byte bPIN_MOTOR_PWM, long bPWM_Max, byte bPIN_MOTOR_A, byte bPIN_MOTOR_B, byte bPIN_ENCODER_CHANNEL_A, byte bPIN_ENCODER_CHANNEL_B);
    void Execute();
    byte setPWM(long PWM_Width);
    void setTargetPosition (long lTargetPosition);
    void setEncoderInverted (bool bInverted);
    void setEncoderPosition (long lNewPosition);
    long getEncoderPosition();
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
  }
}

void tDC_PIDMOTOR::TurnOff () {
  if (MotorParameter.bTurnedOn==true)
  {
    MotorParameter.bTurnedOn=false;
    analogWrite (PWMParameter.bPIN_MOTOR_PWM,0);
  }
}

void tDC_PIDMOTOR::setTargetPosition (long lTargetPosition) {
  if (MotorParameter.lTargetPosition!=lTargetPosition)
  {
    PIDParameter.lIntegral=0;
    MotorParameter.lTargetPosition=lTargetPosition;
  }
}

void tDC_PIDMOTOR::setEncoderInverted (bool bInverted) {
  EncoderParameter.bEncoderInverted=bInverted;
}

void tDC_PIDMOTOR::setEncoderPosition (long lNewPosition) {
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

long tDC_PIDMOTOR::getEncoderPosition () {
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
      if (PIDParameter.lIntegral>10000) PIDParameter.lIntegral=10000;
      if (PIDParameter.lIntegral<-10000) PIDParameter.lIntegral=-10000;
      iTerm = PIDParameter.fI*PIDParameter.lIntegral*fsampleTime;
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
  PIDParameter.fI = 1.21;
  PIDParameter.fD = 0.63;
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
  setEncoderPosition (0);
  setDirection(1);
  setPWM(0);
  TurnOn ();
  while(abs(lposition)<20){
    ipwm=setPWM(ipwm+10);
    lposition = getEncoderPosition();
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
  setEncoderPosition (lPOINT1);
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
            Serial.print ("Kp=");
            Serial.println (PIDParameter.fP);
            Serial.print ("Tu=");
            Serial.println (Tu);
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
          setEncoderPosition (lPOINT1);
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

void DC_PIDMOTOR_EncoderInterrupt () {
  DC_PIDMOTOR.EncoderInterrupt ();
}

/* ************************************************************************************************* */
/* ************************************************************************************************* */
/* ************************************************************************************************* */

void setup(){
  Serial.begin(115200);
  while(!Serial)
  {
    delay (1);
  };
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  DC_PIDMOTOR.TurnOff ();
}

void loop(){
  static long lpEP;
  static bool mMove;
  static long lMove;
  static bool bMove;
  static unsigned long ulIntervall;
  static unsigned long ulLastIntervall;
  long lEP;
  char c;
  if (mMove==true)
  {
    ulIntervall = millis();
    if (ulIntervall-ulLastIntervall>5)
    {
      if (bMove==false)
      lMove++;
      else
      lMove--;
      if (lMove>500)
        bMove=true;
      if (lMove<0)
        bMove=false;
      DC_PIDMOTOR.setTargetPosition (lMove);
      DC_PIDMOTOR.TurnOn();
      ulLastIntervall=ulIntervall;
    }
  }
  DC_PIDMOTOR.Execute();
  if (Serial.available())
  {
    c=Serial.read();
    if (c=='m')
    {
      mMove=!mMove;
    }
    if (c=='I')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I()+0.01,DC_PIDMOTOR.getPID_D());
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='i')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I()-0.01,DC_PIDMOTOR.getPID_D());
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='P')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P()+0.1,DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D());
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='p')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P()-0.1,DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D());
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='D')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D()+0.01);
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='d')
    {
      DC_PIDMOTOR.setPID (DC_PIDMOTOR.getPID_P(),DC_PIDMOTOR.getPID_I(),DC_PIDMOTOR.getPID_D()-0.01);
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='0') 
    {
      DC_PIDMOTOR.TurnOff ();
      Serial.print("P: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("I: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("D: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
    if (c=='1') 
    {
      DC_PIDMOTOR.setTargetPosition (10);
      DC_PIDMOTOR.TurnOn ();
    }
    if (c=='2') 
    {
      DC_PIDMOTOR.setTargetPosition (20);
      DC_PIDMOTOR.TurnOn ();
    }
    if (c=='3') 
    {
      DC_PIDMOTOR.setTargetPosition (30);
      DC_PIDMOTOR.TurnOn ();
    }
    if (c=='5') 
    {
      DC_PIDMOTOR.setTargetPosition (500);
      DC_PIDMOTOR.TurnOn ();
    }
    if (c=='e')
    {
      if (DC_PIDMOTOR.loopCheckEncoderInverted ()==true)
      {
        Serial.println ("Encoder inverted.");
      }
      else
      {
        Serial.println ("Encoder not inverted.");
      }
    }
    /*----------- check encoder polarisation -------------- */
    if (c=='t')
    {
      /*-----------  PID auto-tuning -----------  */
      DC_PIDMOTOR.loopAutoTune();
      Serial.print("PID TUNED Kp: "); Serial.println(DC_PIDMOTOR.getPID_P());
      Serial.print("PID TUNED Ki: "); Serial.println(DC_PIDMOTOR.getPID_I());
      Serial.print("PID TUNED Kd: "); Serial.println(DC_PIDMOTOR.getPID_D());      
    }
  }
  lEP = DC_PIDMOTOR.getEncoderPosition ();
  if (lEP!=lpEP)
  {
    Serial.print ("EP=");
    Serial.println (lEP);
    lpEP=lEP;
  }
}
