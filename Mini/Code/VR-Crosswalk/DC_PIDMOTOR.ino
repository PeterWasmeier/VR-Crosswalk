
void fMotor_TurnOn () {
  if (Motor.bTurnedOn==false)
  {
    Motor.lIntegral=0;
    analogWrite (PIN_BASEPLATE_MOTOR_ENABLE,Motor.bLastPWMValue);
    Motor.ulSampleLastTime = micros();
    Motor.bTurnedOn=true;
  }
}

void fMotor_TurnOff () {
  if (Motor.bTurnedOn==true)
  {
    Motor.bTurnedOn=false;
    analogWrite (PIN_BASEPLATE_MOTOR_ENABLE,0);
  }
}

void fMotor_setTargetPosition (int iTargetPosition) {
  if (Motor.iTargetPosition!=iTargetPosition)
  {
    Motor.lIntegral=0;
    Motor.iTargetPosition=iTargetPosition;
  }
}

int fMotor_getTargetPosition () {
  return Motor.iTargetPosition;
}

void fMotor_setEncoderInverted (bool bInverted) {
  Motor.bEncoderInverted=bInverted;
}

void fMotor_setCurrentPosition (int iNewPosition) {
  Motor.iEncoderPosition=iNewPosition; 
}

byte fMotor_setPWM(long lNewPWMValue) {
  byte bNewPWMValue;
  lNewPWMValue=abs(lNewPWMValue);
  if (lNewPWMValue>MOTOR_HIGH_SPEED)
  {
    lNewPWMValue=MOTOR_HIGH_SPEED;
  }
  bNewPWMValue=lNewPWMValue;
  if (bNewPWMValue!=Motor.bLastPWMValue)
  {
    Motor.bLastPWMValue=bNewPWMValue;
    if (Motor.bTurnedOn==true)
    {
      analogWrite (PIN_BASEPLATE_MOTOR_ENABLE,Motor.bLastPWMValue);
    }
  }
  return Motor.bLastPWMValue;
}

int fMotor_getCurrentPosition () {
  return Motor.iEncoderPosition;
}

void fMotor_setDirection (signed char bDirection) {
  if (Motor.bLastDirection!=bDirection)
  {
    if (bDirection>0)
    {
      digitalWrite (PIN_BASEPLATE_MOTOR_PLUS,HIGH);
      digitalWrite (PIN_BASEPLATE_MOTOR_MINUS,LOW);
    }
    else
    if (bDirection<0)
    {
      digitalWrite (PIN_BASEPLATE_MOTOR_PLUS,LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR_MINUS,HIGH);    
    }
    else
    {
      digitalWrite (PIN_BASEPLATE_MOTOR_PLUS,LOW);
      digitalWrite (PIN_BASEPLATE_MOTOR_MINUS,LOW);    
    }
    Motor.bLastDirection=bDirection;
  }
}

long fMotor_compute() {
  unsigned long ulcurTime;
  unsigned long ulsampleTime;
  float fsampleTime;
  int icurError;
  float fcurError;
  float pTerm;
  float iTerm;
  float dTerm;
  int iDiff;
  float fDiff;

  ulcurTime = micros();
  if (ulcurTime<Motor.ulSampleLastTime)
    ulsampleTime = (0xFFFFFFFF - Motor.ulSampleLastTime) + ulcurTime;
  else
    ulsampleTime = ulcurTime - Motor.ulSampleLastTime;
  if (ulsampleTime<1000) return Motor.lLastPIDValue; // Less than 1ms, do nothing
  
  Motor.ulSampleLastTime=ulcurTime;
  fsampleTime = ulsampleTime;
  fsampleTime = fsampleTime / 1000000; // fsampleTime is now in seconds
  
  icurError = Motor.iTargetPosition - Motor.iEncoderPosition;
  fcurError = icurError;
  
  pTerm = Motor.fP * fcurError;
  if (Motor.bInterruptEnabled==true)
  {
    if (abs(icurError)<10)
    {
      Motor.lIntegral  = Motor.lIntegral + icurError;
      if (Motor.lIntegral>1000000) Motor.lIntegral=1000000;
      if (Motor.lIntegral<-1000000) Motor.lIntegral=-1000000;
      iTerm = Motor.fI*Motor.lIntegral*fsampleTime;
    }
    else
      iTerm=0;
  }
  else
    iTerm=0;
  if (Motor.bInterruptEnabled==true)
  {
    iDiff = icurError - Motor.iLastError;
    fDiff = iDiff;
    dTerm = Motor.fD * fDiff / fsampleTime;
    Motor.iLastError=icurError;
  }
  else
    dTerm=0;
  Motor.lLastPIDValue=round(pTerm+iTerm+dTerm);
  return Motor.lLastPIDValue;
}

void fMotor_Init () {
  // War bisher gut geeignet
  //PIDParameter.fP = 6;
  //PIDParameter.fI = 1.21;
  //PIDParameter.fD = 0.08;

  // Bisher am besten auch mit 500er Schritten:
  Motor.fP = 6.4;
  Motor.fI = 1.49; // 1.21;
  Motor.fD = 0.49;//  0.63;
  Motor.lIntegral = 0;
  Motor.iLastError = 0;
  Motor.bEncoderInverted=false;
  
  pinMode (PIN_BASEPLATE_MOTOR_ENABLE, OUTPUT);
  analogWrite(PIN_BASEPLATE_MOTOR_ENABLE,0);
  pinMode (PIN_BASEPLATE_MOTOR_PLUS, OUTPUT);
  pinMode (PIN_BASEPLATE_MOTOR_MINUS, OUTPUT);
  digitalWrite (PIN_BASEPLATE_MOTOR_PLUS,LOW);
  digitalWrite (PIN_BASEPLATE_MOTOR_MINUS,LOW);
  
  pinMode (PIN_BASEPLATE_ENCODER_A, INPUT_PULLUP);
  pinMode (PIN_BASEPLATE_ENCODER_B, INPUT_PULLUP);
  Motor.bEncoderPreviousChannelA = digitalRead (PIN_BASEPLATE_ENCODER_A);
  Motor.bEncoderPreviousChannelB = digitalRead (PIN_BASEPLATE_ENCODER_B);
  Motor.bExecuteIsActive=false;
  Motor.bInterruptEnabled=true;
}

void fMotor_setPID(float pValue, float iValue, float dValue) {
  Motor.fP = pValue;
  Motor.fI = iValue;
  Motor.fD = dValue;
}

void fMotor_Execute () {
  long lpwmWidth;
  if (Motor.bExecuteIsActive==true) return;
  if (Motor.bTurnedOn==false) return;
  Motor.bExecuteIsActive=true;
  lpwmWidth = fMotor_compute();
  fMotor_setDirection(lpwmWidth);
  fMotor_setPWM(lpwmWidth);
  Motor.bExecuteIsActive=false;
}

bool fMotor_loopCheckEncoderInverted () {
  int ipwm = 0; 
  long lposition = 0;
  bool bResult;
  fMotor_TurnOff ();
  Motor.bInterruptEnabled=false;
  delay (1300);
  fMotor_setEncoderInverted (false);
  fMotor_setCurrentPosition (0);
  fMotor_setDirection(1);
  fMotor_setPWM(0);
  fMotor_TurnOn ();
  while(abs(lposition)<20){
    ipwm=fMotor_setPWM(ipwm+10);
    lposition = fMotor_getCurrentPosition();
    delay(1);
  }
  fMotor_TurnOff ();
  if(lposition < 0)
  {
    fMotor_setEncoderInverted (true);
    bResult=true;
  }
  else
    bResult=false;
  Motor.bInterruptEnabled=true;
  return bResult;
}

bool fMotor_loopAutoTune(void) {
  unsigned long ulStartMicros=0;
  unsigned long ulStopMicros=0;
  unsigned long ulDurationMicros=0;
  unsigned long ulWaveStartMicros=0;
  unsigned long ulWaveStopMicros=0;
  unsigned long ulWaveDurationMicros=0;
  unsigned long ulWaveDurationMicrosSum=0;
  float fWaveDurationAverage;
  int iPreviousEncoderPosition;
  int iCurrentEncoderPosition;
  int iWaveMinEncoderPosition;
  int iWaveMaxEncoderPosition;
  int iPOINT1;
  int iPOINT2;
  int iWaveCounts=0;
  int iWaveSumCounts=0;
  byte bCurrentDirection; // 1=left, 2=right
  byte bPreviousDirection; // 1=left, 2=right
  double Tu;
  
  Motor.bInterruptEnabled=false;
  fMotor_TurnOff ();
  fMotor_setPWM(0);
  delay (1000);
  iPOINT1=90;
  iPOINT2=100;
  Motor.fP=0;
  Motor.fI=0;
  Motor.fD=0;
  fMotor_setCurrentPosition (iPOINT1);
  fMotor_setTargetPosition (iPOINT2);
  iCurrentEncoderPosition = Motor.iEncoderPosition;
  iPreviousEncoderPosition= iCurrentEncoderPosition;
  iWaveMinEncoderPosition=iCurrentEncoderPosition;
  iWaveMaxEncoderPosition=iCurrentEncoderPosition;
  iWaveCounts=0;
  iWaveSumCounts=0;
  
  fMotor_TurnOn ();
  ulStartMicros=micros ();
  
  while (1)
  {
    fMotor_Execute ();
    iCurrentEncoderPosition=Motor.iEncoderPosition;
    ulStopMicros=micros();
    if (ulStopMicros<ulStartMicros)
      ulDurationMicros = (0xFFFFFFFF - ulStartMicros) + ulStopMicros; // Overflow
    else
      ulDurationMicros = ulStopMicros-ulStartMicros;
      
    if (iCurrentEncoderPosition!=iPreviousEncoderPosition)
    { // It has been moved somehow in any direction
      ulStartMicros=micros ();
      if (iCurrentEncoderPosition<iWaveMinEncoderPosition)
        iWaveMinEncoderPosition=iCurrentEncoderPosition;
      else
      if (iCurrentEncoderPosition>iWaveMaxEncoderPosition)
        iWaveMaxEncoderPosition=iCurrentEncoderPosition;
      // Check if it is "waving":
      if ((iWaveMinEncoderPosition<Motor.iTargetPosition)&&(iWaveMaxEncoderPosition>Motor.iTargetPosition))
      {
        // It is waving
        if (iCurrentEncoderPosition-iPreviousEncoderPosition>0)
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
          iWaveMinEncoderPosition=iCurrentEncoderPosition;
          iWaveMaxEncoderPosition=iCurrentEncoderPosition;
          if (iWaveSumCounts>39)
          { // We have 40 real waves without changing Kp Parameter
            fMotor_TurnOff ();
            fMotor_setPWM (0);
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
            Motor.fP = Motor.fP / 3.3;
            Motor.fI = Tu / 0.13;
            Motor.fD = Tu / 0.25;

            Motor.bInterruptEnabled=true;
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
        if (iCurrentEncoderPosition==Motor.iTargetPosition)
        {
          // Position is reached since 60ms.
          // But we want it waving, so choose another target position:
          fMotor_TurnOff ();
          fMotor_setPWM (0);
          delay (1300);
          iWaveCounts=0;
          ulWaveDurationMicrosSum=0;
          iWaveSumCounts=0;          
          iWaveMinEncoderPosition=iCurrentEncoderPosition;
          iWaveMaxEncoderPosition=iCurrentEncoderPosition;
          if (Motor.iTargetPosition==iPOINT1)
            fMotor_setTargetPosition (iPOINT2);
          else
            fMotor_setTargetPosition (iPOINT1);
          fMotor_TurnOn();
          ulStartMicros=micros ();
        }
        else
        {   // No movement since 60ms, and Targetposition not reached, increase Kp:
          fMotor_TurnOff ();
          fMotor_setPWM (0);
          Motor.fP=Motor.fP+1;
          delay (1300);
          fMotor_setCurrentPosition (iPOINT1);
          fMotor_setTargetPosition (iPOINT2);
          iCurrentEncoderPosition = Motor.iEncoderPosition;
          iPreviousEncoderPosition= iCurrentEncoderPosition;         
          fMotor_TurnOn ();
          iWaveCounts=0;
          ulWaveDurationMicrosSum=0;
          iWaveSumCounts=0;          
          iWaveMinEncoderPosition=iCurrentEncoderPosition;
          iWaveMaxEncoderPosition=iCurrentEncoderPosition;
          ulStartMicros=micros ();
        }
      }
    }
    iPreviousEncoderPosition=iCurrentEncoderPosition;
  }
}

void fMotor_EncoderInterrupt() {
  bool bEncoderChannelA;
  bool bEncoderChannelB;
  if (Motor.bEncoderInverted)
  {
    bEncoderChannelB = digitalRead (PIN_BASEPLATE_ENCODER_A);
    bEncoderChannelA = digitalRead (PIN_BASEPLATE_ENCODER_B);    
  }
  else
  {
    bEncoderChannelA = digitalRead (PIN_BASEPLATE_ENCODER_A);
    bEncoderChannelB = digitalRead (PIN_BASEPLATE_ENCODER_B);
  }
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==HIGH))
  {
    if ((Motor.bEncoderPreviousChannelA==HIGH)&&(Motor.bEncoderPreviousChannelB==HIGH))
      Motor.iEncoderPosition++;
    else
      Motor.iEncoderPosition--;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==HIGH))
  {
    if ((Motor.bEncoderPreviousChannelA==LOW)&&(Motor.bEncoderPreviousChannelB==HIGH))
      Motor.iEncoderPosition--;
    else
      Motor.iEncoderPosition++;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==LOW))
  {
    if ((Motor.bEncoderPreviousChannelA==HIGH)&&(Motor.bEncoderPreviousChannelB==HIGH))
      Motor.iEncoderPosition--;
    else
      Motor.iEncoderPosition++;
  }
  else
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==LOW))
  {
    if ((Motor.bEncoderPreviousChannelA==HIGH)&&(Motor.bEncoderPreviousChannelB==LOW))
      Motor.iEncoderPosition--;
    else
      Motor.iEncoderPosition++;
  }
  Motor.bEncoderPreviousChannelA = bEncoderChannelA;
  Motor.bEncoderPreviousChannelB = bEncoderChannelB;
  if ((Motor.bInterruptEnabled==true)&&(Motor.bExecuteIsActive==false)) fMotor_Execute ();
}
