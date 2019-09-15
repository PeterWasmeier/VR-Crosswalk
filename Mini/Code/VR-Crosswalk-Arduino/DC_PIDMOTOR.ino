
void fMotor_TurnOn () {
  if (Motor.bTurnedOn==false)
  {
    Motor.dIntegral=0;
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
    //Motor.dIntegral=0;
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

signed char fMotor_setPWM(signed char bNewPWMValue) {
  bNewPWMValue=abs(bNewPWMValue);
  if (bNewPWMValue>MOTOR_HIGH_SPEED)
  {
    bNewPWMValue=MOTOR_HIGH_SPEED;
  }
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

signed char fMotor_compute() {
  unsigned long ulcurTime;
  unsigned long ulsampleTime;
  long lResult;
  double dsampleTime;
  int icurError;
  double dcurError;
  double pTerm;
  double iTerm;
  double dTerm;
  int iDiff;
  double dDiff;

  ulcurTime = micros();
  if (ulcurTime<Motor.ulSampleLastTime)
    ulsampleTime = (0xFFFFFFFF - Motor.ulSampleLastTime) + ulcurTime;
  else
    ulsampleTime = ulcurTime - Motor.ulSampleLastTime;
  if (ulsampleTime<1000) return Motor.bLastPIDValue; // Less than 1ms, do nothing
  
  Motor.ulSampleLastTime=ulcurTime;
  dsampleTime = ulsampleTime;
  dsampleTime = dsampleTime / 1000000; // fsampleTime is now in seconds
  
  icurError = Motor.iTargetPosition - Motor.iEncoderPosition;
  dcurError = icurError;
  // Calculate proportional:
  pTerm = Motor.dP * dcurError;
  // Calculate Integral:
  Motor.dIntegral  = Motor.dIntegral + (dcurError*dsampleTime);
  iTerm = Motor.dIntegral * Motor.dI;
  // Calculate differential:
  iDiff = icurError - Motor.iLastError;
  dDiff = iDiff;
  dTerm = Motor.dD * (dDiff / dsampleTime);
  Motor.iLastError=icurError;

  lResult = round(pTerm+iTerm+dTerm);
  
  if (lResult>120) 
  {
    lResult=120; 
  }
  else 
  if (lResult<-120) 
  {
    lResult=-120;
  }
  Motor.bLastPIDValue= lResult;
  return Motor.bLastPIDValue;
}

void fMotor_Init () {
  Motor.dP = 10.0;
  Motor.dI = 0.5;
  Motor.dD = 0.5;
  Motor.dIntegral = 0;
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
}

void fMotor_Execute () {
  signed char bpwmWidth;
  if (Motor.bExecuteIsActive==true) return;
  if (Motor.bTurnedOn==false) return;
  Motor.bExecuteIsActive=true;
  bpwmWidth = fMotor_compute();
  fMotor_setDirection(bpwmWidth);
  fMotor_setPWM(bpwmWidth);
  Motor.bExecuteIsActive=false;
}

void fMotor_loopCheckEncoderInverted () {
  int ipwm = 0; 
  long lposition = 0;
  bool bResult;
  fMotor_TurnOff ();
  delay (1300);
  fMotor_setEncoderInverted (false);
  fMotor_setCurrentPosition (0);
  fMotor_setDirection(1);
  fMotor_setPWM(0);
  Motor.bExecuteIsActive=true;
  fMotor_TurnOn ();
  while(abs(lposition)<20){
    ipwm=fMotor_setPWM(ipwm+10);
    lposition = Motor.iEncoderPosition;
    delay(1);
  }
  fMotor_TurnOff ();
  Motor.bExecuteIsActive=false;
  if(lposition < 0)
  {
    fMotor_setEncoderInverted (true);
    bResult=true;
  }
  else
    bResult=false;
  return bResult;
}

void fMotor_loopAutoTune(void) {
  unsigned long ulStartMicros=0;
  unsigned long ulStopMicros=0;
  unsigned long ulDurationMicros=0;
  unsigned long ulWaveStartMicros=0;
  unsigned long ulWaveStopMicros=0;
  unsigned long ulWaveDurationMicros=0;
  unsigned long ulWaveDurationMicrosSum=0;
  double dWaveDurationAverage;
  double dWaveDurationMicrosSum;
  double dWaveSumCounts;
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
  
  fMotor_TurnOff ();
  fMotor_setPWM(0);
  delay (1000);
  iPOINT1=50;
  iPOINT2=100;
  Motor.dP=0;
  Motor.dI=0;
  Motor.dD=0;
  fMotor_setCurrentPosition (iPOINT1);
  fMotor_setTargetPosition (iPOINT2);
  iCurrentEncoderPosition = Motor.iEncoderPosition;
  iPreviousEncoderPosition= iCurrentEncoderPosition;
  iWaveMinEncoderPosition=iCurrentEncoderPosition;
  iWaveMaxEncoderPosition=iCurrentEncoderPosition;
  iWaveCounts=0;
  iWaveSumCounts=0;
  Serial.println ("AUTOTUNE started...");
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
            dWaveDurationMicrosSum=ulWaveDurationMicrosSum;
            dWaveSumCounts=iWaveSumCounts;
            dWaveDurationAverage=dWaveDurationMicrosSum/dWaveSumCounts;

            Tu = (dWaveDurationAverage*2) / 1000000; // Average in Seconds
            // classic PID according to Ziegler–Nichols method
            //PIDParameter.fP = 0.6 * PIDParameter.fP;
            //PIDParameter.fI = Tu / 2; 
            //PIDParameter.fD = Tu / 8; 

            // no overshoot according to Ziegler–Nichols method
            Motor.dP = 0.2 * Motor.dP;
            Motor.dI = Tu / 2;
            Motor.dD = Tu / 3;
            Serial.println ("AUTOTUNE done.");
            Serial.print ("P="); Serial.println (Motor.dP);
            Serial.print ("I="); Serial.println (Motor.dI);
            Serial.print ("D="); Serial.println (Motor.dD);
            Motor.dIntegral=0;
            // these ones works good for the used DC motor:
            /*
            Motor.dP = Motor.fP / 3.3;
            Motor.dI = Tu / 0.13;
            Motor.dD = Tu / 0.25;
            */
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
          Motor.dP=Motor.dP+1;
          Serial.print ("P=");
          Serial.println (Motor.dP);
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
  if (Motor.bExecuteIsActive==false) fMotor_Execute ();
}
