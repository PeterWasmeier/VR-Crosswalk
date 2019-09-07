void fSteppermotor_Init () {
  Steppermotor.bTurnedOn=false;
  Steppermotor.uiAcceleration = STEPPER_ACCELERATION;
  Steppermotor.iCurrentPosition=30*5; // When power up, the footplate is 30mm away from the center. Each step is ~0,2mm. So 30*5 steps=30mm
  Steppermotor.iTargetPosition=30*5;  // When power up, the footplate is 30mm away from the center. Each step is ~0,2mm. So 30*5 steps=30mm
  Steppermotor.bStepIndex=0;
  pinMode (PIN_STEPPER_COIL1_PLUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL2_PLUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL1_MINUS, OUTPUT);
  pinMode (PIN_STEPPER_COIL2_MINUS, OUTPUT);
  digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
  digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
  digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
  digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);  
}

void fSteppermotor_TurnOn () {
  if (Steppermotor.bTurnedOn==false)
  {
    digitalWrite (PIN_STEPPER_COIL1_PLUS,  bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL1_PLUS);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,  bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL2_PLUS);
    digitalWrite (PIN_STEPPER_COIL1_MINUS, bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL1_MINUS);
    digitalWrite (PIN_STEPPER_COIL2_MINUS, bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL2_MINUS);
    Steppermotor.ulStart = millis ();
    Steppermotor.bLastDirection=0;
    Steppermotor.bTurnedOn=true;
  }
}

void fSteppermotor_TurnOff () {
  if (Steppermotor.bTurnedOn==true)
  {
    Steppermotor.bTurnedOn=false;
    digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);  
  }
}

void fSteppermotor_setTargetPosition (int iTargetPosition, bool bAutoTurnOff) {
  if (iTargetPosition<0) iTargetPosition=0;
  if (iTargetPosition>300) iTargetPosition=300;
  Steppermotor.bAutoTurnOff = bAutoTurnOff;
  Steppermotor.iTargetPosition = iTargetPosition; 
}

void fSteppermotor_Execute () {
  unsigned long ulStop;
  unsigned long ulDuration;
  double dVelocity;
  double dRunningSince;
  double dTimeDelayBetweenTwoPulses;
  double dPulsesPerSecond;
  unsigned long lTimeDelayBetweenTwoPulses;
  
  if (Steppermotor.bTurnedOn==false) return;

  ulStop = millis ();
  if (ulStop<Steppermotor.ulStart)
    ulDuration = (0xFFFFFFFF - Steppermotor.ulStart) + ulStop; // Overflow
  else
    ulDuration = ulStop - Steppermotor.ulStart;

  dRunningSince = ulDuration;                 // Result is milliseconds
  dRunningSince = dRunningSince / 1000;       // Convert into seconds since the motor is spinning
  dVelocity = Steppermotor.uiAcceleration;   // Calculate the necessary speed of the motor
  dVelocity = dVelocity * dRunningSince;      // Convert into mm/sec (current speed to use)

  dPulsesPerSecond = (dVelocity * 5);         // Convert from mm/sec into Pulses per second

  dTimeDelayBetweenTwoPulses = 1000.0 / dPulsesPerSecond; // How many milliseconds between two pulses?
  lTimeDelayBetweenTwoPulses = dTimeDelayBetweenTwoPulses;

  if (lTimeDelayBetweenTwoPulses<ulDuration) return; // The time is not yet over, so exit
  
  // It is time to make a new pulse:
  if (Steppermotor.iTargetPosition!=Steppermotor.iCurrentPosition)
  {
    if (Steppermotor.iTargetPosition>Steppermotor.iCurrentPosition)
    {
      if (Steppermotor.bLastDirection==-1)
      {
        Steppermotor.ulStart = millis ();                // Reset the start time, so the acceleration will begin again
        Steppermotor.bLastDirection=1;
        return;                             // We were moving in the opposite direction, so wait 1ms and come again
      }
      Steppermotor.bLastDirection=1;
      Steppermotor.iCurrentPosition++;
      if (Steppermotor.bStepIndex<3)
        Steppermotor.bStepIndex++;
      else
        Steppermotor.bStepIndex=0;
    }
    else
    {
      if (Steppermotor.bLastDirection==1)
      {
        Steppermotor.ulStart = millis ();                // Reset the start time, so the acceleration will begin again
        Steppermotor.bLastDirection=-1;
        return;                             // We were moving in the opposite direction, so wait 1ms and come again
      }
      Steppermotor.bLastDirection=-1;
      Steppermotor.iCurrentPosition--;
      if (Steppermotor.bStepIndex>0)
        Steppermotor.bStepIndex--;
      else
        Steppermotor.bStepIndex=3;
    }
    digitalWrite (PIN_STEPPER_COIL1_PLUS,  bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL1_PLUS);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,  bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL2_PLUS);
    digitalWrite (PIN_STEPPER_COIL1_MINUS, bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL1_MINUS);
    digitalWrite (PIN_STEPPER_COIL2_MINUS, bSteppermotor_Steps[Steppermotor.bStepIndex].bPIN_COIL2_MINUS);
    Steppermotor.ulStart = millis ();
  }
  else
  { 
    if ((Steppermotor.bAutoTurnOff)&&(ulDuration>50)) // Turn off after 50ms
    {
      Steppermotor.bLastDirection=0;                                       // Don't care anymore about direction change
      fSteppermotor_TurnOff ();                                             // This makes it necessary to call TurnOn and this one will set ulStart again
    }
  }
}
