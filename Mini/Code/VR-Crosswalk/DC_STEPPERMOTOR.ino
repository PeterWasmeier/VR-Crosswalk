void fSteppermotor_Init () {
  Steppermotor.bTurnedOn=false;
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
    Steppermotor.ulRunningSince=0;
    Steppermotor.bTurnedOn=true;
  }
}

void fSteppermotor_TurnOff () {
  if (Steppermotor.bTurnedOn==true)
  {
    Steppermotor.bTurnedOn=false;
    Steppermotor.ulRunningSince=0;
    digitalWrite (PIN_STEPPER_COIL1_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_PLUS,LOW);
    digitalWrite (PIN_STEPPER_COIL1_MINUS,LOW);
    digitalWrite (PIN_STEPPER_COIL2_MINUS,LOW);  
  }
}

void fSteppermotor_setTargetPosition (int iTargetPosition, bool bAutoTurnOff) {
  if (iTargetPosition<120) iTargetPosition=120; // The footplate can not move less than 120 Units (=24mm) to the center
  if (iTargetPosition>650) iTargetPosition=650; // The footplate can not move more than 650 Units (=130mm) away from the center because it is only 106mm long (+24mm hole begins here)
  Steppermotor.bAutoTurnOff = bAutoTurnOff;
  Steppermotor.iTargetPosition = iTargetPosition; 
}

void fSteppermotor_Execute () {
  unsigned long ulStop;
  unsigned long ulDurationBetweenTwoPulses;
  double dVelocity;
  double dRunningSince;
  double dTimeDelayBetweenTwoPulses;
  double dPulsesPerSecond;
  unsigned long lTimeDelayBetweenTwoPulses;
  
  if (Steppermotor.bTurnedOn==false) return;

  ulStop = millis ();
  if (ulStop<Steppermotor.ulStart)
    ulDurationBetweenTwoPulses = (0xFFFFFFFF - Steppermotor.ulStart) + ulStop; // Overflow
  else
    ulDurationBetweenTwoPulses = ulStop - Steppermotor.ulStart;

  Steppermotor.ulRunningSince += ulDurationBetweenTwoPulses; // Result is milliseconds
  dRunningSince = Steppermotor.ulRunningSince;
  dRunningSince = dRunningSince / 1000.0;
  dVelocity = STEPPER_ACCELERATION;   // Calculate the necessary speed of the motor
  dVelocity = dVelocity * dRunningSince;      // Convert into mm/sec (current speed to use)

  dPulsesPerSecond = (dVelocity * 5.0);         // Convert from mm/sec into Pulses per second

  dTimeDelayBetweenTwoPulses = 1000.0 / dPulsesPerSecond; // How many milliseconds between two pulses?
  lTimeDelayBetweenTwoPulses = dTimeDelayBetweenTwoPulses;

  if (ulDurationBetweenTwoPulses<lTimeDelayBetweenTwoPulses) return; // The time is not yet over, so exit
  
  // It is time to make a new pulse:
  if (Steppermotor.iTargetPosition!=Steppermotor.iCurrentPosition)
  {
    if (Steppermotor.iTargetPosition>Steppermotor.iCurrentPosition)
    {
      if (Steppermotor.bLastDirection==-1)
      {
        Steppermotor.ulRunningSince=0;
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
        Steppermotor.ulRunningSince=0;
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
    Steppermotor.ulRunningSince=0;
    if ((Steppermotor.bAutoTurnOff)&&(ulDurationBetweenTwoPulses>50)) // Turn off after 50ms
    {
      Steppermotor.bLastDirection=0;                                       // Don't care anymore about direction change
      fSteppermotor_TurnOff ();                                             // This makes it necessary to call TurnOn and this one will set ulStart again
    }
  }
}
