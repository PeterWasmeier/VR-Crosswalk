void loop(){
  double dServoRightPosition;
  int iPrimaryCurrentPosition;
  double dPrimaryCurrentPosition;
  int iSecondaryCurrentPosition;
  
  fGY271_Read_Values ();

  iPrimaryCurrentPosition = Motor.iEncoderPosition;  
  iSecondaryCurrentPosition = Steppermotor.iCurrentPosition;;
  
  //fCNC_GetCurrentXY (iPrimaryCurrentPosition, iSecondaryCurrentPosition, &FootprintRight.Current.X, &FootprintRight.Current.Y);
  //fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
  
  fMotor_Execute();
  fInterface ();
  
  // Control the angle of the footplate:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  dPrimaryCurrentPosition = iPrimaryCurrentPosition;
  iServoLeftPosition = 1500;
  dServoRightPosition = 1500 + (  ((90.0/ENCODER_PULSES_PER_90DEGREE) * dPrimaryCurrentPosition) * (600.0/45.0) ); // The servo will be -600=-45°, +600=+45° 
  iServoRightPosition = dServoRightPosition;
  if (iServoRightPosition>2100) iServoRightPosition=2100;
  if (iServoRightPosition<900) iServoRightPosition=900;  
  if (iServoLeftPosition>2100) iServoLeftPosition=2100;
  if (iServoLeftPosition<900) iServoLeftPosition=900;  
  sServoLeft.writeMicroseconds (iServoLeftPosition);
  sServoRight.writeMicroseconds (iServoRightPosition+81);

  
  fLogging ();

}
