void loop(){
  int iServoLeftPosition;
  int iServoRightPosition;
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
  fLogging ();
  
  // Control the angle of the footplate:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  dPrimaryCurrentPosition = iPrimaryCurrentPosition;
  iServoLeftPosition = 1500;
  dServoRightPosition = 1500 + (  ((180.0/600.0) * dPrimaryCurrentPosition) * (600.0/90.0) );
  iServoRightPosition = dServoRightPosition;
  if (iServoRightPosition>2100) iServoRightPosition=2100;
  if (iServoRightPosition<900) iServoRightPosition=900;  
  if (iServoLeftPosition>2100) iServoLeftPosition=2100;
  if (iServoLeftPosition<900) iServoLeftPosition=900;  
  sServoLeft.writeMicroseconds (iServoLeftPosition);
  sServoRight.writeMicroseconds (iServoRightPosition+81);

}
