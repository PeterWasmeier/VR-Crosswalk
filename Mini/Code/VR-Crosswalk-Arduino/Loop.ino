void loop(){
  int iPrimaryCurrentPosition;
  double dPrimaryCurrentPosition;
  double dDegreeFootprintRight;
  int iSecondaryCurrentPosition;
  int iX;
  int iY;
  
  fGY271_Read_Values ();

  iPrimaryCurrentPosition = Motor.iEncoderPosition;  
  iSecondaryCurrentPosition = Steppermotor.iCurrentPosition;
  
  fCNC_GetCurrentXY (iPrimaryCurrentPosition, iSecondaryCurrentPosition, &FootprintRight.Current.X, &FootprintRight.Current.Y);
  
  if (bGO_R_Command_Active==true)
  {
    if ((FootprintRight.SensorOffsetValid==true)&&((bGO_R_Command_Once==true)||(bGO_R_Command_Always==true))) 
    {
      bGO_R_Command_Once=false;

      dDegreeFootprintRight = FootprintRight.dServoPosition - 1500; // -45° = -600, 0°=0, +45°=+600 (plus offset of 1500)
      dDegreeFootprintRight = 45.0/600.0 * dDegreeFootprintRight;
      RotatePoint ( FootprintRight.SensorOffset.X, 
                    FootprintRight.SensorOffset.Y,
                    -dDegreeFootprintRight,
                    &FootprintRight.SensorOffset.X,
                    &FootprintRight.SensorOffset.Y);
      iX = FootprintRight.Current.X + FootprintRight.SensorOffset.X + FootprintRight.Offset.X;
      iY = FootprintRight.Current.Y + FootprintRight.SensorOffset.Y + FootprintRight.Offset.Y;
      if ((abs(FootprintRight.Destination.X-iX)>5) ||
          (abs(FootprintRight.Destination.Y-iY)>5)) 
      {
        FootprintRight.Destination.X = iX;
        FootprintRight.Destination.Y = iY;
        fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
        fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
        fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis( Motor.iTargetPosition/* iPrimaryCurrentPosition*/), true);
        fSteppermotor_TurnOn();
        fMotor_TurnOn();
      }
    }

    {
      fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iTargetPosition/*iPrimaryCurrentPosition*/), true);    
      if (Steppermotor.iTargetPosition!=Steppermotor.iCurrentPosition)
      {
        if (Steppermotor.bTurnedOn==false)
          fSteppermotor_TurnOn();
      }
    }
  }
  
  fMotor_Execute();
  fInterface ();
  fLogging ();

}
