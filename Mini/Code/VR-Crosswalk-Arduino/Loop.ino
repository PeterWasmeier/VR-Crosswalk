void loop(){
  int iPrimaryCurrentPosition;
  double dPrimaryCurrentPosition;
  int iSecondaryCurrentPosition;
  
  fGY271_Read_Values ();

  iPrimaryCurrentPosition = Motor.iEncoderPosition;  
  iSecondaryCurrentPosition = Steppermotor.iCurrentPosition;;
  
  fCNC_GetCurrentXY (iPrimaryCurrentPosition, iSecondaryCurrentPosition, &FootprintRight.Current.X, &FootprintRight.Current.Y);
  if (bGO_R_Command_Active==true)
  {
    if (FootprintRight.SensorOffsetValid==true)
    {
      /*if ( 
          (FootprintRight.SensorOffset.X>1.0) ||
          (FootprintRight.SensorOffset.X<1.0) ||
          (FootprintRight.SensorOffset.Y>1.0) ||
          (FootprintRight.SensorOffset.Y<1.0) 
          )*/
      {
        FootprintRight.Destination.X = FootprintRight.Current.X + FootprintRight.SensorOffset.X + FootprintRight.Offset.X;
        FootprintRight.Destination.Y = FootprintRight.Current.Y + FootprintRight.SensorOffset.Y + FootprintRight.Offset.Y;
        fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
        fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
        fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(iPrimaryCurrentPosition), true);
        fSteppermotor_TurnOn();
        fMotor_TurnOn();
      }
    }
    else
    {
      fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(iPrimaryCurrentPosition), true);    
      if (Steppermotor.iTargetPosition!=Steppermotor.iCurrentPosition)
      {
        if (Steppermotor.bTurnedOn==false)
        {
          fSteppermotor_TurnOn();
        }
      }
      if (Motor.bTurnedOn==false)
      {
        fMotor_TurnOn();
      }
    }
  }
  
  fMotor_Execute();
  fInterface ();
  fLogging ();

}
