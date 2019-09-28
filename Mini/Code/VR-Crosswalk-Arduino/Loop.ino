void loop(){
  int iPrimaryCurrentPosition;
  double dPrimaryCurrentPosition;
  int iSecondaryCurrentPosition;
  int iX;
  int iY;
  
  fGY271_Read_Values ();

  iPrimaryCurrentPosition = Motor.iEncoderPosition;  
  iSecondaryCurrentPosition = sStepper.readSteps();
  
  fCNC_GetCurrentXY (iPrimaryCurrentPosition, iSecondaryCurrentPosition, &FootprintRight.Current.X, &FootprintRight.Current.Y);
  
  if (bGO_R_Command_Active==true)
  {
    if ((FootprintRight.SensorOffsetValid==true)&&((bGO_R_Command_Once==true)||(bGO_R_Command_Always==true))) 
    {
      bGO_R_Command_Once=false;

      iX = FootprintRight.Current.X + FootprintRight.Offset.X + FootprintRight.SensorOffset.X;
      iY = FootprintRight.Current.Y + FootprintRight.Offset.Y + FootprintRight.SensorOffset.Y;

      fAverageMedianXY (&iX,&iY);
      
      if ((abs(FootprintRight.Destination.X-iX)>2) ||
          (abs(FootprintRight.Destination.Y-iY)>2)) 
      {
        FootprintRight.Destination.X = iX;
        FootprintRight.Destination.Y = iY;
        fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
      }
    }

    Steppermotor_iTargetPosition = fCNC_CalculateSecondaryAxis(Motor.iTargetPosition);
    sStepper.writeSteps (Steppermotor_iTargetPosition);
  }
  
  fMotor_Execute();
  Handle_Servos ();
  fInterface ();
  fLogging ();

}

void Handle_Servos (void)
{
  static int iServoLeft;
  static int iServoRight;
//  double dPrimaryCurrentPosition;
  int iServoPosition;

  FootprintRight.dServoPosition = FootprintRight.dServoPosition + (FootprintRight.Alpha-90) * 0.1;  // Proportional Controller to go to 90°
  if (FootprintRight.dServoPosition>2100) FootprintRight.dServoPosition=2100;
  if (FootprintRight.dServoPosition<900) FootprintRight.dServoPosition=900;  
  FootprintRight.iServoPosition = FootprintRight.dServoPosition;


  iServoPosition = FootprintRight.iServoPosition;
  if (iServoPosition<1500) iServoPosition = map (iServoPosition, 900, 1500, SERVO_RIGHT_MINUS45, SERVO_RIGHT_MIDDLE); else
  if (iServoPosition>1500) iServoPosition = map (iServoPosition, 1500, 2100, SERVO_RIGHT_MIDDLE, SERVO_RIGHT_PLUS45); else iServoPosition=SERVO_RIGHT_MIDDLE;  
  if ((iServoPosition<2100)&&(iServoPosition>900))
  {
    //if (abs(iServoRight-iServoPosition)>15)
    {
      if (iServoRight!=iServoPosition)
      {
        sServoRight.write (iServoPosition);
        iServoRight=iServoPosition;
      }
    }
  }

}
