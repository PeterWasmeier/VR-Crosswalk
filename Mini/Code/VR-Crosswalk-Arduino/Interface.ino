
void fInterface_ExecuteCommand () {
  double dValue;
  Interface.bValueActive=false;
  Interface.bCommandLength=0;
  Interface.bValueLength=0;
  if (strcmp (Interface.cCommand,"CHECKENCODER")==0) {
    bGO_R_Command_Active=false;
    fMotor_loopCheckEncoderInverted ();
  }
  else
  if (strcmp (Interface.cCommand,"AUTOTUNE")==0) {
    bGO_R_Command_Active=false;
    fMotor_loopAutoTune ();
  }
  else
  if (strcmp (Interface.cCommand,"STATUS")==0) {
    bRequestAll=true;
  }
  else
  if (strcmp (Interface.cCommand,"P")==0) {
    dValue = atoi (Interface.cValue);
    Motor.dP = dValue / 1000;
  }
  else
  if (strcmp (Interface.cCommand,"I")==0) {
    dValue = atoi (Interface.cValue);
    Motor.dI = dValue / 1000;
  }
  else
  if (strcmp (Interface.cCommand,"D")==0) {
    dValue = atoi (Interface.cValue);
    Motor.dD = dValue / 1000;
  }
  else
  if (strcmp (Interface.cCommand,"STEPPER_ACC")==0) {
    Steppermotor.Acceleration = atoi (Interface.cValue);
  }
  else
  if (strcmp (Interface.cCommand,"STEPPER")==0) {
    bGO_R_Command_Active=false;
    fSteppermotor_setTargetPosition (atoi (Interface.cValue), true);
    fSteppermotor_TurnOn ();
  }
  else
  if (strcmp (Interface.cCommand,"MOTOR")==0) {
    bGO_R_Command_Active=false;
    fMotor_setTargetPosition (atoi (Interface.cValue));
    fMotor_TurnOn ();
  }
  else
  if (strcmp (Interface.cCommand,"OFF")==0) {
    fMotor_TurnOff ();
    fSteppermotor_TurnOff ();
    bGO_R_Command_Active=false;
  }
  else
  if (strcmp (Interface.cCommand,"OFFSET_R")==0) {
    FootprintRight.Offset.X=-FootprintRight.SensorOffset.X;
    FootprintRight.Offset.Y=-FootprintRight.SensorOffset.Y;
  }
  else
  if (strcmp (Interface.cCommand,"MOVE_R")==0) {
    FootprintRight.Destination.X = FootprintRight.Current.X + 10;
    FootprintRight.Destination.Y = FootprintRight.Current.Y;
    fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
    fSteppermotor_TurnOn();
    fMotor_TurnOn();
    bGO_R_Command_Active=true;
  }
  else
  if (strcmp (Interface.cCommand,"MOVE_L")==0) {
    FootprintRight.Destination.X = FootprintRight.Current.X - 10;
    FootprintRight.Destination.Y = FootprintRight.Current.Y;
    fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
    fSteppermotor_TurnOn();
    fMotor_TurnOn();
    bGO_R_Command_Active=true;
  }
  else
  if (strcmp (Interface.cCommand,"MOVE_U")==0) {
    FootprintRight.Destination.X = FootprintRight.Current.X;
    FootprintRight.Destination.Y = FootprintRight.Current.Y + 10;
    fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
    fSteppermotor_TurnOn();
    fMotor_TurnOn();
    bGO_R_Command_Active=true;
  }
  else
  if (strcmp (Interface.cCommand,"MOVE_D")==0) {
    FootprintRight.Destination.X = FootprintRight.Current.X;
    FootprintRight.Destination.Y = FootprintRight.Current.Y - 10;
    fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
    fSteppermotor_TurnOn();
    fMotor_TurnOn();
    bGO_R_Command_Active=true;
  }
  else
  if (strcmp (Interface.cCommand,"GO_R")==0) {    
    // Turn all the axis on:
    FootprintRight.Destination.X = FootprintRight.Current.X + FootprintRight.SensorOffset.X + FootprintRight.Offset.X;
    FootprintRight.Destination.Y = FootprintRight.Current.Y + FootprintRight.SensorOffset.Y + FootprintRight.Offset.Y;
    fCNC_SetDestination  (FootprintRight.Destination.X,FootprintRight.Destination.Y); // Units in mm
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    fSteppermotor_setTargetPosition (fCNC_CalculateSecondaryAxis(Motor.iEncoderPosition), true);
    fSteppermotor_TurnOn();
    fMotor_TurnOn();
    bGO_R_Command_Active=true;
  }
}

void fInterface () {
  char cLetter;
  if (Serial.available()) 
  {
    cLetter = Serial.read();
    if (Interface.bValueActive==true)
    {
      if (Interface.bValueLength<sizeof(Interface.cValue))
      {
        if (cLetter=='\r')
        {
          Interface.cValue[Interface.bValueLength]=0;
          fInterface_ExecuteCommand ();
          return;
        }
        else
        if (cLetter=='\n') return;
        Interface.cValue[Interface.bValueLength++]=cLetter;        
      }
      else
      {
        Interface.bValueActive=false;
        Interface.bCommandLength=0;
        Interface.bValueLength=0;
        Serial.println ("Value too long");
      }
    }
    else
    {
      if (Interface.bCommandLength<sizeof(Interface.cCommand))
      {
        if (cLetter=='=')
        {
          Interface.cCommand[Interface.bCommandLength]=0;
          Interface.bValueActive=true;
          return;
        }
        else
        if (cLetter=='\r')
        {
          Interface.cCommand[Interface.bCommandLength]=0;
          fInterface_ExecuteCommand ();
          return;
        }
        else
        if (cLetter=='\n') return;
        Interface.cCommand[Interface.bCommandLength++]=cLetter;
      }
      else
      {
        Interface.bValueActive=false;
        Interface.bCommandLength=0;
        Interface.bValueLength=0;
        Serial.println ("Command too long");
        Serial.println ("Type HELP");
      }
    }
  }
}
