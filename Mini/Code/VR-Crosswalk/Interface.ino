
void fInterface_ExecuteCommand () {
  Interface.bValueActive=false;
  Interface.bCommandLength=0;
  Interface.bValueLength=0;
  if (strcmp (Interface.cCommand,"HELP")==0) {
    Serial.println ("OFF           Will turn off motor and stepper");
    Serial.println ("STEPPER=100   Turn on stepper and move to position 100");
    Serial.println ("MOTOR=600     Turn on motor and rotate to +90° (=600 units)");
  }
  else
  if (strcmp (Interface.cCommand,"STEPPER")==0) {
    fSteppermotor_setTargetPosition (atoi (Interface.cValue), true);
    fSteppermotor_TurnOn ();
  }
  else
  if (strcmp (Interface.cCommand,"MOTOR")==0) {
    fMotor_setTargetPosition (atoi (Interface.cValue));
    fMotor_TurnOn ();
  }
  else
  if (strcmp (Interface.cCommand,"OFF")==0) {
    fMotor_TurnOff ();
    fSteppermotor_TurnOff ();
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