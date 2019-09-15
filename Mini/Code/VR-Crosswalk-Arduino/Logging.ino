
void fLogging () {
  static unsigned long llm;
  unsigned long lm;
  int lX;
  int lY;
  int llX;
  int llY;
  bool bSendGY271;

  lm = millis ();         // Example: 12636254
  if ((lm-llm>250)&&(FootprintRight.SensorOffsetValid==true))
  {
    bSendGY271=true; 
    llm=lm;
  }
  else 
  {
    bSendGY271=false;
  }

  if (bRequestAll) {
    Serial.print ("P;"); Serial.println (Motor.dP);
    Serial.print ("I;"); Serial.println (Motor.dI);
    Serial.print ("D;"); Serial.println (Motor.dD);
  }
  if ((Logging.iFootprintRight_Alpha!=FootprintRight.Alpha)||(bRequestAll))
  {
    Logging.iFootprintRight_Alpha=FootprintRight.Alpha;
    Serial.print ("FOOT_R_A;"); Serial.println (Logging.iFootprintRight_Alpha);    
  }
  if ((Logging.iServoLeftPosition!=iServoLeftPosition)||(bRequestAll))
  {
    Logging.iServoLeftPosition=iServoLeftPosition;
    Serial.print ("SERVO_L_POS;"); Serial.println (Logging.iServoLeftPosition);
  }
  if ((Logging.iServoRightPosition!=iServoRightPosition)||(bRequestAll))
  {
    Logging.iServoRightPosition=iServoRightPosition;
    Serial.print ("SERVO_R_POS;"); Serial.println (Logging.iServoRightPosition);
  }
  if ((Logging.bMotor_LastPIDValue!=Motor.bLastPIDValue)||(bRequestAll))
  {
    Logging.bMotor_LastPIDValue=Motor.bLastPIDValue;
    Serial.print ("MOTOR_PWM;"); Serial.println (Logging.bMotor_LastPIDValue);
  }
  if ((Logging.iMotor_TargetPosition!=Motor.iTargetPosition)||(bRequestAll))
  {
    Logging.iMotor_TargetPosition=Motor.iTargetPosition;
    Serial.print ("MOTOR_TARGET;"); Serial.println (Logging.iMotor_TargetPosition);    
  }
  if ((Logging.iMotor_EncoderPosition!=Motor.iEncoderPosition)||(bRequestAll))
  {
    Logging.iMotor_EncoderPosition=Motor.iEncoderPosition;
    Serial.print ("MOTOR_ENCODER;"); Serial.println (Logging.iMotor_EncoderPosition);    
  }
  if ((Logging.iSteppermotor_CurrentPosition!=Steppermotor.iCurrentPosition)||(bRequestAll))
  {
    Logging.iSteppermotor_CurrentPosition=Steppermotor.iCurrentPosition;
    Serial.print ("STEPPER_POSITION;"); Serial.println (Logging.iSteppermotor_CurrentPosition);    
  }
  if ((Logging.iSteppermotor_TargetPosition!=Steppermotor.iTargetPosition)||(bRequestAll))
  {
    Logging.iSteppermotor_TargetPosition=Steppermotor.iTargetPosition;
    Serial.print ("STEPPER_TARGET;"); Serial.println (Logging.iSteppermotor_TargetPosition);    
  }

  if (bSendGY271==true)
  {
    lX = (FootprintRight.SensorOffset.X+FootprintRight.Offset.X)*1000;
    lY = (FootprintRight.SensorOffset.Y+FootprintRight.Offset.Y)*1000; 
    llX= (Logging.FootplateRight_SensorOffset.X+FootprintRight.Offset.X)*1000;
    llY= (Logging.FootplateRight_SensorOffset.Y+FootprintRight.Offset.Y)*1000;
    {
      Logging.FootplateRight_SensorOffset.X=FootprintRight.SensorOffset.X;
      Logging.FootplateRight_SensorOffset.Y=FootprintRight.SensorOffset.Y;
      Serial.print ("FOOT_R_X;"); Serial.println (lX);
      Serial.print ("FOOT_R_Y;"); Serial.println (lY);    
    }
  }
  
  bRequestAll=false;
}
