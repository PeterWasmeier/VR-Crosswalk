
void fLogging () {
  unsigned long lm;
  unsigned long lS;
  unsigned long lM;
  unsigned long lH;
  char cH[3];
  char cM[3];
  char cS[3];
  char cm[4];
  char cTime[13];

  lm = millis ();         // Example: 12636254
  lS = (lm/1000) % 60; 
  lM = (lm/60000) % 60;
  lH = (lm/3600000) % 24;
  lm = lm - (lS*1000) - (lM*60000) - (lH*3600000);

  itoa (lH,cH,10);
  itoa (lM,cM,10);
  itoa (lS,cS,10);
  itoa (lm,cm,10);
  if (lH<10) { cTime[0]='0'; cTime[1]=cH[0]; } else { cTime[0]=cH[0]; cTime[1]=cH[1]; }
  cTime[2]=':';
  if (lM<10) { cTime[3]='0'; cTime[4]=cM[0]; } else { cTime[3]=cM[0]; cTime[4]=cM[1]; }
  cTime[5]=':';
  if (lS<10) { cTime[6]='0'; cTime[7]=cS[0]; } else { cTime[6]=cS[0]; cTime[7]=cS[1]; }
  cTime[8]='.';
  cTime[9]=cm[0];
  cTime[10]=cm[1];
  cTime[11]=cm[2];
  cTime[12]=cm[3];

  if (Logging.bMotor_LastPWMValue!=Motor.bLastPWMValue)
  {
    Logging.bMotor_LastPWMValue=Motor.bLastPWMValue;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";M_PWM;"); Serial.println (Logging.bMotor_LastPWMValue);
  }
  if (Logging.iMotor_TargetPosition!=Motor.iTargetPosition)
  {
    Logging.iMotor_TargetPosition=Motor.iTargetPosition;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";M_TAR;"); Serial.println (Logging.iMotor_TargetPosition);    
  }
  if (Logging.iMotor_EncoderPosition!=Motor.iEncoderPosition)
  {
    Logging.iMotor_EncoderPosition=Motor.iEncoderPosition;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";M_ENC;"); Serial.println (Logging.iMotor_EncoderPosition);    
  }
  if (Logging.iSteppermotor_CurrentPosition!=Steppermotor.iCurrentPosition)
  {
    Logging.iSteppermotor_CurrentPosition=Steppermotor.iCurrentPosition;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";S_POS;"); Serial.println (Logging.iSteppermotor_CurrentPosition);    
  }
  if (Logging.iSteppermotor_TargetPosition!=Steppermotor.iTargetPosition)
  {
    Logging.iSteppermotor_TargetPosition=Steppermotor.iTargetPosition;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";S_TAR;"); Serial.println (Logging.iSteppermotor_TargetPosition);    
  }
  if ((abs (Logging.FootplateRight_SensorOffset.X-FootprintRight.SensorOffset.X)>1) ||
      (abs(Logging.FootplateRight_SensorOffset.Y-FootprintRight.SensorOffset.Y)>1))
  {
    Logging.FootplateRight_SensorOffset.X=FootprintRight.SensorOffset.X;
    Logging.FootplateRight_SensorOffset.Y=FootprintRight.SensorOffset.Y;
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";F_X;"); Serial.println (Logging.FootplateRight_SensorOffset.X);    
    Serial.print ("01.01.2019;");
    Serial.print (cTime); Serial.print (";F_Y;"); Serial.println (Logging.FootplateRight_SensorOffset.Y);    
  }
}
