void setup(){
  int X;
  int Y;
  Serial.begin(115200);
  while(!Serial)
  {
    delay (1);
  };
  // I2C stuff:
  nI2C->SetTimeoutMS(25);
  FootprintRight.I2C_TCA9548A_Upper = nI2C->RegisterDevice(0x74, 1, CI2C::Speed::FAST);
  FootprintRight.I2C_TCA9548A_Lower = nI2C->RegisterDevice(0x75, 1, CI2C::Speed::FAST);
  I2C_GY271                         = nI2C->RegisterDevice(0x0D, 1, CI2C::Speed::FAST);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower);
  fFootprint_Init ();
  
  fMotor_Init ();
  fMotor_TurnOff ();
    
  // Startposition=30/0:
  Motor.iEncoderPosition=0;
  Motor.iTargetPosition=0;  
  CNC.iDestinationX=30;
  CNC.iDestinationY=0;
  CNC.iPreviousDestinationX=30;
  CNC.iPreviousDestinationY=0;
  sStepper.attach (PIN_STEPPER_COIL2_MINUS,PIN_STEPPER_COIL1_MINUS,PIN_STEPPER_COIL2_PLUS,PIN_STEPPER_COIL1_PLUS);
  sStepper.setPosition (150);
  sStepper.setSpeed (400);
  // Interrupts for the encoder of the motor:
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), DC_PIDMOTOR_EncoderInterrupt, CHANGE);

}
