void setup(){
  Serial.begin(115200);
  while(!Serial)
  {
    delay (1);
  };
  // I2C stuff:
  nI2C->SetTimeoutMS(25);
//  FootprintLeft.I2C_TCA9548A_Upper  = nI2C->RegisterDevice(0x70, 1, CI2C::Speed::FAST);
//  FootprintLeft.I2C_TCA9548A_Lower  = nI2C->RegisterDevice(0x71, 1, CI2C::Speed::FAST);
  FootprintRight.I2C_TCA9548A_Upper = nI2C->RegisterDevice(0x74, 1, CI2C::Speed::FAST);
  FootprintRight.I2C_TCA9548A_Lower = nI2C->RegisterDevice(0x75, 1, CI2C::Speed::FAST);
  I2C_GY271                         = nI2C->RegisterDevice(0x0D, 1, CI2C::Speed::FAST);
//  I2C_VL6180X                       = nI2C->RegisterDevice(0x29, 2, CI2C::Speed::FAST);
//  fTCA9548A_Disable_I2C_BusDevice (FootprintLeft.I2C_TCA9548A_Upper);
//  fTCA9548A_Disable_I2C_BusDevice (FootprintLeft.I2C_TCA9548A_Lower);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Upper);
  fTCA9548A_Disable_I2C_BusDevice (FootprintRight.I2C_TCA9548A_Lower);
  fFootprint_Init ();
  
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_A), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  attachInterrupt (digitalPinToInterrupt(PIN_BASEPLATE_ENCODER_B), DC_PIDMOTOR_EncoderInterrupt, CHANGE);
  
  fMotor_Init ();
  fMotor_TurnOff ();
  
  fSteppermotor_Init ();
  fSteppermotor_TurnOff ();
  
  fCNC_SetCurrent (30.0,0.0); // Units in mm
  fCNC_SetDestination  (30.0,0.0); // Units in mm
  Steppermotor.iCurrentPosition=30*5; // When power up, the footplate is 30mm away from the center. Each step is ~0,2mm. So 30*5 steps=30mm
  Steppermotor.iTargetPosition=30*5;  // When power up, the footplate is 30mm away from the center. Each step is ~0,2mm. So 30*5 steps=30mm
  
  // Millisecond timer interrupt
  OCR0A = 0x7D;
  TIMSK0 |= _BV(OCIE0A); 
}
