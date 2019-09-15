void fTCA9548A_Disable_I2C_BusDevice (CI2C::Handle bModule) {
  byte bStatus;
  bSTOP_CurrentTCA9548A = bModule.device_address;
  bSTOP_CurrentTCA9548A_Port=0;
  if (bStatus=nI2C->Write (bModule, &cTCA9548A_SendBuffer[0], 1)!=0) fSTOP_I2C (bModule.device_address,bStatus,"fTCA9548A_Disable_I2C_BusDevice");
}

void fTCA9548A_Select_I2C_BusDevice (CI2C::Handle bModule, byte bPort) {
  static CI2C::Handle PreviousModule;
  byte bStatus;
  if ((PreviousModule.device_address>0)&&(PreviousModule.device_address!=bModule.device_address))
  { // Disable the previous Module:
    fTCA9548A_Disable_I2C_BusDevice (PreviousModule);
  }
  PreviousModule = bModule;
  bSTOP_CurrentTCA9548A = bModule.device_address;
  bSTOP_CurrentTCA9548A_Port=1<<bPort;
  if (bStatus=nI2C->Write (bModule, &cTCA9548A_SendBuffer[bPort+1], 1)!=0) fSTOP_I2C (bModule.device_address,bStatus,"fTCA9548A_Select_I2C_BusDevice");
}
