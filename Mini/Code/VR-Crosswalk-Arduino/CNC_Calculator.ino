
void fCNC_SetCurrent (double CurrentX, double CurrentY) {
  CNC.dDestinationX = CurrentX;
  CNC.dDestinationY = CurrentY;
}

void fCNC_SetDestination (double DestinationX, double DestinationY) {
  CNC.dDestinationX = DestinationX;
  CNC.dDestinationY = DestinationY;
}

void fCNC_GetCurrentXY (long lCurrentPositionPrimaryAxis, long lCurrentPositionSecondaryAxis, double *X, double *Y) {
  double dAlpha;
  double dAlphaRadiant;
  double dX;
  double dY;
  double dCurrentPositionSecondaryAxis;
  dCurrentPositionSecondaryAxis =  lCurrentPositionSecondaryAxis / 5; // Result is now in mm
  dAlpha = (90.0 / ENCODER_PULSES_PER_90DEGREE) * lCurrentPositionPrimaryAxis; // Result will be -90° ... +90° (-600 ... +600 Units)
  dAlphaRadiant = dAlpha * (PI / 180.0);                 // Result is now in radiant
  dY = sin(dAlphaRadiant) * dCurrentPositionSecondaryAxis;
  dX = cos(dAlphaRadiant) * dCurrentPositionSecondaryAxis;
  *X = dX;
  *Y = dY;
}
  
long fCNC_CalculatePrimaryAxis () {
  // Where to go with the primary axis?
  double dAlphaRadiant;
  double dAlpha;
  double dPosition;
  long lPosition;
  if (CNC.dDestinationX==0.0) 
  {
    if (CNC.dDestinationY>0) return ENCODER_PULSES_PER_90DEGREE; // +90°
    if (CNC.dDestinationY<0) return -ENCODER_PULSES_PER_90DEGREE; // -90°
    return 0; // This might be wrong and should never happen, have to fix this one later XXX
  }
  dAlphaRadiant = atan (CNC.dDestinationY / CNC.dDestinationX);  // Result will be -PI/2 .... +PI/2  
  dAlpha = (dAlphaRadiant / PI) * 180;                   // Result will be -90° ... +90°
  dPosition = (ENCODER_PULSES_PER_90DEGREE / 90.0) * dAlpha;                   // Convert from degree to Units (-600 ... +600)
  lPosition = dPosition;
  return lPosition;
}

long fCNC_CalculateSecondaryAxis (long lCurrentPositionPrimaryAxis) {
  // Where to go with the secondary axis?
  double dAlpha;
  double dAlphaRadiant;
  double dTanAlpha;
  double dM;
  double dB;
  double dX;
  double dY;
  double dResult;
  long lResult;
  double dX1;
  double dY1;
  double dX2;
  double dY2;
  double dTanAlpha_Minus_dM;
  double dX2_Minus_dX1;
  double dY2_Minus_dY1;
  
  dX1 = 30.0;
  dY1 = 0;
  dX2 = CNC.dDestinationX;
  dY2 = CNC.dDestinationY; 

  dAlpha = (90.0 / ENCODER_PULSES_PER_90DEGREE) * lCurrentPositionPrimaryAxis; // Result will be -90° ... +90° (-600 ... +600 Units)
  dAlphaRadiant = dAlpha * (PI / 180.0);                 // Result is now in radiant
  dX2_Minus_dX1 = dX2-dX1;  

  if (dX2_Minus_dX1==0.0) {                                 // 90° direction?
    // Special case: source and target is on the same Y axis:
    dResult = dX2 / cos( dAlphaRadiant );                   
    lResult = dResult * 5;                                  // Convert from mm to Units
  }
  else
  {
    dY2_Minus_dY1 = dY2 - dY1;
    dM = (dY2_Minus_dY1) / dX2_Minus_dX1;
    dTanAlpha = tan ( dAlphaRadiant );
    dB = dY1 - (dM * dX1);
    dTanAlpha_Minus_dM = dTanAlpha-dM;
    if (dTanAlpha_Minus_dM==0.0)
    {
      // Special case, at alpha=0 and m=0: 
      lResult = CNC.dDestinationX * 5;                          // Convert from mm to Units
    }
    else
    {
      dX = dB / dTanAlpha_Minus_dM;    
      dY = (dTanAlpha * dB) / dTanAlpha_Minus_dM;
      dResult = sqrt ( (dX * dX) + (dY * dY) );
      lResult = dResult * 5;                                // Convert from mm to Units
    }
  }
  return lResult;
}
