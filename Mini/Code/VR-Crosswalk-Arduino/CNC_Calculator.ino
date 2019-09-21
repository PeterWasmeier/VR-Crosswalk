
void fCNC_SetCurrent (int CurrentX, int CurrentY) {
  CNC.iPreviousDestinationX = CurrentX;
  CNC.iPreviousDestinationY = CurrentY;
}

void fCNC_SetDestination (int DestinationX, int DestinationY) {
  if ((CNC.iDestinationX!=DestinationX)||(CNC.iDestinationY!=DestinationY))
  {
    CNC.iPreviousDestinationX = CNC.iDestinationX;
    CNC.iPreviousDestinationY = CNC.iDestinationY;
    CNC.iDestinationX = DestinationX;
    CNC.iDestinationY = DestinationY;
  }
}

void fCNC_GetCurrentXY (long lCurrentPositionPrimaryAxis, long lCurrentPositionSecondaryAxis, int *X, int *Y) {
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

double fCNC_CalculateDegree (double dX, double dY)
{
  double dAlphaRadiant;
  double dAlpha;
  if (dX==0)
  {
    if (dY>0)
      return 90;
    else
      return 270;
  }
  dAlphaRadiant = atan (dY / dX);  // Result will be -PI/2 .... +PI/2
  dAlpha = (dAlphaRadiant / PI) * 180;                   // Result will be -90° ... +90°
  if (dX<0)
    dAlpha = 180 + dAlpha;
  if (dAlpha<0)
    dAlpha = 360 + dAlpha;
  return dAlpha;
}

int fCNC_CalculatePrimaryAxis () {
  // Where to go with the primary axis?
  double dAlphaRadiant;
  double dAlphaDestination;
  double dAlphaPreviousDestination;
  double dAlphaDeltaLeftTurn;
  double dAlphaDeltaRightTurn;
  double dAlpha;
  double dPosition;
  double dX;
  double dY;
  long lPosition;
  dAlphaPreviousDestination = fCNC_CalculateDegree (CNC.iPreviousDestinationX, CNC.iPreviousDestinationY);
  dAlphaDestination = fCNC_CalculateDegree (CNC.iDestinationX, CNC.iDestinationY);

  if (dAlphaDestination<dAlphaPreviousDestination) dAlphaDestination=dAlphaDestination+360;

  dAlphaDeltaLeftTurn=(dAlphaDestination-dAlphaPreviousDestination);
  dAlphaDeltaRightTurn=360-dAlphaDeltaLeftTurn;

  if (dAlphaDeltaLeftTurn<dAlphaDeltaRightTurn)
    dPosition = Motor.iTargetPosition + ((ENCODER_PULSES_PER_90DEGREE / 90.0) * dAlphaDeltaLeftTurn);
  else
    dPosition = Motor.iTargetPosition - ((ENCODER_PULSES_PER_90DEGREE / 90.0) * dAlphaDeltaRightTurn);
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
  
  dX1 = CNC.iPreviousDestinationX;
  dY1 = CNC.iPreviousDestinationY;
  dX2 = CNC.iDestinationX;
  dY2 = CNC.iDestinationY; 

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
      // Special case: the primary axis is parallel to the route from source to target. So, no solution, return the current value: 
      lResult = CNC.iDestinationX * 5;
    }
    else
    {
      dX = dB / dTanAlpha_Minus_dM;
      dY = (dTanAlpha * dB) / dTanAlpha_Minus_dM;
      dResult = sqrt ( (dX * dX) + (dY * dY) );
      lResult = dResult * 5;                                // Convert from mm to Units
      // Keep in mind: there might be a special case, when the primary axis is not cutting the route, in this case the result will be zero
    }
  }
  return lResult;
}
