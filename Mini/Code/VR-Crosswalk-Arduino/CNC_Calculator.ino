
void fCNC_SetCurrent (int CurrentX, int CurrentY) {
  CNC.iPreviousDestinationX = CurrentX;
  CNC.iPreviousDestinationY = CurrentY;
}

/* fCNC_SetDestination
 *  Use this function to tell both axis where to go.
 *  Parameters:
 *  DestinationX/DestinationY: position of the right footplate in mm where to go
 */
void fCNC_SetDestination (int DestinationX, int DestinationY) {
  if ((CNC.iDestinationX!=DestinationX)||(CNC.iDestinationY!=DestinationY))
  {
    CNC.iPreviousDestinationX = CNC.iDestinationX;
    CNC.iPreviousDestinationY = CNC.iDestinationY;
    CNC.iDestinationX = DestinationX;
    CNC.iDestinationY = DestinationY;
    fMotor_setTargetPosition ( fCNC_CalculatePrimaryAxis () );
    Steppermotor_iTargetPosition = fCNC_CalculateSecondaryAxis(Motor.iTargetPosition/*Motor.iEncoderPosition*/);
    sStepper.writeSteps (Steppermotor_iTargetPosition);    
    fMotor_TurnOn();
  }
}

/* fCNC_GetCurrentXY
 *  Calculates, depending on the current position of the primary and secondary axis,
 *  where the footplate is right now located.
 *  
 *  Parameter:
 *  lCurrentPositionPrimaryAxis: current encoder position of the primary axis
 *  lCurrentPositionSecondaryAxis: current position of the secondary axis
 *  *X/*Y: position in mm where the right footplate is located right now
 */
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

/* fCNC_CalculateDegree
 *  Calculate the amount of degree, from 0 to 359, depending
 *  on dX and dY, Zero degree = horizontal to the right.
 *  It is "rotating" counter clockwise.
 *  
 *  Parameters:
 *  dX: Position in X direction in mm
 *  dY: Position in Y direction in mm
 */
double fCNC_CalculateDegree (double dX, double dY) {
  double dAlphaRadiant;
  double dAlpha;
  if (dX==0)
  {
    if (dY>0)
      return 90;
    else
      return 270;
  }
  dAlphaRadiant = atan (dY / dX);
  dAlpha = (dAlphaRadiant / PI) * 180;
  if (dX<0)
    dAlpha = 180 + dAlpha;
  if (dAlpha<0)
    dAlpha = 360 + dAlpha;
  return dAlpha;
}

/* fCNC_CalculatePrimaryAxis
 *  Calculate where the primary axis has to rotate to,
 *  depending on CNC.iDestinationX and CNC.iDestinationY.
 *  
 *  Parameters:
 *  CNC.iDestinationX/CNC.iDestinationY: destination position of the foot
 */
int fCNC_CalculatePrimaryAxis () {
  // Where to go with the primary axis?
  double dAlphaRadiant;
  double dAlphaDestination;
  double dAlphaDeltaLeftTurn;
  double dAlphaDeltaRightTurn;
  double dAlpha;
  double dPosition;
  double dX;
  double dY;
  int iPosition;
  dAlphaDestination = fCNC_CalculateDegree (CNC.iDestinationX, CNC.iDestinationY);
  dPosition = (ENCODER_PULSES_PER_90DEGREE / 90.0) * dAlphaDestination;
  iPosition = dPosition;
  return iPosition;
}

/* fCNC_CalculateSecondaryAxis
 *  Calculate the position of the secondary axis, depending on the
 *  current position of the primary axis. The secondary axis has
 *  to follow the primary one.
 *  Parameter:
 *  iCurrentPositionPrimaryAxis: current position of the primary axis in units
 */
int fCNC_CalculateSecondaryAxis (int iCurrentPositionPrimaryAxis) {
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

  dAlpha = (90.0 / ENCODER_PULSES_PER_90DEGREE) * iCurrentPositionPrimaryAxis; // Result will be -90° ... +90° (-600 ... +600 Units)
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
