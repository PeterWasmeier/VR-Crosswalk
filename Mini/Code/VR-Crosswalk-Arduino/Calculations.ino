void RotatePoint (int iPointX, int iPointY, double dDegree, int *iResultX, int *iResultY)
{
  // Calculate the current angle:
  double dRadius;
  double dCurrentAngleRadiant;
  double dCurrentAngle;
  double dDestinationAgle;
  double dDestinationAgleRadiant;
  double dX;
  double dY;
  double dPointX;

  dRadius = sqrt ( iPointX*iPointX + iPointY*iPointY );
  if (dRadius==0.0)
  {
    *iResultX = iPointX;
    *iResultY = iPointY;
    return;
  }
  dPointX=iPointX;
  dCurrentAngleRadiant = acos ( dPointX / dRadius );
  if (iPointY<0) dCurrentAngleRadiant=-dCurrentAngleRadiant;
  dCurrentAngle = (dCurrentAngleRadiant / M_PI) * 180;
  dDestinationAgle = dCurrentAngle + dDegree;
  dDestinationAgleRadiant = dDestinationAgle * (M_PI / 180.0);

  dX = cos ( dDestinationAgleRadiant ) * dRadius;
  dY = sin ( dDestinationAgleRadiant ) * dRadius;

  *iResultX = dX;
  *iResultY = dY;
}

void fAverageMedianXY (int *iValueX, int *iValueY)
{
  static int iPreviousValueX;
  static int iPreviousValueY;
  static int iMedianX[5];
  static int iMedianY[5];
  static byte bIndex=0;
  byte bSorted[5]={0,1,2,3,4};
  byte bi;
  byte bTemp;
  iMedianX[bIndex]=*iValueX;
  iMedianY[bIndex]=*iValueY;
  bIndex++;
  if (bIndex>4) bIndex=0;
  for (bi=0;bi<4;bi++)
  {
    if (iMedianX[bSorted[bi]]>iMedianX[bSorted[bi+1]]) {
      bTemp=bSorted[bi];
      bSorted[bi]=bSorted[bi+1];
      bSorted[bi+1]=bTemp;
      bi=255;
    }
  }
  *iValueX=iMedianX[bSorted[2]];
  *iValueX=(*iValueX + iPreviousValueX) >> 1;
  iPreviousValueX = *iValueX;
  for (bi=0;bi<4;bi++)
  {
    if (iMedianY[bSorted[bi]]>iMedianY[bSorted[bi+1]]) {
      bTemp=bSorted[bi];
      bSorted[bi]=bSorted[bi+1];
      bSorted[bi+1]=bTemp;
      bi=255;
    }
  }
  *iValueY=iMedianY[bSorted[2]];
  *iValueY=(*iValueY + iPreviousValueY) >> 1;
  iPreviousValueY = *iValueY;
}

bool fCalculate_Intersection (double a1, double b1, double S1x, double S1y, double a2, double b2, double S2x, double S2y, tPositionDouble *Position) {
  double C1;
  double C2;
  double M1;
  double X;
  double Y;
  double divisor;
  if ((S1x==0)&&(S2x==0)) return false; // We can not calculate, otherwise we will have a "Division by zero"
  if (S1x==0)                           // The first line is vertical, so use this method to calculate:
  {
    C2 = b2 - (S2y/S2x) * (a2 - a1);
    X  = a1;
    Y  = C2;
  }
  else
  if (S2x==0)                           // The second line is vertical, so use this method to calculate:
  {
    C1 = b1 - (S1y/S1x) * (a1 - a2);
    X  = a2;
    Y  = C1;    
  }
  else                                  // None of the lines is vertical, so use this method to calculate:
  {
    divisor = (S1y/S1x) - (S2y/S2x);
    if (divisor==0) return false;
    C1 = b1 - ((S1y / S1x) * a1);
    M1 = (S1y / S1x);
    X = ( b2 - (S2y*a2/S2x) - ( b1 - (S1y*a1/S1x) ) ) / ( divisor );
    Y = M1 * X + C1;
  }
  // Return the Result:
  Position->X = X;
  Position->Y = Y;
  return true;
}

double fCalculate_Distance (double x1, double y1, double x2, double y2) {
  return sqrt( ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)) );
}

bool fCalculate_GY271_CenterOfFootplate (tFootplate *Footplate) {
  tPositionDouble Intersection_S0S1;
  tPositionDouble Intersection_S0S2;
  tPositionDouble Intersection_S1S2;
  tPositionDouble Intersection_S3S4;
  tPositionDouble Intersection_S3S5;
  tPositionDouble Intersection_S4S5;
  tPositionDouble NorthPole;
  tPositionDouble SouthPole;

  // UPPER FOOTPLATE:
  
  // Calculate the intersection between 1.Sensor versus 2.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[0], GY271_Sensor_Y[0], Footplate->GY271[0].Value.X, Footplate->GY271[0].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[1], GY271_Sensor_Y[1], Footplate->GY271[1].Value.X, Footplate->GY271[1].Value.Y,  // versus 2.Sensor
                               &Intersection_S0S1)==false) return false;
  
  // Calculate the intersection between 1.Sensor versus 3.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[0], GY271_Sensor_Y[0], Footplate->GY271[0].Value.X, Footplate->GY271[0].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[2], GY271_Sensor_Y[2], Footplate->GY271[2].Value.X, Footplate->GY271[2].Value.Y,  // versus 2.Sensor
                               &Intersection_S0S2)==false) return false;
  
  // Calculate the intersection between 2.Sensor versus 3.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[1], GY271_Sensor_Y[1], Footplate->GY271[1].Value.X, Footplate->GY271[1].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[2], GY271_Sensor_Y[2], Footplate->GY271[2].Value.X, Footplate->GY271[2].Value.Y,  // versus 2.Sensor
                               &Intersection_S1S2)==false) return false;
                               
  // LOWER FOOTPLATE:
  
  // Calculate the intersection between 4.Sensor versus 5.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[3], GY271_Sensor_Y[3], Footplate->GY271[3].Value.X, Footplate->GY271[3].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[4], GY271_Sensor_Y[4], Footplate->GY271[4].Value.X, Footplate->GY271[4].Value.Y,  // versus 2.Sensor
                               &Intersection_S3S4)==false) return false;
  
  // Calculate the intersection between 4.Sensor versus 6.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[3], GY271_Sensor_Y[3], Footplate->GY271[3].Value.X, Footplate->GY271[3].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[5], GY271_Sensor_Y[5], Footplate->GY271[5].Value.X, Footplate->GY271[5].Value.Y,  // versus 2.Sensor
                               &Intersection_S3S5)==false) return false;
  
  // Calculate the intersection between 5.Sensor versus 6.Sensor:
  if (fCalculate_Intersection (GY271_Sensor_X[4], GY271_Sensor_Y[4], Footplate->GY271[4].Value.X, Footplate->GY271[4].Value.Y,  // 1.Sensor 
                               GY271_Sensor_X[5], GY271_Sensor_Y[5], Footplate->GY271[5].Value.X, Footplate->GY271[5].Value.Y,  // versus 2.Sensor
                               &Intersection_S4S5)==false) return false;

  // UPPER FOOTPLATE:
  
  double Distance_S0S1_S0S2;
  double Distance_S0S1_S1S2;
  double Distance_S0S2_S1S2;
  double Distance_S3S4_S3S5;
  double Distance_S3S4_S4S5;
  double Distance_S3S5_S4S5;
  double Distance_South_North;
  double dAlphaRadiant;
  double dAlpha;
  
  // Calculate the distance between each intersection:

  // UPPER FOOTPLATE:
  
  Distance_S0S1_S0S2 = fCalculate_Distance ( Intersection_S0S1.X, Intersection_S0S1.Y,
                                             Intersection_S0S2.X, Intersection_S0S2.Y);
  Distance_S0S1_S1S2 = fCalculate_Distance ( Intersection_S0S1.X, Intersection_S0S1.Y,
                                             Intersection_S1S2.X, Intersection_S1S2.Y);
  Distance_S0S2_S1S2 = fCalculate_Distance ( Intersection_S0S2.X, Intersection_S0S2.Y,
                                             Intersection_S1S2.X, Intersection_S1S2.Y);
  // LOWER FOOTPLATE:
  Distance_S3S4_S3S5 = fCalculate_Distance ( Intersection_S3S4.X, Intersection_S3S4.Y,
                                             Intersection_S3S5.X, Intersection_S3S5.Y);
  Distance_S3S4_S4S5 = fCalculate_Distance ( Intersection_S3S4.X, Intersection_S3S4.Y,
                                             Intersection_S4S5.X, Intersection_S4S5.Y);
  Distance_S3S5_S4S5 = fCalculate_Distance ( Intersection_S3S5.X, Intersection_S3S5.Y,
                                             Intersection_S4S5.X, Intersection_S4S5.Y);
  // Calculate the "north-pole":
  if ((Distance_S0S1_S0S2<Distance_S0S1_S1S2) && (Distance_S0S1_S0S2<Distance_S0S2_S1S2))
  { // Distance_S0S1_S0S2 is the smalest one:
    NorthPole.X = (Intersection_S0S1.X + Intersection_S0S2.X) / 2;
    NorthPole.Y = (Intersection_S0S1.Y + Intersection_S0S2.Y) / 2;
  }
  else
  if ((Distance_S0S1_S1S2<Distance_S0S1_S0S2) && (Distance_S0S1_S1S2<Distance_S0S2_S1S2))
  { // Distance_S0S2 is the smalest one:
    NorthPole.X = (Intersection_S0S1.X + Intersection_S1S2.X) / 2;
    NorthPole.Y = (Intersection_S0S1.Y + Intersection_S1S2.Y) / 2;
  }
  else
  if ((Distance_S0S2_S1S2<Distance_S0S1_S0S2) && (Distance_S0S2_S1S2<Distance_S0S1_S1S2))
  { // Distance_S1S2 is the smalest one:
    NorthPole.X = (Intersection_S0S2.X + Intersection_S1S2.X) / 2;
    NorthPole.Y = (Intersection_S0S2.Y + Intersection_S1S2.Y) / 2;
  }

  // Calculate the "south-pole":

  if ((Distance_S3S4_S3S5<Distance_S3S4_S4S5) && (Distance_S3S4_S3S5<Distance_S3S5_S4S5))
  { // Distance_S3S4 is the smalest one:
    SouthPole.X = (Intersection_S3S4.X + Intersection_S3S5.X) / 2;
    SouthPole.Y = (Intersection_S3S4.Y + Intersection_S3S5.Y) / 2;
  }
  else
  if ((Distance_S3S4_S4S5<Distance_S3S4_S3S5) && (Distance_S3S4_S4S5<Distance_S3S5_S4S5))
  { // Distance_S3S5 is the smalest one:
    SouthPole.X = (Intersection_S3S4.X + Intersection_S4S5.X) / 2;
    SouthPole.Y = (Intersection_S3S4.Y + Intersection_S4S5.Y) / 2;
  }
  else
  if ((Distance_S3S5_S4S5<Distance_S3S4_S3S5) && (Distance_S3S5_S4S5<Distance_S3S4_S4S5))
  { // Distance_S4S5 is the smalest one:
    SouthPole.X = (Intersection_S3S5.X + Intersection_S4S5.X) / 2;
    SouthPole.Y = (Intersection_S3S5.Y + Intersection_S4S5.Y) / 2;
  }    

  // CALCULATE CENTER OF FOOTPLATE:

  Footplate->SensorOffset.X = (NorthPole.X + SouthPole.X) / 2;
  Footplate->SensorOffset.Y = (NorthPole.Y + SouthPole.Y) / 2;

  // CALCULATE ANGLE OF FOOTPLATE
  Distance_South_North = fCalculate_Distance ( NorthPole.X, NorthPole.Y,
                                               SouthPole.X, SouthPole.Y);
  dAlphaRadiant = acos ((NorthPole.X-SouthPole.X)/Distance_South_North);
  dAlpha = (dAlphaRadiant / PI) * 180;                   // Result will be -90° ... +90°
  Footplate->Alpha = dAlpha;
  
  return true;
}
