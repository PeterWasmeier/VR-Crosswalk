//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop
#include <math.h>
#include "Unit1.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TForm1 *Form1;


#define GY271_Sensor1_X -(40.64/2)
#define GY271_Sensor1_Y -(143.149/2)+21.23

#define GY271_Sensor2_X 0.0
#define GY271_Sensor2_Y -(143.149/2)

#define GY271_Sensor3_X (40.64/2)
#define GY271_Sensor3_Y -(143.149/2)+21.23

#define GY271_Sensor4_X -(40.64/2)
#define GY271_Sensor4_Y (143.149/2)-22.86

#define GY271_Sensor5_X 0.0
#define GY271_Sensor5_Y (143.149/2)

#define GY271_Sensor6_X (40.64/2)
#define GY271_Sensor6_Y (143.149/2)-22.86

void TForm1::CloseComm(HANDLE hCom)
{
  PurgeComm(hCom, PURGE_RXABORT);
  CloseHandle(hCom);
}

HANDLE TForm1::OpenComm(char *Port)
{
  if (hCom!=0)
  {
    CloseComm (hCom);
  }
  HANDLE hCom2 = ::CreateFile(Port,
                             GENERIC_READ | GENERIC_WRITE,
                             0,
                             0,
                             OPEN_EXISTING,
                             FILE_ATTRIBUTE_NORMAL,
                             0);

  if (hCom2 == INVALID_HANDLE_VALUE)
  {
    // ... Fehler
    MessageBox (0,"Com-Port not available","Error",0);
    return 0;
  }
  iReceiveDataLength=0;
  return hCom2;
}

void TForm1::SetDeviceControlBlock(HANDLE hCom, DWORD BaudRate, BYTE ByteSize, BYTE Parity, BYTE StopBits)
{
  if (hCom == INVALID_HANDLE_VALUE)
    return;

  DCB dcb;

  // Schnittstelle auslesen
  if (!::GetCommState(hCom, &dcb))
  {
    // ... Fehler
  }

  dcb.BaudRate  = BaudRate;
  dcb.ByteSize  = ByteSize;
  dcb.Parity    = Parity;
  dcb.StopBits  = StopBits;

  // Schnittstelle konfigurieren
  if (!::SetCommState(hCom, &dcb))
  {
    // ... Fehler
  }
}

int TForm1::SendData(HANDLE hCom, char *Data, int Length)
{
  DWORD NumberOfBytesWritten;

  if (!::WriteFile(hCom, Data, Length, &NumberOfBytesWritten, 0))
  {
    // ... Fehler
  }

  return NumberOfBytesWritten;
}

DWORD TForm1::ReceiveData(HANDLE hCom, char *Data, int Length)
{
  DWORD NumberOfBytesRead;

  if (!::ReadFile(hCom, Data, Length, &NumberOfBytesRead, 0))
  {
    // ... Fehler
  }

  return NumberOfBytesRead;
}

//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
        : TForm(Owner)
{
  hCom = 0;
  iOffsetX=0;
  iOffsetY=0;
  iOffsetZ=0;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::COM11Click(TObject *Sender)
{
  // COM-Port öffnen und initialisieren
  if ((hCom = OpenComm("COM1")) != INVALID_HANDLE_VALUE)
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::COM21Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM2")) != INVALID_HANDLE_VALUE)
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::COM31Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM3")) != INVALID_HANDLE_VALUE)
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::COM41Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM4")) != INVALID_HANDLE_VALUE)
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::COM51Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM5")) != INVALID_HANDLE_VALUE)
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
}

//---------------------------------------------------------------------------
void __fastcall TForm1::fCircle (int X, int Y, int R)
{
  PaintBox1->Canvas->Ellipse(X-R,Y-R,X+R,Y+R);
}


void __fastcall TForm1::fDrawLine (double X1, double Y1, double X2, double Y2, TColor color)
{
  // The origin is in the center of the screen:
  int MiddleX = PaintBox1->Width / 2;
  int MiddleY = PaintBox1->Height / 2;
  // Convert "mm" to "Pixel"
  int x1 = X1 * tbZoom->Position; // 1mm = 3 Pixel
  int y1 = Y1 * tbZoom->Position; // 1mm = 3 Pixel
  int x2 = X2 * tbZoom->Position; // 1mm = 3 Pixel
  int y2 = Y2 * tbZoom->Position; // 1mm = 3 Pixel

  x1 = -x1; // Invert X, because of Screen
  x2 = -x2; // Invert X, because of Screen

  x1 = x1 + MiddleX;
  y1 = y1 + MiddleY;
  x2 = x2 + MiddleX;
  y2 = y2 + MiddleY;
  PaintBox1->Canvas->Pen->Color = color;
  PaintBox1->Canvas->MoveTo(x1,y1);
  PaintBox1->Canvas->LineTo(x2,y2);
}

void __fastcall TForm1::fDrawIntersection (double X, double Y, TColor color)
{
  // The origin is in the center of the screen:
  int MiddleX = PaintBox1->Width / 2;
  int MiddleY = PaintBox1->Height / 2;
  // Convert "mm" to "Pixel"
  int x = X * tbZoom->Position; // 1mm = 3 Pixel
  int y = Y * tbZoom->Position; // 1mm = 3 Pixel

  x = -x; // Invert X, because of Screen

  x = x + MiddleX;
  y = y + MiddleY;
  PaintBox1->Canvas->Brush->Color = color;
  PaintBox1->Canvas->Pen->Color = clBlack;
  fCircle(x,y,5);
}

void __fastcall TForm1::fDrawMagnet (double X, double Y, int Sx, int Sy)
{
  // The origin is in the center of the screen:
  int MiddleX = PaintBox1->Width / 2;
  int MiddleY = PaintBox1->Height / 2;
  // Convert "mm" to "Pixel"
  int x = X * tbZoom->Position; // 1mm = 3 Pixel
  int y = Y * tbZoom->Position; // 1mm = 3 Pixel

  x = -x; // Invert X, because of Screen

  x = x + MiddleX;
  y = y + MiddleY;

  PaintBox1->Canvas->Brush->Color = clWhite;
  PaintBox1->Canvas->Pen->Color = clBlack;
  fCircle(x,y,20);

  /*PaintBox1->Canvas->MoveTo(x,y);
  PaintBox1->Canvas->LineTo(x-Sx,y+Sy);

  PaintBox1->Canvas->MoveTo(x,y);
  PaintBox1->Canvas->LineTo(x+Sx,y-Sy);
  */
}

//---------------------------------------------------------------------------
void __fastcall TForm1::fDrawFoorprint (int X1, int Y1, int Z1,
                                        int X2, int Y2, int Z2,
                                        int X3, int Y3, int Z3,
                                        int X4, int Y4, int Z4,
                                        int X5, int Y5, int Z5,
                                        int X6, int Y6, int Z6)
{
  int iWidth;
  int iHeight;
  iWidth = PaintBox1->Width;
  iHeight = PaintBox1->Height;
  PaintBox1->Canvas->Brush->Color = clWhite;
  PaintBox1->Canvas->FillRect(TRect(0,0,iWidth,iHeight));

  fDrawLine (-(56.0/2),-(158/2),56.0/2,-(158/2),clBlue);
  fDrawLine (56.0/2,-(158/2),56.0/2,158/2,clBlue);
  fDrawLine (56.0/2,158/2,-(56.0/2),158/2,clBlue);
  fDrawLine (-(56.0/2),158/2,-(56.0/2),-(158/2),clBlue);
  // Top 3:
  fDrawMagnet ( GY271_Sensor1_X, GY271_Sensor1_Y, X1, Y1);
  fDrawMagnet ( GY271_Sensor2_X, GY271_Sensor2_Y, X2, Y2);
  fDrawMagnet ( GY271_Sensor3_X, GY271_Sensor3_Y, X3, Y3);

  // Lower 3:
  fDrawMagnet ( GY271_Sensor4_X, GY271_Sensor4_Y, X4, Y4);
  fDrawMagnet ( GY271_Sensor5_X, GY271_Sensor5_Y, X5, Y5);
  fDrawMagnet ( GY271_Sensor6_X, GY271_Sensor6_Y, X6, Y6);

}

bool __fastcall TForm1::fCalculateIntersection (double a1, double b1, double S1x, double S1y,
                                                double a2, double b2, double S2x, double S2y,
                                                double *x, double *y)
{
  double X;
  double Y;
  double C1;
  double C2;
  double M1;
  if ((S1x==0)&&(S2x==0)) return false;
  if (S1x==0)
  {
    try
    {
      C2 = b2 - (S2y/S2x) * (a2 - a1);
    }
    catch (...)
    {
      MessageBox(0,"1","Catch",0);
    }
    X = a1;
    Y = C2;
  }
  else
  if (S2x==0)
  {
    try
    {
      C1 = b1 - (S1y/S1x) * (a1 - a2);
    }
    catch (...)
    {
      MessageBox(0,"2","Catch",0);
    }
    X = a2;
    Y = C1;
  }
  else
  {
    try
    {
      C1 = b1 - ((S1y / S1x) * a1);
    }
    catch (...)
    {
      MessageBox(0,"3","Catch",0);
    }
    try
    {
      M1 = (S1y / S1x);
    }
    catch (...)
    {
      MessageBox(0,"4","Catch",0);
    }
    double d=(S1y/S1x) - (S2y/S2x);
    if (d==0)
    {
      return false;
    }
    try
    {
      X = ( b2 - (S2y*a2/S2x) - ( b1 - (S1y*a1/S1x) ) ) / ( (S1y/S1x) - (S2y/S2x) );
    }
    catch (...)
    {
      MessageBox(0,"5","Catch",0);
    }
    Y = M1 * X + C1;
  }
  // Return the Result:
  *x = X;
  *y = Y;
  return true;
}

//---------------------------------------------------------------------------
void __fastcall TForm1::fInterpreteReceivedData (char *cChar)
{
  AnsiString sString;
  AnsiString sS[6];
  AnsiString sCenterX;
  AnsiString sCenterY;
  long lX[6];
  long lY[6];
  long lZ[6];
  bool bValid;
  sString = cChar;
  sString=StringReplace(sString,"\t"," ",TReplaceFlags() << rfReplaceAll);
  StatusBar1->SimpleText=sString;
  bValid=true;
  for (int i=0;i<6;i++)
  {
    try
    {
      sS[i]=sString.SubString(1,sString.Pos(' ')-1);
      sString = sString.SubString (sString.Pos(' ')+1,999);
      lX[i]=StrToInt(sS[i]);

      sS[i]=sString.SubString(1,sString.Pos(' ')-1);
      sString = sString.SubString (sString.Pos(' ')+1,999);
      lY[i]=StrToInt(sS[i]);

      sS[i]=sString.SubString(1,sString.Pos(' ')-1);
      sString = sString.SubString (sString.Pos(' ')+1,999);
      lZ[i]=StrToInt(sS[i]);
    }
    catch (...)
    {
      bValid=false;
    }
  }

  sCenterX=sString.SubString(1,sString.Pos(' ')-1);
  sString = sString.SubString (sString.Pos(' ')+1,999);

  sCenterY=sString.SubString(1,sString.Pos(' ')-1);
  sString = sString.SubString (sString.Pos(' ')+1,999);

  if (bValid==false) return;

  fDrawFoorprint (lX[0], lY[0], lZ[0],
                  lX[1], lY[1], lZ[1],
                  lX[2], lY[2], lZ[2],
                  lX[3], lY[3], lZ[3],
                  lX[4], lY[4], lZ[4],
                  lX[5], lY[5], lZ[5]);
  // Data available?
  if ((abs(lX[0])<1300) &&
      (abs(lY[0])<1300) &&
      (abs(lX[1])<1300) &&
      (abs(lY[1])<1300) &&
      (abs(lX[2])<1300) &&
      (abs(lY[2])<1300) &&
      (abs(lX[3])<1300) &&
      (abs(lY[3])<1300) &&
      (abs(lX[4])<1300) &&
      (abs(lY[4])<1300) &&
      (abs(lX[5])<1300) &&
      (abs(lY[5])<1300))
      return; // No Data available

  // Calculate and draw the intersections:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  double P1_x;
  double P2_x;
  double P3_x;
  double P4_x;
  double P5_x;
  double P6_x;
  double P1_y;
  double P2_y;
  double P3_y;
  double P4_y;
  double P5_y;
  double P6_y;

  double NorthPole_x;
  double NorthPole_y;
  double SouthPole_x;
  double SouthPole_y;

  double Foot_x;
  double Foot_y;

  // Intersectons of the upper 3 Sensors:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Intersection Sensor 1 and Sensor 2:
  if (fCalculateIntersection ( GY271_Sensor1_X, GY271_Sensor1_Y, lX[0], lY[0],
                           GY271_Sensor2_X, GY271_Sensor2_Y, lX[1], lY[1], &P1_x, &P1_y)==false) return;
  fDrawIntersection (P1_x,P1_y,clWhite);

  // Intersection Sensor 1 and Sensor 3:
  if (fCalculateIntersection ( GY271_Sensor1_X, GY271_Sensor1_Y, lX[0], lY[0],
                           GY271_Sensor3_X, GY271_Sensor3_Y, lX[2], lY[2], &P2_x, &P2_y)==false) return;
  fDrawIntersection (P2_x,P2_y,clWhite);

  // Intersection Sensor 2 and Sensor 3:
  if (fCalculateIntersection ( GY271_Sensor2_X, GY271_Sensor2_Y, lX[1], lY[1],
                           GY271_Sensor3_X, GY271_Sensor3_Y, lX[2], lY[2], &P3_x, &P3_y)==false) return;
  fDrawIntersection (P3_x,P3_y,clWhite);

  // Calculate the distance between P1, P2 and P3
  double Distance_P1P2;
  double Distance_P1P3;
  double Distance_P2P3;
  double ShortestDistance;

  double Faktor_P1P2;
  double Faktor_P1P3;
  double Faktor_P2P3;
  double Total_Distance;

  Distance_P1P2 = sqrt (((P1_x - P2_x)*(P1_x - P2_x)) + ((P1_y - P2_y)*(P1_y - P2_y)));
  Distance_P1P3 = sqrt (((P1_x - P3_x)*(P1_x - P3_x)) + ((P1_y - P3_y)*(P1_y - P3_y)));
  Distance_P2P3 = sqrt (((P2_x - P3_x)*(P2_x - P3_x)) + ((P2_y - P3_y)*(P2_y - P3_y)));

  Total_Distance = Distance_P1P2 + Distance_P1P3 + Distance_P2P3;

  Faktor_P1P2 = Total_Distance / Distance_P1P2;
  Faktor_P1P3 = Total_Distance / Distance_P1P3;
  Faktor_P2P3 = Total_Distance / Distance_P2P3;

  if (mnu2Of3->Checked==false)
  {
    NorthPole_x = (Faktor_P1P2 * ((P1_x + P2_x)/2) +
                   Faktor_P1P3 * ((P1_x + P3_x)/2) +
                   Faktor_P2P3 * ((P2_x + P3_x)/2) ) /
                   (Faktor_P1P2+Faktor_P1P3+Faktor_P2P3);

    NorthPole_y = (Faktor_P1P2 * ((P1_y + P2_y)/2) +
                   Faktor_P1P3 * ((P1_y + P3_y)/2) +
                   Faktor_P2P3 * ((P2_y + P3_y)/2)) /
                   (Faktor_P1P2+Faktor_P1P3+Faktor_P2P3);
  }
  else
  {
    if ((Distance_P1P2<Distance_P1P3) && (Distance_P1P2<Distance_P2P3))
    { // Distance_P1P2 is the smalest one:
      ShortestDistance = Distance_P1P2;
      NorthPole_x = (P1_x + P2_x) / 2;
      NorthPole_y = (P1_y + P2_y) / 2;
    }
    else
    if ((Distance_P1P3<Distance_P1P2) && (Distance_P1P3<Distance_P2P3))
    { // Distance_P1P3 is the smalest one:
      ShortestDistance = Distance_P1P3;
      NorthPole_x = (P1_x + P3_x) / 2;
      NorthPole_y = (P1_y + P3_y) / 2;
    }
    else
    if ((Distance_P2P3<Distance_P1P2) && (Distance_P2P3<Distance_P1P3))
    { // Distance_P2P3 is the smalest one:
      ShortestDistance = Distance_P2P3;
      NorthPole_x = (P2_x + P3_x) / 2;
      NorthPole_y = (P2_y + P3_y) / 2;
    }
  }
  fDrawIntersection (NorthPole_x,NorthPole_y,clRed);

  // Intersectons of the lower 3 Sensors:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Intersection Sensor 4 and Sensor 5:
  if (fCalculateIntersection ( GY271_Sensor4_X, GY271_Sensor4_Y, lX[3], lY[3],
                           GY271_Sensor5_X, GY271_Sensor5_Y, lX[4], lY[4], &P4_x, &P4_y)==false) return;
  fDrawIntersection (P4_x,P4_y,clWhite);

  // Intersection Sensor 4 and Sensor 6:
  if (fCalculateIntersection ( GY271_Sensor4_X, GY271_Sensor4_Y, lX[3], lY[3],
                           GY271_Sensor6_X, GY271_Sensor6_Y, lX[5], lY[5], &P5_x, &P5_y)==false) return;
  fDrawIntersection (P5_x,P5_y,clWhite);

  // Intersection Sensor 5 and Sensor 6:
  if (fCalculateIntersection ( GY271_Sensor5_X, GY271_Sensor5_Y, lX[4], lY[4],
                           GY271_Sensor6_X, GY271_Sensor6_Y, lX[5], lY[5], &P6_x, &P6_y)==false) return;
  fDrawIntersection (P6_x,P6_y,clWhite);

  double Distance_P4P5;
  double Distance_P4P6;
  double Distance_P5P6;

  double Faktor_P4P5;
  double Faktor_P4P6;
  double Faktor_P5P6;

  Distance_P4P5 = sqrt (((P4_x - P5_x)*(P4_x - P5_x)) + ((P4_y - P5_y)*(P4_y - P5_y)));
  Distance_P4P6 = sqrt (((P4_x - P6_x)*(P4_x - P6_x)) + ((P4_y - P6_y)*(P4_y - P6_y)));
  Distance_P5P6 = sqrt (((P5_x - P6_x)*(P5_x - P6_x)) + ((P5_y - P6_y)*(P5_y - P6_y)));

  Total_Distance = Distance_P4P5 + Distance_P4P6 + Distance_P5P6;

  Faktor_P4P5 = Total_Distance / Distance_P4P5;
  Faktor_P4P6 = Total_Distance / Distance_P4P6;
  Faktor_P5P6 = Total_Distance / Distance_P5P6;

  if (mnu2Of3->Checked==false)
  {
    SouthPole_x = (Faktor_P4P5 * ((P4_x + P5_x)/2) +
                   Faktor_P4P6 * ((P4_x + P6_x)/2) +
                   Faktor_P5P6 * ((P5_x + P6_x)/2) ) /
                   (Faktor_P4P5+Faktor_P4P6+Faktor_P5P6);

    SouthPole_y = (Faktor_P4P5 * ((P4_y + P5_y)/2) +
                   Faktor_P4P6 * ((P4_y + P6_y)/2) +
                   Faktor_P5P6 * ((P5_y + P6_y)/2) ) /
                   (Faktor_P4P5+Faktor_P4P6+Faktor_P5P6);
  }
  else
  {
    if ((Distance_P4P5<Distance_P4P6) && (Distance_P4P5<Distance_P5P6))
    { // Distance_P4P5 is the smalest one:
      ShortestDistance = Distance_P4P5;
      SouthPole_x = (P4_x + P5_x) / 2;
      SouthPole_y = (P4_y + P5_y) / 2;
    }
    else
    if ((Distance_P4P6<Distance_P4P5) && (Distance_P4P6<Distance_P5P6))
    { // Distance_P4P6 is the smalest one:
      ShortestDistance = Distance_P4P6;
      SouthPole_x = (P4_x + P6_x) / 2;
      SouthPole_y = (P4_y + P6_y) / 2;
    }
    else
    if ((Distance_P5P6<Distance_P4P5) && (Distance_P5P6<Distance_P4P6))
    { // Distance_P5P6 is the smalest one:
      ShortestDistance = Distance_P5P6;
      SouthPole_x = (P5_x + P6_x) / 2;
      SouthPole_y = (P5_y + P6_y) / 2;
    }
  }
  fDrawIntersection (SouthPole_x,SouthPole_y,clBlue);

  // Connect North and South pole with a line:
  fDrawLine (NorthPole_x, NorthPole_y, SouthPole_x, SouthPole_y, clPurple);

  // Draw the center of the foot:
  Foot_x = (NorthPole_x + SouthPole_x) / 2;
  Foot_y = (NorthPole_y + SouthPole_y) / 2;
  fDrawIntersection (Foot_x,Foot_y,clGreen);

  // Draw a line from Footplate Center to Center of shoe
  fDrawLine (0, 0, Foot_x, Foot_y, clRed);

  double CX;
  double CY;
  CX = StrToInt (sCenterX);
  CY = StrToInt (sCenterY);
  fDrawIntersection (CX,CY,clFuchsia );
}

//---------------------------------------------------------------------------
void __fastcall TForm1::tmrPollTimer(TObject *Sender)
{
  int iLength;
  char cData;
  if (hCom==0)
  {
    fInterpreteReceivedData("-1506\t-3971\t-4025\t577\t-2171\t-2637\t2793\t-3407\t-3881\t26278\t-2806\t-2\t-2597\t-15300\t12493\t-21076\t1857\t8611\t-6\t13\t#");
    //fInterpreteReceivedData("-4123\t-6720\t-6685\t636\t-3918\t-3500\t5183\t-5235\t-5918\t12708\t-17010\t20220\t-1937\t-8152\t3437\t-12537\t-4405\t6757\t#");
    return;
  }
  for (int i=0;i<28;i++)
  {
    iLength=ReceiveData(hCom,&cData,1);
    if ((iLength>0) && (cData!=13) && (cData!=10))
    {
      if (cData=='*')
      {
        iReceiveDataLength=0;
      }
      else
      {
        cReceiveData[iReceiveDataLength++]=cData;
      }
    }
    if (cData=='#')
    {
      // Daten sind vollständig übertragen
      fInterpreteReceivedData (cReceiveData);
      iReceiveDataLength=0;
    }
  }
}
//---------------------------------------------------------------------------

void __fastcall TForm1::SetOffsetforGY2711Click(TObject *Sender)
{
  SendData(hCom, "1", 1);
}
//---------------------------------------------------------------------------

void __fastcall TForm1::RemoveOffsetforGY2711Click(TObject *Sender)
{
  SendData(hCom, "2", 1);
}
//---------------------------------------------------------------------------

void __fastcall TForm1::mnu2Of3Click(TObject *Sender)
{
  mnu2Of3->Checked=!mnu2Of3->Checked;
}
//---------------------------------------------------------------------------


