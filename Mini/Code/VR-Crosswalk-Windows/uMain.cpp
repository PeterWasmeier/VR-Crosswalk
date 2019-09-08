//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Math.h"
#include "uMain.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "PERFGRAP"
#pragma link "CSPIN"
#pragma resource "*.dfm"
TfrmMain *frmMain;

#define ScrollPoints 1
#define MaxPoints 500

//---------------------------------------------------------------------------
__fastcall TfrmMain::TfrmMain(TComponent* Owner)
        : TForm(Owner)
{
  hCom=INVALID_HANDLE_VALUE;
  bReceiveStarted=false;
  Dummy=0;
}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::Close1Click(TObject *Sender)
{
  CloseComm (hCom);
  Close ();
}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::COM11Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM1")) != INVALID_HANDLE_VALUE)
  {
    COM11->Checked=true;
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
  }
}
//---------------------------------------------------------------------------
HANDLE TfrmMain::OpenComm(char *Port)
{
  bReceiveStarted=false;
  CloseComm (hCom);
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
    return INVALID_HANDLE_VALUE;
  }
  iReceiveDataLength=0;
  return hCom2;
}
//---------------------------------------------------------------------------
void TfrmMain::SetDeviceControlBlock(HANDLE hCom, DWORD BaudRate, BYTE ByteSize, BYTE Parity, BYTE StopBits)
{
  if (hCom == INVALID_HANDLE_VALUE) return;

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

  COMMTIMEOUTS timeouts;

  timeouts.ReadIntervalTimeout = 1;
  timeouts.ReadTotalTimeoutMultiplier = 1;
  timeouts.ReadTotalTimeoutConstant = 1;
  timeouts.WriteTotalTimeoutMultiplier = 1;
  timeouts.WriteTotalTimeoutConstant = 1;
  SetCommTimeouts(hCom, &timeouts);

}
//---------------------------------------------------------------------------
int TfrmMain::SendData(HANDLE hCom, char *Data, int Length)
{
  DWORD NumberOfBytesWritten;
  if (hCom == INVALID_HANDLE_VALUE) return 0;
  if (!::WriteFile(hCom, Data, Length, &NumberOfBytesWritten, 0))
  {
    // ... Fehler
  }
  return NumberOfBytesWritten;
}
//---------------------------------------------------------------------------
DWORD TfrmMain::ReceiveData(HANDLE hCom, char *Data, int Length)
{
  DWORD NumberOfBytesRead;
  if (hCom == INVALID_HANDLE_VALUE) return 0;
  if (!::ReadFile(hCom, Data, Length, &NumberOfBytesRead, 0))
  {
    NumberOfBytesRead=0;// ... Fehler
  }
  return NumberOfBytesRead;
}
//---------------------------------------------------------------------------
void TfrmMain::CloseComm(HANDLE hCom)
{
  COM11->Checked=false;
  COM21->Checked=false;
  COM31->Checked=false;
  COM41->Checked=false;
  COM51->Checked=false;
  if (hCom == INVALID_HANDLE_VALUE) return;
  PurgeComm(hCom, PURGE_RXABORT);
  CloseHandle(hCom);
  hCom=INVALID_HANDLE_VALUE;
}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::Timer1Timer(TObject *Sender)
{
  int iLength;
  char cData;
  if (hCom == INVALID_HANDLE_VALUE) return;
  Timer1->Enabled=false;
  for (int i=0;i<2048;i++)
  {
    iLength=ReceiveData(hCom,&cData,1);
    if (iLength==0) break;
    if ((cData!=13) && (cData!=10))
    {
      if (cData=='*')
      {
        iReceiveDataLength=0;
        bReceiveStarted=true;
      }
      else
      {
        if (bReceiveStarted==true)
        {
          cReceiveData[iReceiveDataLength++]=cData;
        }
      }
      if ((cData=='#')&&(bReceiveStarted==true))
      {
        bReceiveStarted=false;
        iReceiveDataLength=0;
        // Daten sind vollständig übertragen
        fInterpreteReceivedData (cReceiveData);
        if (bUpdateChart) UpdateChart ();
      }
    }
  }
  if (bRefreshDisplay==true)
  {
    bRefreshDisplay=false;
    DrawBaseplate(iMotor_EncoderPosition,iSteppermotor_CurrentPosition,
                iServoRightPosition,iServoLeftPosition,
                FootplateRight_SensorOffsetX,FootplateRight_SensorOffsetY,
                0,0);
  }
  Timer1->Enabled=true;

}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::fInterpreteReceivedData (char *cChar)
{
  AnsiString sData;
  AnsiString sS[5];
  TListItem *LI;
  sData = cChar;
  Memo1->Lines->Add(sData);

  // *;
  sS[0]=sData.SubString(1,sData.Pos(';')-1);
  sData = sData.SubString (sData.Pos(';')+1,999);
  // Time
  sS[1]=sData.SubString(1,sData.Pos(';')-1);
  sData = sData.SubString (sData.Pos(';')+1,999);
  // Name
  sS[2]=sData.SubString(1,sData.Pos(';')-1);
  sData = sData.SubString (sData.Pos(';')+1,999);
  // Value
  sS[3]=sData.SubString(1,sData.Pos(';')-1);
  sData = sData.SubString (sData.Pos(';')+1,999);

  LI=ListView1->FindCaption (0,sS[2],false,true,false);
  if (LI!=NULL)
  {
    LI->SubItems->Strings[0]=sS[3];
  }
  else
  {
    LI = ListView1->Items->Add();
    LI->Caption = sS[2];
    LI->SubItems->Add(sS[3]);
  }

  try
  {
  if (sS[2]=="MOTOR_PWM") // Current PWM signal for the motor
  {
    iMotor_LastPWMValue = StrToInt(sS[3]);
    bUpdateChart=true;
  }
  if (sS[2]=="SERVO_L_POS") // Servo Left Position in Units
  {
    iServoLeftPosition = StrToInt(sS[3]);
    bRefreshDisplay=true;
  }
  if (sS[2]=="SERVO_R_POS") // Servo Right Position in Units
  {
    iServoRightPosition = StrToInt(sS[3]);
    bRefreshDisplay=true;
  }
  if (sS[2]=="MOTOR_TARGET") // Motor target position
  {
    iMotor_TargetPosition = StrToInt(sS[3]);
    bUpdateChart=true;
  }
  if (sS[2]=="MOTOR_ENCODER") // Motor current position
  {
    iMotor_EncoderPosition = StrToInt(sS[3]);
    bRefreshDisplay=true;
    bUpdateChart=true;
  }
  if (sS[2]=="STEPPER_POS") // Stepper motor position
  {
    iSteppermotor_CurrentPosition = StrToInt(sS[3]);
    bRefreshDisplay=true;
  }
  if (sS[2]=="FOOT_R_X") // GY271 Sensor X value from the right footplate
  {
    FootplateRight_SensorOffsetX = StrToInt(sS[3]);
    FootplateRight_SensorOffsetX = FootplateRight_SensorOffsetX / 1000;
    bRefreshDisplay=true;
  }
  if (sS[2]=="FOOT_R_Y") // GY271 Sensor Y value from the right footplate
  {
    FootplateRight_SensorOffsetY = StrToInt(sS[3]);
    FootplateRight_SensorOffsetY = FootplateRight_SensorOffsetY / 1000;
    bRefreshDisplay=true;
  }
  }
  catch (...)
  {
    //
  }
}

void __fastcall TfrmMain::UpdateChart()
{
  double tmp1,tmp2,tmp3,tmpMin,tmpMax;
  bool bDelete=false;

  bUpdateChart=false;

  if (Chart1->Series[0]->XValues->Count()>MaxPoints) { Chart1->Series[0]->Delete(0);bDelete=true; }
  if (Chart1->Series[1]->XValues->Count()>MaxPoints) { Chart1->Series[1]->Delete(0);bDelete=true; }
  if (Chart1->Series[2]->XValues->Count()>MaxPoints) { Chart1->Series[2]->Delete(0);bDelete=true; }

  if (Chart1->Series[0]->XValues->Count()>0) tmp1 = Chart1->Series[0]->XValues->Last(); else tmp1=0;
  if (Chart1->Series[1]->XValues->Count()>0) tmp2 = Chart1->Series[1]->XValues->Last(); else tmp2=0;
  if (Chart1->Series[2]->XValues->Count()>0) tmp3 = Chart1->Series[2]->XValues->Last(); else tmp3=0;

  Chart1->Series[0]->GetHorizAxis->SetMinMax(tmp1-150,tmp1+1);
  Chart1->Series[1]->GetHorizAxis->SetMinMax(tmp2-150,tmp2+1);
  Chart1->Series[2]->GetHorizAxis->SetMinMax(tmp3-150,tmp3+1);

  Chart1->Series[0]->AddXY(tmp1+1,iMotor_LastPWMValue);      // RED
  Chart1->Series[1]->AddXY(tmp2+1,iMotor_TargetPosition);    // GREEN
  Chart1->Series[2]->AddXY(tmp3+1,iMotor_EncoderPosition);   // BLUE

  // Min-Max PWM
  tmpMin = Chart1->Series[0]->YValues->MinValue;
  tmpMax = Chart1->Series[0]->YValues->MaxValue;
  //Chart1->Series[0]->GetVertAxis->SetMinMax(tmpMin-tmpMin/5,tmpMax+tmpMax/5);
  //Chart1->Series[0]->GetVertAxis->SetMinMax(-90,90);

  // Min-Max Position
  tmpMin = Chart1->Series[1]->YValues->MinValue;
  if (Chart1->Series[2]->YValues->MinValue<tmpMin)
    tmpMin = Chart1->Series[2]->YValues->MinValue;
  tmpMax = Chart1->Series[1]->YValues->MaxValue;
  if (Chart1->Series[2]->YValues->MaxValue>tmpMax)
    tmpMax = Chart1->Series[2]->YValues->MaxValue;
  //Chart1->Series[1]->GetVertAxis->SetMinMax(-600,+600);
//  Chart1->Series[1]->GetVertAxis->SetMinMax(tmpMin-tmpMin/5,tmpMax+tmpMax/5);

}

void __fastcall TfrmMain::COM21Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM2")) != INVALID_HANDLE_VALUE)
  {
    COM21->Checked=true;
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
  }
}
//---------------------------------------------------------------------------

void __fastcall TfrmMain::COM31Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM3")) != INVALID_HANDLE_VALUE)
  {
    COM31->Checked=true;
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
  }
}
//---------------------------------------------------------------------------

void __fastcall TfrmMain::COM41Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM4")) != INVALID_HANDLE_VALUE)
  {
    COM41->Checked=true;
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
  }
}
//---------------------------------------------------------------------------

void __fastcall TfrmMain::COM51Click(TObject *Sender)
{
  if ((hCom = OpenComm("COM5")) != INVALID_HANDLE_VALUE)
  {
    COM51->Checked=true;
    SetDeviceControlBlock(hCom, 115000, 8, NOPARITY, ONESTOPBIT);
  }
}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::RotatePoint (double dCenterX, double dCenterY, double dRadius, double dDegree, double *dResultX, double *dResultY)
{
  double dX, dY;
  double dDegreeRadiant;
  dDegreeRadiant = dDegree * (M_PI / 180.0);

  dY = sin (dDegreeRadiant) * dRadius;
  dX = cos (dDegreeRadiant) * dRadius;
  *dResultX = dCenterX + dX;
  *dResultY = dCenterY - dY; // Invert dY, because on computer screen, Y is inverted
}

void __fastcall TfrmMain::RotatePoint2 (double dPointX, double dPointY, double dDegree, double dOffsetX, double dOffsetY, long *lResultX, long *lResultY)
{
  // Calculate the current angle:
  double dRadius;
  double dCurrentAngleRadiant;
  double dCurrentAngle;
  double dDestinationAgle;
  double dDestinationAgleRadiant;
  double dX;
  double dY;

  dRadius = sqrt ( dPointX*dPointX + dPointY*dPointY );
  if (dRadius==0.0)
  {
    *lResultX = dOffsetX;
    *lResultY = dOffsetY; // Invert dY, because on computer screen, Y is inverted
    return;
  }
  dCurrentAngleRadiant = acos ( dPointX / dRadius );
  if (dPointY<0) dCurrentAngleRadiant=-dCurrentAngleRadiant;
  dCurrentAngle = (dCurrentAngleRadiant / M_PI) * 180;

  dDestinationAgle = dCurrentAngle + dDegree;
  dDestinationAgleRadiant = dDestinationAgle * (M_PI / 180.0);

  dX = cos ( dDestinationAgleRadiant ) * dRadius;
  dY = sin ( dDestinationAgleRadiant ) * dRadius;

  *lResultX = dX + dOffsetX;
  *lResultY = dY + dOffsetY; // Invert dY, because on computer screen, Y is inverted
}

void __fastcall TfrmMain::DrawBaseplate(int iMotorposition, int iStepperposition, int iFootplateRightServoPosition, int iFootplateLeftServoPosition, double dFootplateRightGY271X, double dFootplateRightGY271Y, double dFootplateLeftGY271X, double dFootplateLeftGY271Y)
{
  int iX,iY;
  long lX,lY;
  double dX, dY, dX1, dY1, dX2, dY2;
  double dX1m, dY1m, dX2m, dY2m;
  double dX1a, dY1a, dX2a, dY2a;
  double dX1b, dY1b, dX2b, dY2b;
  double dFootprintLeftCenterX, dFootprintLeftCenterY;
  double dFootprintRightCenterX, dFootprintRightCenterY;
  double dDegreeFootprintRight;
  double dDegreeFootprintLeft;
  double dDegreeFootprintRightRadiant;
  double dDegreeBaseplate;
  double dDegreeBaseplateRadiant;
  double dMotorposition;
  double dStepperposition;
  double dRadiusStepper;
  TPoint points[4];

  //Panel6->Visible=false;


  // Draw baseplate:
  Image1->Canvas->Brush->Color = clCream;   // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(0,0,165*2*2,165*2*2);

  dStepperposition = iStepperposition;
  dRadiusStepper = dStepperposition / 5 * 2; // Convert from Units to mm (5 steps = 1mm)

  dDegreeFootprintRight = iFootplateRightServoPosition - 1500; // -45° = -600, 0°=0, +45°=+600 (plus offset of 1500)
  dDegreeFootprintRight = 45.0/600.0 * dDegreeFootprintRight;
  dDegreeFootprintLeft = iFootplateLeftServoPosition-1500;
  dDegreeFootprintLeft = 45.0/600.0 * -dDegreeFootprintLeft;

  dMotorposition = iMotorposition;
  dDegreeBaseplate = 90.0 * (dMotorposition / 600);

  dDegreeBaseplateRadiant = dDegreeBaseplate * (M_PI / 180.0);

  // Calculate both center positions of the RIGHT stepperaxis hole:
  dY1 = sin (dDegreeBaseplateRadiant) * 130.0 * 2; // 130=distance from center to middle of outer
  dX1 = cos (dDegreeBaseplateRadiant) * 130.0 * 2;
  dY2 = sin (dDegreeBaseplateRadiant) * 24.0  * 2;  // 24=distance from center to middle of inner
  dX2 = cos (dDegreeBaseplateRadiant) * 24.0  * 2;

  dX1m = 165*2 + dX1;
  dY1m = 165*2 - dY1; // Invert dY, because on computer screen, Y is inverted

  dX2m = 165*2 + dX2;
  dY2m = 165*2 - dY2;

  // Draw right hole for the stepper axis:
  Image1->Canvas->Brush->Color = clBlack; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(dX1m-6*2, dY1m-6*2, dX1m+6*2, dY1m+6*2);  // Draw right hole with diameter 12mm
  Image1->Canvas->Ellipse(dX2m-6*2, dY2m-6*2, dX2m+6*2, dY2m+6*2);  // Draw right hole with diameter 12mm
  // Connect both holes:
  RotatePoint (dX1m, dY1m, 6*2, 90+dDegreeBaseplate, &dX1a, &dY1a); // Above
  RotatePoint (dX1m, dY1m, 6*2,-90+dDegreeBaseplate, &dX1b, &dY1b);
  RotatePoint (dX2m, dY2m, 6*2, 90+dDegreeBaseplate, &dX2a, &dY2a);
  RotatePoint (dX2m, dY2m, 6*2,-90+dDegreeBaseplate, &dX2b, &dY2b);
  points[0] = Point(dX2a,dY2a);
  points[1] = Point(dX1a,dY1a);
  points[2] = Point(dX1b,dY1b);
  points[3] = Point(dX2b,dY2b);
  Image1->Canvas->Polygon(points, 3);

  // Draw left hole for the stepper axis:
  dX1m = 165*2 - dX1;
  dY1m = 165*2 + dY1; // Invert dY, because on computer screen, Y is inverted
  dX2m = 165*2 - dX2;
  dY2m = 165*2 + dY2;
  Image1->Canvas->Brush->Color = clBlack; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(dX1m-6*2, dY1m-6*2, dX1m+6*2, dY1m+6*2);  // Draw right hole with diameter 12mm
  Image1->Canvas->Ellipse(dX2m-6*2, dY2m-6*2, dX2m+6*2, dY2m+6*2);  // Draw right hole with diameter 12mm
  // Connect both holes:
  RotatePoint (dX1m, dY1m, 6*2, 90+dDegreeBaseplate, &dX1a, &dY1a); // Above
  RotatePoint (dX1m, dY1m, 6*2,-90+dDegreeBaseplate, &dX1b, &dY1b);
  RotatePoint (dX2m, dY2m, 6*2, 90+dDegreeBaseplate, &dX2a, &dY2a);
  RotatePoint (dX2m, dY2m, 6*2,-90+dDegreeBaseplate, &dX2b, &dY2b);
  points[0] = Point(dX2a,dY2a);
  points[1] = Point(dX1a,dY1a);
  points[2] = Point(dX1b,dY1b);
  points[3] = Point(dX2b,dY2b);
  Image1->Canvas->Polygon(points, 3);

  // ***************************************************************
  // Draw the center of the right footplate
  // ***************************************************************

  // Calculate center of the right footplate:
  dFootprintRightCenterY = sin (dDegreeBaseplateRadiant) * dRadiusStepper;
  dFootprintRightCenterX = cos (dDegreeBaseplateRadiant) * dRadiusStepper;
  dFootprintLeftCenterY = -dFootprintRightCenterY;
  dFootprintLeftCenterX = -dFootprintRightCenterX;

  dFootprintRightCenterX = 165*2 + dFootprintRightCenterX;
  dFootprintRightCenterY = 165*2 - dFootprintRightCenterY; // Invert dY, because on computer screen, Y is inverted

  // Draw Center of right Footplate:
  Image1->Canvas->Brush->Color = clRed; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clRed;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(dFootprintRightCenterX-6*2, dFootprintRightCenterY-6*2, dFootprintRightCenterX+6*2, dFootprintRightCenterY+6*2);

  // Draw Center of left Footplate:
  dFootprintLeftCenterX = 165*2 + dFootprintLeftCenterX;
  dFootprintLeftCenterY = 165*2 - dFootprintLeftCenterY;
  Image1->Canvas->Brush->Color = clRed; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clRed;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(dFootprintLeftCenterX-6*2, dFootprintLeftCenterY-6*2, dFootprintLeftCenterX+6*2, dFootprintLeftCenterY+6*2);

  // ***************************************************************
  // Draw right footplate
  // ***************************************************************
  // Hier muss zweimal gedreht werden, nicht nur einmal! XXX
  RotatePoint2 (-30*2,  81*2, -dDegreeBaseplate, 0, 0, &points[0].x, &points[0].y);
  RotatePoint2 ( 30*2,  81*2, -dDegreeBaseplate, 0, 0, &points[1].x, &points[1].y);
  RotatePoint2 ( 30*2, -81*2, -dDegreeBaseplate, 0, 0, &points[2].x, &points[2].y);
  RotatePoint2 (-30*2, -81*2, -dDegreeBaseplate, 0, 0, &points[3].x, &points[3].y);

  RotatePoint2 (points[0].x, points[0].y, dDegreeFootprintRight, dFootprintRightCenterX, dFootprintRightCenterY, &points[0].x, &points[0].y);
  RotatePoint2 (points[1].x, points[1].y, dDegreeFootprintRight, dFootprintRightCenterX, dFootprintRightCenterY, &points[1].x, &points[1].y);
  RotatePoint2 (points[2].x, points[2].y, dDegreeFootprintRight, dFootprintRightCenterX, dFootprintRightCenterY, &points[2].x, &points[2].y);
  RotatePoint2 (points[3].x, points[3].y, dDegreeFootprintRight, dFootprintRightCenterX, dFootprintRightCenterY, &points[3].x, &points[3].y);

  Image1->Canvas->Brush->Color = clGreen; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Polygon(points, 3);

  // ***************************************************************
  // Draw left footplate
  // ***************************************************************

  RotatePoint2 (-30*2,  81*2, -dDegreeBaseplate, 0, 0, &points[0].x, &points[0].y);
  RotatePoint2 ( 30*2,  81*2, -dDegreeBaseplate, 0, 0, &points[1].x, &points[1].y);
  RotatePoint2 ( 30*2, -81*2, -dDegreeBaseplate, 0, 0, &points[2].x, &points[2].y);
  RotatePoint2 (-30*2, -81*2, -dDegreeBaseplate, 0, 0, &points[3].x, &points[3].y);

  RotatePoint2 (points[0].x, points[0].y, dDegreeFootprintLeft, dFootprintLeftCenterX, dFootprintLeftCenterY, &points[0].x, &points[0].y);
  RotatePoint2 (points[1].x, points[1].y, dDegreeFootprintLeft, dFootprintLeftCenterX, dFootprintLeftCenterY, &points[1].x, &points[1].y);
  RotatePoint2 (points[2].x, points[2].y, dDegreeFootprintLeft, dFootprintLeftCenterX, dFootprintLeftCenterY, &points[2].x, &points[2].y);
  RotatePoint2 (points[3].x, points[3].y, dDegreeFootprintLeft, dFootprintLeftCenterX, dFootprintLeftCenterY, &points[3].x, &points[3].y);

  Image1->Canvas->Brush->Color = clGreen; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Polygon(points, 3);

  // ***************************************************************
  // Draw GY271 values
  // ***************************************************************
  RotatePoint2 (dFootplateRightGY271X*2,  dFootplateRightGY271Y*2, dDegreeFootprintRight, dFootprintRightCenterX, dFootprintRightCenterY, &lX, &lY);
  Image1->Canvas->Brush->Color = clRed; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clRed;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(lX-6*2, lY-6*2, lX+6*2, lY+6*2);

  Image1->Canvas->Brush->Color = clBlack; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->MoveTo (dFootprintRightCenterX,dFootprintRightCenterY);
  Image1->Canvas->LineTo (lX,lY);

  RotatePoint2 (dFootplateLeftGY271X*2,  dFootplateLeftGY271Y*2, dDegreeFootprintLeft, dFootprintLeftCenterX, dFootprintLeftCenterY, &lX, &lY);
  Image1->Canvas->Brush->Color = clRed; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clRed;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->Ellipse(lX-6*2, lY-6*2, lX+6*2, lY+6*2);


  Image1->Canvas->Brush->Color = clBlack; // Background color, inner fill of the ellipse
  Image1->Canvas->Pen->Color = clBlack;     // Forecolor, outer bounds color of the ellipse
  Image1->Canvas->MoveTo (dFootprintLeftCenterX,dFootprintLeftCenterY);
  Image1->Canvas->LineTo (lX,lY);

  Panel6->Visible=true;

}


void __fastcall TfrmMain::Button2Click(TObject *Sender)
{
     fInterpreteReceivedData ("*;13:22:11.231;MOTOR_ENCODER;123;#");
  if (bUpdateChart)
    UpdateChart ();
/*
  DrawBaseplate(300,            // Motor Position
                400,            // Stepper Position
                2100,1500,      // Servo right and Servo left
                0,0,            // GY271 Center Footplate right
                0,0);           // GY271 Center Footplate left
                */
}
//---------------------------------------------------------------------------
void __fastcall TfrmMain::SendString (AnsiString sData)
{
  if (hCom == INVALID_HANDLE_VALUE) return;
  Memo1->Lines->Add(sData);
  sData = sData + "\r";
  SendData (hCom,sData.c_str(),sData.Length());
}


void __fastcall TfrmMain::btnOFFClick(TObject *Sender)
{
  SendString ("OFF");
}
//---------------------------------------------------------------------------

void __fastcall TfrmMain::btnStepperClick(TObject *Sender)
{
  AnsiString sString;
  sString="STEPPER=" + IntToStr(csStepperPosition->Value);
  SendString (sString);
}
//---------------------------------------------------------------------------

void __fastcall TfrmMain::Button1Click(TObject *Sender)
{
  AnsiString sString;
  sString="MOTOR=" + IntToStr(csMotorPosition->Value);
  SendString (sString);
}
//---------------------------------------------------------------------------


void __fastcall TfrmMain::FormShow(TObject *Sender)
{
/*  Chart1->ClipPoints = false;
  Chart1->Title->Visible = false;
  Chart1->Legend->Visible = false;
  Chart1->LeftAxis->Axis->Width=1;
  Chart1->BottomAxis->Axis->Width=1;
  Chart1->BottomAxis->RoundFirstLabel = false;
  Chart1->View3D = false;
  Chart1->Series[0]->XValues->Order=loNone;
  Chart1->Series[1]->XValues->Order=loNone;
  Chart1->Series[2]->XValues->Order=loNone;
  Chart1->BottomAxis->SetMinMax(1,200);
  Chart1->Canvas->ReferenceCanvas->Pen->OwnerCriticalSection = 0;
  */
}
//---------------------------------------------------------------------------

