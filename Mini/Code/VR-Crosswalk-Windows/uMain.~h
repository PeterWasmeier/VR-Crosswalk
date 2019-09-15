//---------------------------------------------------------------------------

#ifndef uMainH
#define uMainH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include <Graphics.hpp>
#include "PERFGRAP.h"
#include "CSPIN.h"
#include <Menus.hpp>
#include <ComCtrls.hpp>
#include <Chart.hpp>
#include <Series.hpp>
#include <TeEngine.hpp>
#include <TeeProcs.hpp>
#include <Mask.hpp>
//---------------------------------------------------------------------------
class TfrmMain : public TForm
{
__published:	// IDE-managed Components
        TSplitter *Splitter1;
        TMainMenu *MainMenu1;
        TMenuItem *File1;
        TMenuItem *Close1;
        TMenuItem *COMPort1;
        TMenuItem *COM11;
        TMenuItem *COM21;
        TMenuItem *COM31;
        TMenuItem *COM41;
        TMenuItem *COM51;
        TTimer *Timer1;
        TPanel *Panel7;
        TPageControl *PageControl1;
        TTabSheet *TabSheet1;
        TChart *Chart1;
        TLineSeries *Series1;
        TLineSeries *Series2;
        TLineSeries *Series3;
        TTabSheet *TabSheet2;
        TChart *Chart2;
        TLineSeries *LineSeries2;
        TLineSeries *LineSeries3;
        TPanel *Panel5;
        TPanel *Panel1;
        TImage *Image1;
        TSplitter *Splitter3;
        TPanel *Panel2;
        TSplitter *Splitter2;
        TPanel *Panel3;
        TMemo *Memo1;
        TPanel *Panel4;
        TSplitter *Splitter5;
        TPanel *Panel10;
        TListView *ListView1;
        TPanel *Panel11;
        TLabel *Label1;
        TLabel *Label2;
        TLabel *Label3;
        TLabel *Label4;
        TLabel *Label5;
        TLabel *Label6;
        TButton *btnOFF;
        TButton *btnStepper;
        TCSpinEdit *csStepperPosition;
        TCSpinEdit *csMotorPosition;
        TButton *btnMotor;
        TButton *Button2;
        TButton *btnP;
        TCSpinEdit *csP;
        TButton *btnI;
        TCSpinEdit *csI;
        TButton *btnD;
        TCSpinEdit *csD;
        TButton *btnStepperACC;
        TCSpinEdit *csStepperACC;
        TButton *btnSTATUS;
        TSplitter *Splitter4;
        TButton *btnMotor0;
        TButton *btnMotor100;
        TButton *btnGOR;
        TButton *btnStepper250;
        TButton *btnOffset_R;
        TButton *btnUp;
        TButton *btnDown;
        TButton *btnRight;
        TButton *btnLeft;
        TTimer *Timer2;
        TCheckBox *cbAutoupdate;
        void __fastcall Close1Click(TObject *Sender);
        void __fastcall COM11Click(TObject *Sender);
        void __fastcall Timer1Timer(TObject *Sender);
        void __fastcall COM21Click(TObject *Sender);
        void __fastcall COM31Click(TObject *Sender);
        void __fastcall COM41Click(TObject *Sender);
        void __fastcall COM51Click(TObject *Sender);
        void __fastcall Button2Click(TObject *Sender);
        void __fastcall btnOFFClick(TObject *Sender);
        void __fastcall btnStepperClick(TObject *Sender);
        void __fastcall btnMotorClick(TObject *Sender);
        void __fastcall btnPClick(TObject *Sender);
        void __fastcall btnIClick(TObject *Sender);
        void __fastcall btnDClick(TObject *Sender);
        void __fastcall btnStepperACCClick(TObject *Sender);
        void __fastcall btnSTATUSClick(TObject *Sender);
        void __fastcall btnMotor0Click(TObject *Sender);
        void __fastcall btnMotor100Click(TObject *Sender);
        void __fastcall btnGORClick(TObject *Sender);
        void __fastcall btnStepper250Click(TObject *Sender);
        void __fastcall btnOffset_RClick(TObject *Sender);
        void __fastcall Button1Click(TObject *Sender);
        void __fastcall btnUpClick(TObject *Sender);
        void __fastcall btnDownClick(TObject *Sender);
        void __fastcall btnRightClick(TObject *Sender);
        void __fastcall btnLeftClick(TObject *Sender);
        void __fastcall Timer2Timer(TObject *Sender);
private:	// User declarations

  int iMotor_LastPWMValue;
  int iMotor_TargetPosition;
  int iMotor_EncoderPosition;
  int iSteppermotor_CurrentPosition;
  int iSteppermotor_TargetPosition;
  double FootplateRight_SensorOffsetX;
  double FootplateRight_SensorOffsetY;
  int iServoLeftPosition;
  int iServoRightPosition;
        bool bRefreshDisplay;
        bool bUpdateChartMotor;
        bool bUpdateChartStepper;
        int iChartElements;
        int Dummy;
        HANDLE hCom;
        bool bReceiveStarted;
        char cReceiveData[512];
        int iReceiveDataLength;
        void CloseComm(HANDLE hCom);
        bool OpenComm(char *Port);
        void SetDeviceControlBlock(HANDLE hCom, DWORD BaudRate, BYTE ByteSize, BYTE Parity, BYTE StopBits);
        int SendData(HANDLE hCom, char *Data, int Length);
        void __fastcall UpdateChartMotor();
        void __fastcall UpdateChartStepper();
        DWORD ReceiveData(HANDLE hCom, char *Data, int Length);
        void __fastcall fInterpreteReceivedData (char *cChar);
        void __fastcall SendString (AnsiString sData);
        void __fastcall DrawBaseplate (int iMotorposition, int iStepperposition, int iFootplateRightServoPosition, int iFootplateLeftServoPosition, double dFootplateRightGY271X, double dFootplateRightGY271Y, double dFootplateLeftGY271X, double dFootplateLeftGY271Y);
        void __fastcall RotatePoint (double dCenterX, double dCenterY, double dRadius, double dDegree, double *dResultX, double *dResultY);
        void __fastcall RotatePoint2 (double dPointX, double dPointY, double dDegree, double dOffsetX, double dOffsetY, long *lResultX, long *lResultY);
public:		// User declarations
        __fastcall TfrmMain(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TfrmMain *frmMain;
//---------------------------------------------------------------------------
#endif
