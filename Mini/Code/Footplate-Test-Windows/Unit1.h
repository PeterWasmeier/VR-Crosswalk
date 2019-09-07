//---------------------------------------------------------------------------

#ifndef Unit1H
#define Unit1H
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <Menus.hpp>
#include <ExtCtrls.hpp>
#include <ComCtrls.hpp>
//---------------------------------------------------------------------------
class TForm1 : public TForm
{
__published:	// IDE-managed Components
        TMainMenu *MainMenu1;
        TMenuItem *File1;
        TMenuItem *COMPort1;
        TMenuItem *COM11;
        TMenuItem *COM21;
        TMenuItem *COM31;
        TMenuItem *COM41;
        TMenuItem *COM51;
        TMenuItem *N1;
        TMenuItem *Close1;
        TTimer *tmrPoll;
        TStatusBar *StatusBar1;
        TPaintBox *PaintBox1;
        TLabel *Label1;
        TLabel *Label2;
        TMenuItem *Commands1;
        TMenuItem *SetOffsetforGY2711;
        TMenuItem *RemoveOffsetforGY2711;
        TMenuItem *Options1;
        TMenuItem *mnu2Of3;
        TTrackBar *tbZoom;
        void __fastcall COM11Click(TObject *Sender);
        void __fastcall COM21Click(TObject *Sender);
        void __fastcall COM31Click(TObject *Sender);
        void __fastcall COM41Click(TObject *Sender);
        void __fastcall COM51Click(TObject *Sender);
        void __fastcall tmrPollTimer(TObject *Sender);
        void __fastcall SetOffsetforGY2711Click(TObject *Sender);
        void __fastcall RemoveOffsetforGY2711Click(TObject *Sender);
        void __fastcall mnu2Of3Click(TObject *Sender);
private:	// User declarations
        HANDLE hCom;
        int iOffsetX;
        int iOffsetY;
        int iOffsetZ;
        bool bSetOffset;
        char cReceiveData[512];
        int iReceiveDataLength;
        void CloseComm(HANDLE hCom);
        HANDLE OpenComm(char *Port);
        void __fastcall fInterpreteReceivedData (char *String);
        void SetDeviceControlBlock(HANDLE hCom, DWORD BaudRate, BYTE ByteSize, BYTE Parity, BYTE StopBits);
        int SendData(HANDLE hCom, char *Data, int Length);
        DWORD ReceiveData(HANDLE hCom, char *Data, int Length);
        bool __fastcall fCalculateIntersection (double a1, double b1, double S1y, double S1x,
                                                double a2, double b2, double S2y, double S2x,
                                                double *x, double *y);
        void __fastcall fDrawFoorprint (int X1, int Y1, int Z1,
                                        int X2, int Y2, int Z2,
                                        int X3, int Y3, int Z3,
                                        int X4, int Y4, int Z4,
                                        int X5, int Y5, int Z5,
                                        int X6, int Y6, int Z6);
        void __fastcall fCircle (int X, int Y, int R);
        void __fastcall fDrawIntersection (double X, double Y, TColor color);
        void __fastcall fDrawMagnet (double X, double Y, int Sx, int Sy);
        void __fastcall fDrawLine (double X1, double Y1, double X2, double Y2, TColor color);

public:		// User declarations
        __fastcall TForm1(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
