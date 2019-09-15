object Form1: TForm1
  Left = 576
  Top = 0
  Width = 410
  Height = 450
  Caption = 'Form1'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  Menu = MainMenu1
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object PaintBox1: TPaintBox
    Left = 0
    Top = 0
    Width = 402
    Height = 377
    Align = alClient
    Color = clCream
    ParentColor = False
  end
  object Label1: TLabel
    Left = 240
    Top = 24
    Width = 32
    Height = 13
    Caption = 'Label1'
  end
  object Label2: TLabel
    Left = 240
    Top = 48
    Width = 32
    Height = 13
    Caption = 'Label2'
  end
  object StatusBar1: TStatusBar
    Left = 0
    Top = 377
    Width = 402
    Height = 19
    Panels = <>
    SimplePanel = False
  end
  object tbZoom: TTrackBar
    Left = 0
    Top = 0
    Width = 150
    Height = 45
    Min = 3
    Orientation = trHorizontal
    Frequency = 1
    Position = 3
    SelEnd = 0
    SelStart = 0
    TabOrder = 1
    TickMarks = tmBottomRight
    TickStyle = tsAuto
  end
  object MainMenu1: TMainMenu
    Left = 40
    Top = 96
    object File1: TMenuItem
      Caption = 'File'
      object COMPort1: TMenuItem
        Caption = 'COM-Port'
        object COM11: TMenuItem
          Caption = 'COM1'
          OnClick = COM11Click
        end
        object COM21: TMenuItem
          Caption = 'COM2'
          OnClick = COM21Click
        end
        object COM31: TMenuItem
          Caption = 'COM3'
          OnClick = COM31Click
        end
        object COM41: TMenuItem
          Caption = 'COM4'
          OnClick = COM41Click
        end
        object COM51: TMenuItem
          Caption = 'COM5'
          OnClick = COM51Click
        end
      end
      object N1: TMenuItem
        Caption = '-'
      end
      object Close1: TMenuItem
        Caption = 'Close'
      end
    end
    object Commands1: TMenuItem
      Caption = 'Commands'
      object SetOffsetforGY2711: TMenuItem
        Caption = 'Set Offset for GY271'
        OnClick = SetOffsetforGY2711Click
      end
      object RemoveOffsetforGY2711: TMenuItem
        Caption = 'Remove Offset for GY271'
        OnClick = RemoveOffsetforGY2711Click
      end
    end
    object Options1: TMenuItem
      Caption = 'Options'
      object mnu2Of3: TMenuItem
        Caption = 'use 2 of 3'
        Checked = True
        OnClick = mnu2Of3Click
      end
    end
  end
  object tmrPoll: TTimer
    Interval = 10
    OnTimer = tmrPollTimer
    Left = 72
    Top = 96
  end
end
