const char PIN_FOOTLEFT_ENCODER_A    = A0;  
const char PIN_FOOTLEFT_ENCODER_B    = A1;  
const char PIN_FOOTLEFT_PWM          =  9;  
const char PIN_FOOTLEFT_PLUS         =  7;
const char PIN_FOOTLEFT_MINUS        =  8;

const char PIN_FOOTRIGHT_ENCODER_A   = 4;  
const char PIN_FOOTRIGHT_ENCODER_B   = 5;  
const char PIN_FOOTRIGHT_PWM         = 6;  
const char PIN_FOOTRIGHT_PLUS        = A2;
const char PIN_FOOTRIGHT_MINUS       = A3;

const double ENCODER_PULSES_PER_360DEGREE = 3168; // Übersetzung des Getriebemotors ist 1:72, der Encoder hat 11 Pulse pro Motorumdrehung: 72 x 11 x 4 = 3168 Interrupts sind eine Umdrehung
static byte FootPower;

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void FootplateMotor_setup ()
{
  FootPower=75;
  
  pinMode (PIN_FOOTLEFT_ENCODER_A, INPUT_PULLUP);
  pinMode (PIN_FOOTLEFT_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_FOOTLEFT_PWM, OUTPUT);
  analogWrite(PIN_FOOTLEFT_PWM,0);
  pinMode (PIN_FOOTLEFT_PLUS, OUTPUT);
  pinMode (PIN_FOOTLEFT_MINUS, OUTPUT);
  digitalWrite (PIN_FOOTLEFT_PLUS,LOW);
  digitalWrite (PIN_FOOTLEFT_MINUS,LOW);

  pinMode (PIN_FOOTRIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode (PIN_FOOTRIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_FOOTRIGHT_PWM, OUTPUT);
  analogWrite(PIN_FOOTRIGHT_PWM,0);
  pinMode (PIN_FOOTRIGHT_PLUS, OUTPUT);
  pinMode (PIN_FOOTRIGHT_MINUS, OUTPUT);
  digitalWrite (PIN_FOOTRIGHT_PLUS,LOW);
  digitalWrite (PIN_FOOTRIGHT_MINUS,LOW);

  pciSetup (PIN_FOOTLEFT_ENCODER_A);
  pciSetup (PIN_FOOTLEFT_ENCODER_B);
  pciSetup (PIN_FOOTRIGHT_ENCODER_A);
  pciSetup (PIN_FOOTRIGHT_ENCODER_B);
  
  Footplate_Left.bEncoderPreviousChannelA = digitalRead (PIN_FOOTLEFT_ENCODER_A);
  Footplate_Left.bEncoderPreviousChannelB = digitalRead (PIN_FOOTLEFT_ENCODER_B);
  Footplate_Right.bEncoderPreviousChannelA = digitalRead (PIN_FOOTRIGHT_ENCODER_A);
  Footplate_Right.bEncoderPreviousChannelB = digitalRead (PIN_FOOTRIGHT_ENCODER_B);

  
}

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
  //
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{ // Left footplate interrupt
  bool bEncoderChannelA;
  bool bEncoderChannelB;
  int bCurrentDirection;
  bEncoderChannelA = digitalRead (PIN_FOOTLEFT_ENCODER_A);    
  bEncoderChannelB = digitalRead (PIN_FOOTLEFT_ENCODER_B);
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==HIGH))
  {
    if ((Footplate_Left.bEncoderPreviousChannelA==HIGH)&&(Footplate_Left.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=-1;
    else
      bCurrentDirection=1;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==HIGH))
  {
    if ((Footplate_Left.bEncoderPreviousChannelA==LOW)&&(Footplate_Left.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==LOW))
  {
    if ((Footplate_Left.bEncoderPreviousChannelA==HIGH)&&(Footplate_Left.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  else
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==LOW))
  {
    if ((Footplate_Left.bEncoderPreviousChannelA==HIGH)&&(Footplate_Left.bEncoderPreviousChannelB==LOW))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  Footplate_Left.bEncoderPreviousChannelA=bEncoderChannelA;
  Footplate_Left.bEncoderPreviousChannelB=bEncoderChannelB;
  if (bCurrentDirection>0)
  {
    Footplate_Left.iEncoderPosition++;
  }
  else
  {
    Footplate_Left.iEncoderPosition--;
  }  
}  

ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{ // Right footplate interrupt
  bool bEncoderChannelA;
  bool bEncoderChannelB;
  int bCurrentDirection;
  bEncoderChannelA = digitalRead (PIN_FOOTRIGHT_ENCODER_A);    
  bEncoderChannelB = digitalRead (PIN_FOOTRIGHT_ENCODER_B);
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==HIGH))
  {
    if ((Footplate_Right.bEncoderPreviousChannelA==HIGH)&&(Footplate_Right.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=-1;
    else
      bCurrentDirection=1;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==HIGH))
  {
    if ((Footplate_Right.bEncoderPreviousChannelA==LOW)&&(Footplate_Right.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  else
  if ((bEncoderChannelA==HIGH)&&(bEncoderChannelB==LOW))
  {
    if ((Footplate_Right.bEncoderPreviousChannelA==HIGH)&&(Footplate_Right.bEncoderPreviousChannelB==HIGH))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  else
  if ((bEncoderChannelA==LOW)&&(bEncoderChannelB==LOW))
  {
    if ((Footplate_Right.bEncoderPreviousChannelA==HIGH)&&(Footplate_Right.bEncoderPreviousChannelB==LOW))
      bCurrentDirection=1;
    else
      bCurrentDirection=-1;
  }
  Footplate_Right.bEncoderPreviousChannelA=bEncoderChannelA;
  Footplate_Right.bEncoderPreviousChannelB=bEncoderChannelB;
  if (bCurrentDirection>0)
  {
    Footplate_Right.iEncoderPosition++;
  }
  else
  {
    Footplate_Right.iEncoderPosition--;
  }  
}  
 
void FootplateMotor_loop ()
{ 
   static int iEncoderPositionOldLeft;
   static int iEncoderPositionOldRight;
   byte id;
   float Axis0Winkel;
   if (Footplate_Right.ExecuteMovement==false)
   {
       digitalWrite (PIN_FOOTRIGHT_PLUS,LOW);
       digitalWrite (PIN_FOOTRIGHT_MINUS,LOW);        
       analogWrite(PIN_FOOTRIGHT_PWM,0);    
   }
   else
   {
     if (Footplate_Right.Targetposition-2>Footplate_Right.iEncoderPosition)
     {
        // Rotate to the right:
        digitalWrite (PIN_FOOTRIGHT_MINUS,LOW);        
        digitalWrite (PIN_FOOTRIGHT_PLUS,HIGH);
        analogWrite(PIN_FOOTRIGHT_PWM,FootPower);
     } else   
     if (Footplate_Right.Targetposition+2<Footplate_Right.iEncoderPosition)
     {
        // Rotate to the left:
        digitalWrite (PIN_FOOTRIGHT_PLUS,LOW);
        digitalWrite (PIN_FOOTRIGHT_MINUS,HIGH);        
        analogWrite(PIN_FOOTRIGHT_PWM,FootPower);
     } else
     {
       // Target position reached
       digitalWrite (PIN_FOOTRIGHT_PLUS,LOW);
       digitalWrite (PIN_FOOTRIGHT_MINUS,LOW);        
       analogWrite(PIN_FOOTRIGHT_PWM,0);
     }
   }
   iEncoderPositionOldRight=Footplate_Right.iEncoderPosition;

   // LEFT PLATE
   // ~~~~~~~~~~
   if (Footplate_Left.ExecuteMovement==false)
   {
       digitalWrite (PIN_FOOTLEFT_PLUS,LOW);
       digitalWrite (PIN_FOOTLEFT_MINUS,LOW);        
       analogWrite(PIN_FOOTLEFT_PWM,0);    
   }
   else
   {
     if (Footplate_Left.Targetposition-2>Footplate_Left.iEncoderPosition)
     {
        // Rotate to the right:
        digitalWrite (PIN_FOOTLEFT_MINUS,LOW);        
        digitalWrite (PIN_FOOTLEFT_PLUS,HIGH);
        analogWrite(PIN_FOOTLEFT_PWM,FootPower);
     } else   
     if (Footplate_Left.Targetposition+2<Footplate_Left.iEncoderPosition)
     {
        // Rotate to the left:
        digitalWrite (PIN_FOOTLEFT_PLUS,LOW);
        digitalWrite (PIN_FOOTLEFT_MINUS,HIGH);        
        analogWrite(PIN_FOOTLEFT_PWM,FootPower);
     } else
     {
       // Target position reached
       digitalWrite (PIN_FOOTLEFT_PLUS,LOW);
       digitalWrite (PIN_FOOTLEFT_MINUS,LOW);        
       analogWrite(PIN_FOOTLEFT_PWM,0);
     }
   }
   iEncoderPositionOldLeft=Footplate_Left.iEncoderPosition;

   // Aktuellen Winkel der Rotationsachse ermitteln:
   // 360° entsprechen 725.33333 Motorumdrehungen
   if (StartDemoWalking==true)
   {
     Axis0Winkel = ODrive.Axis0.EncoderPosEstimate * (21600.0 / 43520.0);
     Footplate_Right.Targetposition = Axis0Winkel * (3168.0 / 360.0);
     Footplate_Right.ExecuteMovement=true;
     Footplate_Left.Targetposition = -Footplate_Right.Targetposition;
     Footplate_Left.ExecuteMovement=true;
   }
   
   if (Serial.available()>0)
   {
    if (Serial.readBytes(&id,1)==1)
    {
      if (id=='h')
      {
        // Alles nach Hause fahren lassen
        StartDemoWalking=false;
        ODrive.Axis0.Targetposition = 0;
        ODrive.Axis1.Targetposition = -36;
        ODrive.Axis0.ExecuteMovement = true;
        ODrive.Axis1.ExecuteMovement = true;
        Footplate_Left.Targetposition = 0;
        Footplate_Right.Targetposition = 0;
        Footplate_Right.ExecuteMovement=true;
        Footplate_Left.ExecuteMovement=true;
      }
      if (id=='x')
      {
        FootPower+=10;
        Serial.print ("FootPower +10 = ");
        Serial.println (FootPower);
      }else
      if (id=='y')
      {
        FootPower-=10;
        Serial.print ("FootPower -10 = ");
        Serial.println (FootPower);
      }else
      if (id=='q')
      {
        Footplate_Right.iEncoderPosition+=20;
        Footplate_Right.ExecuteMovement=true;
        Serial.println ("Encoder Right +20.");
      }else
      if (id=='w')
      {
        Footplate_Right.iEncoderPosition-=20;
        Footplate_Right.ExecuteMovement=true;
        Serial.println ("Encoder Right -20.");
      }else
      if (id=='Q')
      {
        Footplate_Left.iEncoderPosition-=20;
        Footplate_Left.ExecuteMovement=true;
        Serial.println ("Encoder Left -20.");
      }else
      if (id=='W')
      {
        Footplate_Left.iEncoderPosition+=20;
        Footplate_Left.ExecuteMovement=true;
        Serial.println ("Encoder Left +20.");
      }else
      if (id=='d')
      {
        StartDemoWalking=true;
        Serial.println ("Demowalk started.");
        Serial.print ("Speed=");
        Serial.println (DemoWalkingSpeed);
      }
      else
      if (id=='+')
      {
        DemoWalkingSpeed=DemoWalkingSpeed+0.5;
        Serial.print ("Speed=");
        Serial.println (DemoWalkingSpeed);
      } else
      if (id=='-')
      {
        DemoWalkingSpeed=DemoWalkingSpeed-0.5;
        Serial.print ("Speed=");
        Serial.println (DemoWalkingSpeed);
      } 
      else
      if (id=='s')
      {
        StartDemoWalking=false;
        Footplate_Right.ExecuteMovement=false;
        Footplate_Left.ExecuteMovement=false;
        Serial.println ("Demowalk stopped.");
        Serial.print ("Speed=");
        Serial.println (DemoWalkingSpeed);
      }
    }
   }   
}
