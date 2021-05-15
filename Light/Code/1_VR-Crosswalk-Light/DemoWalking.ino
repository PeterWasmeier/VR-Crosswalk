void DemoWalking_loop ()
{ 
  // Wieviel Pulse pro Umdrehung?
  // Erste Übersetzung ist 1 zu 40     = 
  // Zweite Übersetzung ist 60 zu 1088
  // Aktuelle Position von Achse 0 ermitteln:
  static float old;
  static float Strecke;
  static float Richtung;
  static unsigned long ZeitAlt;
  static float Geschwindigkeit; // Zurück zu legende Strecke in Millimeter pro Sekunde
  float Vergangen; // Zeit in Sekunden seit dem letzten Aufruf
  float Axis0Position;
  float Axis0Winkel;
  float Axis0Bogenmass;
  float Axis1SollPosition;
  float Axis1IstPosition;
  float Fussabstand;
  float CosBogenmass;
  unsigned long Zeit;

  if (StartDemoWalking==false) return;

  Fussabstand = 220.0 / 2.0; // Die beiden Füße sind 220mm voneinander entfernt, bzw. jeder Fuß 110mm vom Drehzentrum
  
  // Achse 0 automatisch drehen:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // Wieviel Zeit ist vergangen seit dem letzten Aufruf?
  Zeit = millis();
  if (Zeit-ZeitAlt>500)
  {
    ZeitAlt=Zeit;
    return;
  }
  Vergangen = (float)(Zeit - ZeitAlt) / 1000.0; // Wieviel Sekunden sind seit dem letzten Aufruf vergangen?
  ZeitAlt = Zeit;
  if (Geschwindigkeit!=DemoWalkingSpeed)
  {
    if (Richtung<0)
      Richtung=-DemoWalkingSpeed;
    else
      Richtung=DemoWalkingSpeed;
  }
  Geschwindigkeit = DemoWalkingSpeed; 
    
  // Rotationsachse in Abhängigkeit der zurück zu legenden Strecke rotieren lassen:
  if (Richtung==0.0) Richtung=Geschwindigkeit; // Richtung ist mm pro Sekunde
  Strecke=Strecke+(Richtung*Vergangen);
  if (Strecke>100) // 100mm bereits erreicht?
  {
    Richtung=-Geschwindigkeit; // Zurück geht es
  }
  if (Strecke<-100) // -100mm bereits erreicht?
  {
    Richtung=Geschwindigkeit; // Wieder vorwärts
  }
  Axis0Winkel = atan (Strecke / Fussabstand) * (180/PI);
  // Nun den Winkel in Motorumdrehungen berechnen
  // 360° entsprechen 725.33333 Motorumdrehungen
  Axis0Position = Axis0Winkel * (725.33333/360.0);
  ODrive.Axis0.Targetposition = Axis0Position;
  ODrive.Axis0.ExecuteMovement=true;  
  
  // Fussplatten in Abhängigkeit der Rotationsachse bewegen:  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (old!=ODrive.Axis0.EncoderPosEstimate)
  { // Winkel der Axis0 hat sich verändert. Axis1 anpassen:    
    old=ODrive.Axis0.EncoderPosEstimate;
/*

    Serial.print ("Axis0.Target=");
    Serial.print (ODrive.Axis0.Targetposition);
    Serial.print (", Axis0.Encoder=");
    Serial.print (ODrive.Axis0.EncoderPosEstimate);
    Serial.print (" (");
    Serial.print (ODrive.Axis0.EncoderPosEstimate-ODrive.Axis0.Targetposition);
    Serial.print (" | ");
    
    Serial.print ("Axis1.Target=");
    Serial.print (ODrive.Axis1.Targetposition);
    Serial.print (", Axis1.Encoder=");
    Serial.print (ODrive.Axis1.EncoderPosEstimate);
    Serial.print (" (");
    Serial.println (ODrive.Axis1.EncoderPosEstimate-ODrive.Axis1.Targetposition);
  */  
    Axis0Position = ODrive.Axis0.EncoderPosEstimate;
    Axis0Winkel   = Axis0Position * (21600.0 / 43520.0); // Axis0Winkel = Axis0Position x (360/1) x (1/40) x (60/1088)
    Axis0Bogenmass = Axis0Winkel * PI / 180.0;

    // Nun die Fußplattenposition berechnen in Abhängigkeit der Achse 0
    // Witzigerweise entsprechen die Anzahl der Motorumdrehungen auch gleichzeitig der zurückgelegten
    // Wegstrecke, da das Schneckengetriebe 1:40 ist (d.h. 40 Motorumdrehungen = 1 Umdrechung nach dem Getriebe).
    // Und der Wirkumfang der Zahnriemenscheibe (20 Zähne) 40mm ist. Somit entsprichen 40 Motorumdrehung gleich
    // 40mm Wegstrecke.
    // Der Trittplattenabstand in der Homeposition entspricht 148mm.
    // Dies entspricht auch der Position 0 nach dem Homing. Aber wie bereits erwähnt, die Trittplatten sind dann bereits 148mm
    // voneinander entfernt.
    
    CosBogenmass = cos (Axis0Bogenmass);
    Axis1SollPosition = (Fussabstand / CosBogenmass);
    Axis1SollPosition = Axis1SollPosition - 74.0; // Die Trittplatten sind bereits 148/2 vom Drehzentrum entfernt
    Axis1SollPosition = -Axis1SollPosition; // Vorzeichen umkehren, da diese Achse negativ fährt
    
    // Sicherheitshalber die Trittplatten begren zen:
    if (Axis1SollPosition>0.0) Axis1SollPosition=0.0;
    if (Axis1SollPosition<-280.0) Axis1SollPosition=-280.0;
    
    ODrive.Axis1.Targetposition = Axis1SollPosition;
    ODrive.Axis1.ExecuteMovement=true;
  }
}
