/*
  DigitalReadSerial

  Reads a digital input on pin 2, prints the result to the Serial Monitor

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/DigitalReadSerial
*/

// digital pin 2 has a pushbutton attached to it. Give it a name:
#define pinG 18
#define pinR 19
#define led 2
#define pot 4

volatile int GG;
volatile int RR;

volatile int CWcounter = 0;
 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterruptG() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinG) != GG){
    GG = !GG;
    RR = digitalRead(pinR);
    GG==RR?CWcounter--:CWcounter++;
    }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);

  pinMode(pinG, INPUT);
  pinMode(pinR, INPUT);
  pinMode(led, OUTPUT);
  
  GG = digitalRead(pinG);
  RR = digitalRead(pinR);
  Serial.print(GG);
  Serial.print("/");
  Serial.print(RR);
  Serial.print("/");
  Serial.println(CWcounter);
  
  attachInterrupt(digitalPinToInterrupt(pinG), handleInterruptG, CHANGE);
  
  while(true) {
    int atual = CWcounter>0?((CWcounter%11484)*360)/11484:((11484+(CWcounter%11484))*360)/11484;
    int ideal = (analogRead(pot)*360)/4095;
    int erro  = ideal>atual?((ideal-atual)<180?ideal-atual:-atual-360+ideal):((atual-ideal)>180?ideal+360-atual:-atual+ideal);
    Serial.print(CWcounter);
    Serial.print("\t");
    Serial.print(atual);
    Serial.print("\t");
    Serial.print(ideal);
    Serial.print("\t");
    Serial.println(erro);
  }  
}

void loop(){;}

//if(oldCWcounter != CWcounter){
//oldCWcounter = CWcounter;
//(GG != oldGG)?(GG==RR?CWcounter--:CWcounter++):(GG==RR?CWcounter++:CWcounter--);

//void IRAM_ATTR handleInterruptG() {
//  digitalWrite(led, HIGH);
//  GG = digitalRead(pinG);
//  portENTER_CRITICAL_ISR(&mux);
//  if (GG != oldGG){
//    oldGG = GG;
//    RR = digitalRead(pinR);
//    GG==RR?CWcounter--:CWcounter++;
//    }
//  portEXIT_CRITICAL_ISR(&mux);
//  digitalWrite(led, LOW);
//}
