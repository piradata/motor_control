#define magic_N 11484
#define pinG 18
#define pinR 19
#define led 2
#define pot 4
#define freq 10000
#define MA 22
#define MB 23
#define MAChannel  0
#define MBChannel  1
#define resolution  10 //PWM 0 -> 1023

#define Kp 200
#define Kd 1
#define H_T 0.1

volatile int GG, RR;
volatile int CWcounter = 0;

int erro, lasterro = 0;
int POS, oldPOS = 0, VEL = 0;
int REF, REF_F = 0;
int out, OUTF = 0;
int AUMENTA;

//DONT TOUCH THIS SHIT!!! IT IS FUCKING WORKING!!!
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR handleInterruptG() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinG) != GG) {
    //(GG != oldGG)?(GG==RR?CWcounter--:CWcounter++):(GG==RR?CWcounter++:CWcounter--);
    GG = !GG;
    RR = digitalRead(pinR);
    GG == RR ? CWcounter-- : CWcounter++;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);

  ledcSetup(MAChannel, freq, resolution);
  ledcAttachPin(MA, MAChannel);
  ledcWrite(MAChannel, 0);

  ledcSetup(MBChannel, freq, resolution);
  ledcAttachPin(MB, MBChannel);
  ledcWrite(MBChannel, 0);

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

  while (true) {
    POS = CWcounter > 0 ? ((CWcounter % magic_N) * 360) / magic_N : ((magic_N + (CWcounter % magic_N)) * 360) / magic_N;
    VEL = POS - oldPOS;
    oldPOS = POS;

    REF = (analogRead(pot) * 360) / 4095;
    REF_F = 0.561 * REF_F + (1 - 0.561) * REF;

    erro  = REF_F > POS ? ((REF_F - POS) < 180 ? REF_F - POS : -POS - 360 + REF_F) : ((POS - REF_F) > 180 ? REF_F + 360 - POS : -POS + REF_F);

    out =  Kp * (erro - (Kd * VEL) / H_T);
    OUTF = 0.96*OUTF + 0.04* out;
    
    if (OUTF >  1023) OUTF = 1023;
    if (OUTF < -1023) OUTF = -1023;
    
    if (OUTF>0) {ledcWrite(MAChannel, 0);ledcWrite(MBChannel, OUTF);}
    else {ledcWrite(MBChannel, 0);ledcWrite(MAChannel, -OUTF);}

    Serial.print(CWcounter);
    Serial.print("\t");
    Serial.print(POS);
    Serial.print("\t");
    Serial.print(REF_F);
    Serial.print("\t");
    Serial.print(erro);
    Serial.print("\t");
    Serial.println(OUTF);

    lasterro = erro;
  }
}

void loop() {
  ;
}
