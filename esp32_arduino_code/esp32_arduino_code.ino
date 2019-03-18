#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <PubSubClient.h>

#define magic_N 11528 //22*524////inicial 11484//22*522
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

#define Kp 360
#define Kd 0.5
#define H_T 0.1

#define ID_MQTT  "Pedroza08"             //Informe um ID unico e seu. Caso sejam usados IDs repetidos a ultima conexão irá sobrepor a anterior. 
#define TOPIC_SUBSCRIBE "referencia"   //Informe um Tópico único. Caso sejam usados tópicos em duplicidade, o último irá eliminar o anterior.
#define TOPIC_PUBLISH "posição"

volatile int GG, RR;
volatile int CWcounter = 0;
volatile int timerFlag = 0;

int erro, lasterro = 0;
int POS, oldPOS = 0, VEL = 0;
int REF = 0, nREF = 0, REF_F = 0;
int out, OUTF = 0;

//WiFi
const char* SSID = "PIRADATA";               // SSID / nome da rede WiFi que deseja se conectar
const char* PASSWORD = "guifernandabf";      // Senha da rede WiFi que deseja se conectar

//MQTT Server
const char* BROKER_MQTT = "iot.eclipse.org"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883;                      // Porta do Broker MQTT

WiFiClient wifiClient;
PubSubClient MQTT(wifiClient);        // Instancia o Cliente MQTT passando o objeto espClient

void mantemConexoes();  //Garante que as conexoes com WiFi e MQTT Broker se mantenham ativas
void conectaWiFi();     //Faz conexão com WiFi
void conectaMQTT();     //Faz conexão com Broker MQTT
void recebeValores(char* topic, byte* payload, unsigned int length);
void enviaPacote();

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//DONT TOUCH THIS SHIT!!! IT IS FUCKING WORKING!!!
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR handleInterruptG() {
  if (digitalRead(pinG) != GG) {
    //(GG != oldGG)?(GG==RR?CWcounter--:CWcounter++):(GG==RR?CWcounter++:CWcounter--);
    GG = !GG;
    RR = digitalRead(pinR);
    portENTER_CRITICAL_ISR(&mux);
    GG == RR ? CWcounter-- : CWcounter++;
    portEXIT_CRITICAL_ISR(&mux);
  }
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

  portENTER_CRITICAL_ISR(&mux);
  GG = digitalRead(pinG);
  RR = digitalRead(pinR);
  portEXIT_CRITICAL_ISR(&mux);
  Serial.print(GG);
  Serial.print("/");
  Serial.print(RR);
  Serial.print("/");
  Serial.println(CWcounter);

  conectaWiFi();
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(recebeValores);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

  attachInterrupt(digitalPinToInterrupt(pinG), handleInterruptG, CHANGE);

  while (true) {
    if (timerFlag > 0) {

      portENTER_CRITICAL(&timerMux);
      timerFlag--;
      portEXIT_CRITICAL(&timerMux);

      mantemConexoes();
      MQTT.loop();

      POS = CWcounter > 0 ? ((CWcounter % magic_N) * 360) / magic_N : ((magic_N + (CWcounter % magic_N)) * 360) / magic_N;
      enviaValores(POS);
      VEL = POS - oldPOS;
      if (VEL % 360 == 0) VEL = 0;
      if (VEL > 180) VEL -= 360;
      if (VEL < -180) VEL += 360;
      oldPOS = POS;

      //REF = (analogRead(pot) * 360) / 4095;
      if (nREF >= 0 and nREF <= 360) REF = nREF;
      //REF_F = 0.2 * REF_F + (1 - 0.2) * REF;
      REF_F = REF;

      erro  = REF_F > POS ? ((REF_F - POS) < 180 ? REF_F - POS : -POS - 360 + REF_F) : ((POS - REF_F) > 180 ? REF_F + 360 - POS : -POS + REF_F);

      out =  Kp * (erro - (Kd * VEL) / H_T);
      OUTF = 0.96 * OUTF + 0.04 * out;

      if (OUTF >  1023) OUTF = 1023;
      if (OUTF < -1023) OUTF = -1023;

      if (OUTF > 0) {
        ledcWrite(MAChannel, 0);
        ledcWrite(MBChannel, OUTF);
      }
      else {
        ledcWrite(MBChannel, 0);
        ledcWrite(MAChannel, -OUTF);
      }

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
}

void loop() {
  ;
}

void mantemConexoes() {
  if (!MQTT.connected()) {
    conectaMQTT();
  }

  conectaWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

void conectaWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Conectando-se na rede: ");
  Serial.print(SSID);
  Serial.println("  Aguarde!");

  WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso, na rede: ");
  Serial.print(SSID);
  Serial.print("  IP obtido: ");
  Serial.println(WiFi.localIP());
}

void conectaMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Conectando ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao Broker com sucesso!");
      MQTT.subscribe(TOPIC_SUBSCRIBE);
    }
    else {
      Serial.println("Não foi possivel se conectar ao broker.");
      Serial.println("Nova tentatica de conexao em 10s");
      delay(10000);
    }
  }
}

void recebeValores(char* topic, byte* payload, unsigned int length)
{
  String msg;
  //obtem a string do payload recebido
  for (int i = 0; i < length; i++)
  {
    char c = (char)payload[i];
    msg += c;
  }
  nREF = msg.toInt();
}

void enviaValores(int VV) {
  String PP = String(VV);
  int TT = PP.length() + 1;
  char CB[TT];
  PP.toCharArray(CB, TT);
  MQTT.publish(TOPIC_PUBLISH, CB);
}
