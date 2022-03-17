#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>

#define robo
//#define controle

#if defined(robo)
#include <ArduinoOTA.h>
#include <Servo.h>

#define DEBUG 0
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define AI1 13
#define AI2 12
#define BI1 14
#define BI2 16
#define W_pwm 5
uint8_t value[6];
unsigned long last_receive;

Servo Arma;

void SetupOTA(const char *nameprefix, const char *ssid, const char *password)
{
  // Configure the hostname
  uint16_t maxlen = strlen(nameprefix) + 7;
  char *fullhostname = new char[maxlen];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(nameprefix);
  delete[] fullhostname;

  // Configure and start the WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    debugln("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232); // Use 8266 port if you are working in Sloeber IDE, it is fixed there and not adjustable

  // No authentication by default
  ArduinoOTA.setPassword("0101");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
                     {
                       //NOTE: make .detach() here for all functions called by Ticker.h library - not to interrupt transfer process in any way.
                       String type;
                       if (ArduinoOTA.getCommand() == U_FLASH)
                         type = "sketch";
                       else // U_SPIFFS
                         type = "filesystem";

                       // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                       debugln("Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { debugln("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         debugln("\nAuth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         debugln("\nBegin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         debugln("\nConnect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         debugln("\nReceive Failed");
                       else if (error == OTA_END_ERROR)
                         debugln("\nEnd Failed"); });

  ArduinoOTA.begin();

  debugln("OTA Initialized");
  debug("IP address: ");
  debugln(WiFi.localIP());
}

void Motor(signed int velocidade1, signed int velocidade2)
{

  if (velocidade1 > 0)
  {
    analogWrite(AI1, velocidade1);
    analogWrite(AI2, 0);
  }
  else
  {
    if (velocidade1 == 0)
    {
      analogWrite(AI1, 0);
      analogWrite(AI2, 0);
    }
    else
    {
      analogWrite(AI1, 0);
      analogWrite(AI2, -velocidade1);
    }
  }

  if (velocidade2 > 0)
  {
    analogWrite(BI1, velocidade2);
    analogWrite(BI2, 0);
  }
  else
  {
    if (velocidade2 == 0)
    {
      analogWrite(BI1, 0);
      analogWrite(BI2, 0);
    }
    else
    {
      analogWrite(BI1, 0);
      analogWrite(BI2, -velocidade2);
    }
  }
}

uint8_t broadcastAddress[6] = {0xC3, 0x5B, 0xBE, 0x60, 0xB6, 0x63};
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{
  last_receive = millis();
  for (int i = 0; i < 6; i++)
  {
    value[i] = data[i];
    debug(data[0]);
    debug("  ");
  }
  debugln();
}

void ConnectEspNow()
{
  if (esp_now_init() != 0)
    debugln("ESPNow Init Failed");
  esp_now_register_recv_cb(OnDataRecv);
}

void setup()
{
  Serial.begin(115200);
  SetupOTA("Tecna", "LenHide", "01010101");
  ConnectEspNow();
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(W_pwm, OUTPUT);
  analogWriteFreq(50000);
  // analogWriteFreq(50);
  // analogWriteRange(255);
  Arma.attach(W_pwm);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop()
{
  ArduinoOTA.handle();
  if (millis() - last_receive < 500)
  {
    // Arma.write(map(255 - data[0], 0, 255, 0, 180));
    int Y = (128 - value[1]) * 2, X = (128 - value[2]) * 2;
    // if (Y > 20 || Y < 20 || X > 20 || X < 20)
    Motor(Y - X, Y + X);
  }
  else
  {
    Arma.write(0);
    Motor(0, 0);
  }

  /*ArduinoOTA.handle();
  Arma.write(180);
  // analogWrite(W_pwm, 255);
  //  Motor(-255,-255);
  delay(2000);
  ArduinoOTA.handle();
  // analogWrite(W_pwm, 0);
  Arma.write(0);
  Motor(0, 0);
  delay(1000);
  */
}
#elif defined(controle)

unsigned long Ch_time[5];
byte value[6];
uint8_t broadcastAddress[6] = {0xCC, 0x50, 0xE3, 0x56, 0xAD, 0xF4}; // CC:50:E3:56:AD:F4
const char *ssid = "LenHide";
const char *password = "01010101";
bool send = 0;

void ICACHE_RAM_ATTR CH0();
void ICACHE_RAM_ATTR CH1();
void ICACHE_RAM_ATTR CH2();
void ICACHE_RAM_ATTR CH3();
void ICACHE_RAM_ATTR CH4();
void ICACHE_RAM_ATTR CH5();

void CH0()
{
  unsigned long time = micros();
  if (digitalRead(12))
  {
    Ch_time[0] = time;
    send = 1;
  }
  else if (Ch_time[0] < time)
    value[0] = map(constrain(time - Ch_time[0], 1070, 1930), 1070, 1930, 0, 255);
}
void CH1()
{
  unsigned long time = micros();
  if (digitalRead(13))
    Ch_time[1] = time;
  else if (Ch_time[1] < time)
    value[1] = map(constrain(time - Ch_time[1], 1070, 1920), 1070, 1920, 0, 255);
}
void CH2()
{
  unsigned long time = micros();
  if (digitalRead(14))
    Ch_time[2] = time;
  else if (Ch_time[2] < time)
    value[2] = map(constrain(time - Ch_time[2], 1090, 1930), 1090, 1930, 0, 255);
}
void CH3()
{
  unsigned long time = micros();
  if (digitalRead(4))
    Ch_time[3] = time;
  else if (Ch_time[3] < time)
    value[3] = map(constrain(time - Ch_time[3], 1090, 1910), 1090, 1910, 0, 255);
}
void CH4()
{
  unsigned long time = micros();
  if (digitalRead(5))
    Ch_time[4] = time;
  else if (Ch_time[4] < time)
    value[4] = map(constrain(time - Ch_time[4], 1070, 1930), 1070, 1930, 0, 255);
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    Serial.println("Delivery success");
  }
  else
  {
    Serial.println("Delivery fail");
  }
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(12), CH0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(13), CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14), CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), CH4, CHANGE);

  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // esp_now_register_send_cb(OnDataSent);
  //  Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
void loop()
{
  if (send)
  {
    esp_now_send(broadcastAddress, (byte *)&value, sizeof(value));
    send = 0;
    //}

    /*
     // FAIL SAFE
     for (int i = 0; i < 5; i++)
       if (Ch_time[i] > micros() || micros() - Ch_time[i] > 50000)
         value[5] = 404;
     if (value[5] == 404)
       Serial.print("  FAIL SAFE  ");
   */

    for (int i = 0; i < 6; i++)
    {
      Serial.print(value[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(1);
}

#endif
