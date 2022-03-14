#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
//#include <OTA.h>

#define DEBUG 1
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
  if (mac_addr == broadcastAddress)
    Motor(data[0], data[1]);
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
  analogWriteFreq(50);
  analogWriteRange(255);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop()
{
  ArduinoOTA.handle();
  analogWrite(W_pwm, 255);
  // Motor(-255,-255);
  delay(100);
  ArduinoOTA.handle();
  analogWrite(W_pwm, 0);

  Motor(0, 0);
  delay(1000);
}