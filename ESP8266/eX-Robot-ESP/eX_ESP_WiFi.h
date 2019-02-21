#ifndef EX_ESP_WIFI_H
#define EX_ESP_WIFI_H

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiUDP UdP;

#ifdef STATION
const char *ssid       = "linksys-RMDC"; // my home ssid
const char *password   = "0123456789";   // my password
#endif
uint16_t   RxPort      = 2222;
uint16_t   TxPort      = 2223;

uint8_t    MAC_array[6];                  // MAC address of Wifi module
char       MAC_char[18];

char       PacketBuffer[128];  // buffer to store incoming and outgoing packets
int        PacketSize        = 0;

union{
  unsigned char Buff[4];
  float d;
}u;

void WiFi_Start()
{
//  WiFi.disconnect();
#ifdef SOFTAP
  Serial.println("\nStarting WIFI in SOFTAP mode");
  // Generate Soft AP. SSID=X-RC-ESP-CH03_XXXXXX, PASS=87654321
  char *APname = "X-RC-ESP-CH03_XXXXXX";
  WiFi.macAddress(MAC_array);
  for (int i = 0; i < sizeof(MAC_array); ++i){
    sprintf(MAC_char, "%s%02x:", MAC_char, MAC_array[i]);
  }
  APname[14] = MAC_char[9];
  APname[15] = MAC_char[10];
  APname[16] = MAC_char[12];
  APname[17] = MAC_char[13];
  APname[18] = MAC_char[15];
  APname[19] = MAC_char[16];
  Serial.println("");
  Serial.print("MAC:");
  Serial.println(MAC_char);
  Serial.print("SSID:");
  Serial.println(APname);
  const char *ssid       = APname;
  const char *password   = "87654321";      // local AP password
  WiFi.softAP(ssid, password);    // password can be added as 2nd parameter
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
#endif
#ifdef STATION
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  Serial.println("");
  Serial.println("Starting WIFI in STATION mode");
  WiFi.mode(WIFI_STA);          // WIFI_AP_STA use dual mode without OTA
  delay(1000);    
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
#endif
  UdP.begin(RxPort);

// OTA start #############################################################
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
// OTA end ###############################################################
}

boolean WiFi_MSG_Read()
{
  PacketSize = UdP.parsePacket();
  if (!PacketSize) return false;
  UdP.read(PacketBuffer,PacketSize);  // read the packet into the buffer
  return true;
}

void WiFi_MSG_Send_Float(char *c, int msgSize, float p)
{
  u.d = p;
  c[msgSize-4] = u.Buff[3];
  c[msgSize-3] = u.Buff[2];
  c[msgSize-2] = u.Buff[1];
  c[msgSize-1] = u.Buff[0];
  UdP.beginPacket(UdP.remoteIP(), TxPort);
  UdP.write(c, msgSize);
  UdP.endPacket();
}

#endif  // EX_ESP_WIFI_H
