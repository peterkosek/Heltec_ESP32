#include "telnet_debug.h"
#include <WiFi.h>
#include <WiFiClient.h>

static WiFiServer telnetServer(23);
static WiFiClient telnetClient;

void initTelnet(const char* ssid, const char* password) {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n[Telnet] Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[Telnet] WiFi connected.");
  Serial.print("[Telnet] IP address: ");
  Serial.println(WiFi.localIP());

  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("[Telnet] Server started on port 23");
}

void telnetLog(const String& message) {
  if (!telnetClient || !telnetClient.connected()) {
    telnetClient = telnetServer.available();  // grab a new client if present
  }

  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  }
}

void handleTelnet() {
  if (telnetServer.hasClient()) {
    if (telnetClient.connected()) {
      telnetClient.stop();  // kick out the previous client
    }
    telnetClient = telnetServer.available();
    telnetClient.println("[Telnet] Connected to ESP32");
  }
}
