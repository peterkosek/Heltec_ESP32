#ifndef TELNET_DEBUG_H
#define TELNET_DEBUG_H

#include <Arduino.h>

void initTelnet(const char* ssid, const char* password);
void telnetLog(const String& message);
void handleTelnet();

#endif
