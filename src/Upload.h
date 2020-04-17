/*
  Functions for connecting to WiFi and uploading info to google doc
  HTTP Example: https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/examples/BasicHttpClient/BasicHttpClient.ino
*/

#include "Wifi.h"
#include <HTTPClient.h>

bool WiFi_init();
void postData(String UID_string, String is_valid);