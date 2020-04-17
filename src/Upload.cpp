/*
  Functions for connecting to WiFi and uploading info to google doc
  HTTP Example: https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/examples/BasicHttpClient/BasicHttpClient.ino
*/

#include "Upload.h"

/*
  Certificate for HTTPS
*/
const char* root_ca  =\
"-----BEGIN CERTIFICATE-----\n"\
"MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n"\
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"\
"DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n"\
"SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n"\
"GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n"\
"AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n"\
"q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n"\
"SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n"\
"Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n"\
"a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n"\
"/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n"\
"AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n"\
"CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n"\
"bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n"\
"c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n"\
"VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n"\
"ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n"\
"MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n"\
"Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n"\
"AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n"\
"uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n"\
"wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n"\
"X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n"\
"PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n"\
"KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n"\
"-----END CERTIFICATE-----\n";

/*
  Two sets of SSIDs and passwords for use in two locations
*/
const char* ssid = "****";
const char* password = "****";
const char* ssid2 = "****";
const char* password2 = "****";

/*
  More network info
*/
const char *GScriptID = "AKfycbzYPSzUWES48WMIpE4-Hn_WeyyYr9WWoTXwO7ySPs8gAfrP-UNJ";
const int httpsPort = 443;   // default HTTPS port
const char* host = "script.google.com";
// Format URL
String url = String("/macros/s/") + GScriptID + "/exec?";
WiFiClientSecure client;

/*
  Attempt to establish wifi connection
*/
bool WiFi_init() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Try for 5 seconds to connect to first network
  long startTime = millis();
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - startTime) < 5000)) {
    Serial.print(".");
    delay(500);    
  }

  // Try for 5 seconds to connect to second network if not connected to the first
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid2, password2);
    long startTime = millis();
    while ((WiFi.status() != WL_CONNECTED) && ((millis() - startTime) < 5000)) {
      Serial.print(".");
      delay(500);    
    }
  }

  // Return false if neither connected
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
    

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Connect to host
  Serial.print(String("Connecting to "));
  Serial.println(host);
  bool WiFiFlag = false;
  for (int i = 0 ; i < 5; i++) {
    int retval = client.connect(host, httpsPort);
    if (retval == 1)  {
      WiFiFlag = true;
      break;
    }
    else {
      Serial.println("Connection failed. Retrying...");
    }
  }
  Serial.println("Connection Status: " + String(client.connected()));
  Serial.flush();

  // If couldn't connect to host (no internet or google down), return false
  if (!WiFiFlag) {
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    Serial.println("Exiting...");
    Serial.flush();
    return false;
  }

  // Everything is online
  return true;
}

/*
  Post data through HTTPS
*/
void postData(String UID_string, String is_valid) {
  HTTPClient http;

  // Format URL with data
  String urlFinal = String("https://") + host + url + "id=" + "Data" + "&UID=" + UID_string + "&Valid=" + String(is_valid);
  Serial.println(urlFinal);
  Serial.println("Making a request");

  // Send HTTP get request
  http.begin(urlFinal, root_ca);
  int httpCode = http.GET();
  http.end();

  // Debug payload
  Serial.println(": done"+httpCode);
}