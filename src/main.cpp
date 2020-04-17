/*
  Tyler McGrew
  RC522 RFID reader implementation on ESP-32 with LCD display and peripherals
  https://github.com/tymcgrew/RFID_SmartLock
*/

#include <Arduino.h>
#include "esp_system.h"
#include "RC522.h"
#include "LCD.h"
#include <SPI.h>
#include "Upload.h"

/*
  Defining a few pins
*/
#define speaker 26
#define motorA 2
#define motorB 4

/*
  Is the WiFi connected?
*/
bool online = false;


/*
  Plays two short, high-pitched beeps on the speaker
*/
void welcomeTone() {
  // Triangle wave form /\/\/\/\/\/\ at a fairly high frequency, not sure the exact pitch
  for (int repeats = 0; repeats < 500; repeats++) {
    for (int i = 0; i < 256; i+=20) {
      dacWrite(speaker, i);  
    }
    for (int i = 254; i >= 1; i-=20) {
      dacWrite(speaker, i);
    }
  }

  // Wait a bit
  delay(50);

  // Second beep, same as the first
  for (int repeats = 0; repeats < 500; repeats++) {
    for (int i = 0; i < 256; i+=20) {
      dacWrite(speaker, i);  
    }
    for (int i = 254; i >= 1; i-=20) {
      dacWrite(speaker, i);
    }
  }
}

/*
  Plays a longer, low-pitches tone on the speaker
*/
void invalidTone() {
  // Triangle wave form /\/\/\/\/\/\ at a lower frequency, not sure the exact pitch
  for (int repeats = 0; repeats < 200; repeats++) {
    for (int i = 0; i < 256; i+=1) {
      dacWrite(speaker, i); 
      delayMicroseconds(10); 
    }
  }
}

/*
  Turns a DC motor (H-Bridge configuration) back and forth to simulate opening and closing a lock
*/
void openLock() {
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, LOW);
  delay(2000);

  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);
  delay(200);

  digitalWrite(motorA, LOW);
  digitalWrite(motorB, HIGH);
  delay(2000);

  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);
}

void setup() {
  Serial.begin(9600);

  // Intialize motor outputs
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);

  delay(1000);

  // Intiazlize LCD
  LCD_init();
  // Initialize RFID Reader
  RC522_init();
  // Check if WiFi is available and connect if so 
  online = WiFi_init();
}

void loop() {
  // Default screen: screen-saver aat 10Hz
  LCD_screenSaver();
  delay(100);

  // Check for card
  if (!RC522_isNewCardPresent())
    return;

  // Get card's UID
  byte uid_bytes[4];
  if (!PICC_getUID(uid_bytes))
    return;

  // Convert UID to int
  uint32_t uid = 0;
  for (int i = 0; i < 4; i++)
    uid += uid_bytes[i] << ((3-i) * 8);
  Serial.println(uid, HEX);

  // Convert to Hex String
  char hex_string[20];
  sprintf(hex_string, "%X", uid);

  // Check if UID is authorized
  if (uid == 0x14FF5B2B) {
    // Put the welcome screen on the LCD
    LCD_welcome(uid);
    // Play welcome tone
    welcomeTone();
    // Turn lock motor
    openLock();
    // Post data to google doc if system is online
    if (online)
      postData(String(hex_string), "YES");
    else
      delay(2000); 
  }
  else {
    // Put the invalid ID screen on the LCD
    LCD_invalid(uid);
    // Play the invalid tone
    invalidTone();
    // Post data to google doc if system is online
    if (online)
      postData(String(hex_string), "NO");
    else
      delay(2000);
  }
}