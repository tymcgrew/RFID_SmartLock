/*
  Tyler McGrew
  RC522 RFID reader implementation on ESP-32 with LCD display and peripherals
*/

#include <Arduino.h>
#include "esp_system.h"
#include "RC522.h"
#include "LCD.h"
#include <SPI.h>
#include "Upload.h"

#define speaker 26
#define motorA 2
#define motorB 4

hw_timer_t *timer = NULL;
volatile byte RC522_SCLK_state = 0;
volatile byte rising_RC522_SCLK_edge = 0;
volatile byte falling_RC522_SCLK_edge = 0;
volatile byte RC522_SCLK_en = 0;

const int LCD_spiClk = 1000000;
SPIClass *LCD_SPI;
SPISettings *LCD_SPISettings;

bool online = false;


void IRAM_ATTR intRoutine() {
  RC522_SCLK_state = (RC522_SCLK_en) ? !RC522_SCLK_state : 0; // SPI Clock runs at fixed frequency given by RC522_SCLK_freq when enabled, else 0
  digitalWrite(RC522_SCLK, RC522_SCLK_state);
  rising_RC522_SCLK_edge = RC522_SCLK_state;   // Sets rising_RC522_SCLK_edge on rising edge, up to functions to use and reset this
  falling_RC522_SCLK_edge = !RC522_SCLK_state; // sets falling_RC522_SCLK_edge on falling edge, up to functions to use and reset this
}

void welcomeTone() {
  for (int repeats = 0; repeats < 500; repeats++) {
    for (int i = 0; i < 256; i+=20) {
      dacWrite(speaker, i);  
    }
    for (int i = 254; i >= 1; i-=20) {
      dacWrite(speaker, i);
    }
  }
  delay(50);
  for (int repeats = 0; repeats < 500; repeats++) {
    for (int i = 0; i < 256; i+=20) {
      dacWrite(speaker, i);  
    }
    for (int i = 254; i >= 1; i-=20) {
      dacWrite(speaker, i);
    }
  }
}

void invalidTone() {
  for (int repeats = 0; repeats < 200; repeats++) {
    for (int i = 0; i < 256; i+=1) {
      dacWrite(speaker, i); 
      delayMicroseconds(10); 
    }
  }
}

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

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);

  delay(1000);
  LCD_init();
  RC522_init();
  online = WiFi_init();
}

void loop() {
  // Default screen: screen-saver
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
    LCD_welcome(uid);
    welcomeTone();
    openLock();
    if (online)
      postData(String(hex_string), "YES");
    else
      delay(2000); 
  }
  else {
    LCD_invalid(uid);
    invalidTone();
    if (online)
      postData(String(hex_string), "NO");
    else
      delay(2000);
  }

  
}