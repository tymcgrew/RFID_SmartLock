/*
    Code Reference:
    https://github.com/sparkfun/GraphicLCD_Nokia_5110/blob/master/Firmware/Nokia-5100-LCD-Example/LCD_Functions.h
    Datasheet:
    http://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
*/

#include <Arduino.h>
#include "esp_system.h"
#include "LCD.h"
#include <SPI.h>

static char LCD_displayArray[LCD_width*LCD_height/8] = {0x0};


void LCD_sendCommand(byte command) {
  digitalWrite(LCD_MODE, LCD_commandMode);
  LCD_SPI->beginTransaction(*LCD_SPISettings);
  digitalWrite(LCD_SS, LOW);
  LCD_SPI->transfer(command);
  digitalWrite(LCD_SS, HIGH);
  LCD_SPI->endTransaction();
}

void LCD_sendDataByte(byte dataByte) {
  digitalWrite(LCD_MODE, LCD_dataMode);
  LCD_SPI->beginTransaction(*LCD_SPISettings);
  digitalWrite(LCD_SS, LOW);
  LCD_SPI->transfer(dataByte);
  digitalWrite(LCD_SS, HIGH);
  LCD_SPI->endTransaction();
}

void LCD_updateDisplay() {
  LCD_resetAddress();
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_sendDataByte(LCD_displayArray[i]);
}

void LCD_resetAddress() {
  LCD_setAddress(0,0);
}

void LCD_setAddress(uint8_t x, uint8_t y) {
  byte x_byte = B10000000 | x;
  byte y_byte = B01000000 | (y & B00000111);
  LCD_sendCommand(x_byte);
  LCD_sendCommand(y_byte);
}

void LCD_clearDisplay() {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = 0x0;
  LCD_updateDisplay();
}

void LCD_invertDisplay() {
  static bool inverted = false;
  byte val;
  if (inverted)
    val = B00001100;   // Normal display mode
  else
    val = B00001101;   // Inverted display mode
  LCD_sendCommand(val);
  inverted = !inverted;
}

void LCD_drawChar(byte charVal, int charPos) {
  int xPos[8] = {336 + 5, 336 + 15, 336 + 25, 336 + 35, 336 + 45, 336 + 55, 336 + 65, 336 + 75};
  for (int i = 0; i < 5; i++)
    LCD_displayArray[xPos[charPos]+i] = LCD_hexChars[charVal][i];
}

void LCD_drawPixel(int x, int y) {
  if (x < 0 || y < 0 || x >= LCD_width || y >= LCD_height)
    return;
  int arrayIndex = (y / 8) * LCD_width + x;
  byte val = LCD_displayArray[arrayIndex];
  val |= 1 << (y % 8);
  LCD_displayArray[arrayIndex] = val;
}

void LCD_drawRect(int x, int y, int width, int height) {
  if (x < 0 || y < 0)
    return;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      if (x + i < LCD_width && y + j < LCD_height) {
        LCD_drawPixel(x+i, y+j);
      }
    }
  }
}

void LCD_screenSaver() {
  LCD_clearDisplay();

  int boxWidth = 12;
  int boxHeight = 12;
  static int boxX = rand() % 84;
  static int boxY = rand() % 48;
  static int boxDx = 3;
  static int boxDy = 2;

  boxX += boxDx;
  boxY += boxDy;

  if (boxX <= 0) {
    boxDx = -boxDx;
    boxX = 0;
  }
  else if (boxY <= 0) {
    boxDy = -boxDy;
    boxY = 0;
  }
  else if (boxX + boxWidth > LCD_width) {
    boxDx = -boxDx;
    boxX = LCD_width - boxWidth;
  }
  else if (boxY + boxHeight > LCD_height) {
    boxDy = -boxDy;
    boxY = LCD_height - boxHeight;
  }

  if (
    (boxX + boxWidth == LCD_width && boxY + boxHeight == LCD_height) ||
    (boxX == 0 && boxY + boxHeight == LCD_height) ||
    (boxX + boxWidth == LCD_width && boxY == 0) ||
    (boxX == 0 && boxY == 0) 
    )
    LCD_invertDisplay();
  
  LCD_drawRect(boxX, boxY, boxWidth, boxHeight);
  LCD_updateDisplay();
}

void LCD_placeID() {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_placeIDArray[i];
  LCD_updateDisplay();
}

void LCD_welcome(uint32_t uid) {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_welcomeArray[i];  
  for (int i = 7; i >= 0; i--) {
    byte uid_byte = ((uid >> (i*4)) & 0xF);
    LCD_drawChar(uid_byte, 7-i);
  }
  LCD_updateDisplay();
}

void LCD_invalid(uint32_t uid) {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_invalidArray[i];  
  for (int i = 7; i >= 0; i--) {
    byte uid_byte = ((uid >> (i*4)) & 0xF);
    LCD_drawChar(uid_byte, 7-i);
  } 
  LCD_updateDisplay(); 
}

void LCD_init() {
  pinMode(LCD_SS, OUTPUT); 
  pinMode(LCD_RST, OUTPUT); 
  pinMode(LCD_MODE, OUTPUT); 
  digitalWrite(LCD_SS, HIGH);     // Active Low chip select
  digitalWrite(LCD_RST, HIGH);    // Active High enabled, reset on low
  digitalWrite(LCD_MODE, LOW);    // Start command mode
  dacWrite(LCD_BL, 255);          // Full power to backlight

  digitalWrite(LCD_RST, LOW);
  delay(1); // Wait 1 ms
  digitalWrite(LCD_RST, HIGH);

  // LCD_SPI init
  LCD_SPI = new SPIClass(VSPI);
  LCD_SPI->begin(LCD_SCLK, 15, LCD_MOSI, LCD_SS);   // MISO is set to unused pin
  LCD_SPISettings = new SPISettings(LCD_spiClk, MSBFIRST, SPI_MODE0);

  // Initializing LCD:
  // Documentation Part 13 (Page 22)
  LCD_sendCommand(B00100001);     // Enter extended instruction set by setting bit 0
  LCD_sendCommand(B10111000);     // Set Vop (bits 6:0) for contrast, VLCD = 3.06 + Vop*.06 => ~6V here
  LCD_sendCommand(B00000100);     // Set temperature coefficient to temperature coefficient 0 
  LCD_sendCommand(B00010101);     // Set LCD bias system
  
  LCD_sendCommand(B00100000);     // Enter basic instruction set by resetting bit 0
  LCD_sendCommand(B00001100);     // Set display configuration to normal mode
}