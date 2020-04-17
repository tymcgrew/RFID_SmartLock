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

// Initialize an array of bytes that represent the entire LCD's data to all zeros
static char LCD_displayArray[LCD_width*LCD_height/8] = {0x0};

/*
  Using a standard hardware SPI master implentation for the LCD
*/
const int LCD_spiClk = 1000000;
SPIClass *LCD_SPI;
SPISettings *LCD_SPISettings;

/*
  Send a command byte to the LCD
*/
void LCD_sendCommand(byte command) {
  // Change LCD mode to commandMode
  digitalWrite(LCD_MODE, LCD_commandMode);

  // Send command byte over SPI
  LCD_SPI->beginTransaction(*LCD_SPISettings);
  digitalWrite(LCD_SS, LOW);
  LCD_SPI->transfer(command);
  digitalWrite(LCD_SS, HIGH);
  LCD_SPI->endTransaction();
}

/*
  Send a data byte to the LCD
*/
void LCD_sendDataByte(byte dataByte) {
  // Change LCD mode to dataMode
  digitalWrite(LCD_MODE, LCD_dataMode);

  // Send data byte over SPI
  LCD_SPI->beginTransaction(*LCD_SPISettings);
  digitalWrite(LCD_SS, LOW);
  LCD_SPI->transfer(dataByte);
  digitalWrite(LCD_SS, HIGH);
  LCD_SPI->endTransaction();
}

/*
  Write displayArray bytes to the LCD
*/
void LCD_updateDisplay() {
  // Set pointer to (0,0)
  LCD_resetAddress();

  // Send each byte sequentially
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_sendDataByte(LCD_displayArray[i]);
}

/*
  Set address to (0,0)
*/
void LCD_resetAddress() {
  LCD_setAddress(0,0);
}

/*
  Set address to (x,y)
*/
void LCD_setAddress(uint8_t x, uint8_t y) {
  // X-address command byte format: 1,X6,X5,X4,X3,X2,X1,X0
  byte x_byte = B10000000 | x;
  // Y-address command byte format: 0,1,0,0,0,Y2,Y1,Y0
  byte y_byte = B01000000 | (y & B00000111);
  // Send command bytes
  LCD_sendCommand(x_byte);
  LCD_sendCommand(y_byte);
}

/*
  Set all pixels on LCD to 0
*/
void LCD_clearDisplay() {
  // Set all LCD array bytes to 0x0
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = 0x0;
  // Update display
  LCD_updateDisplay();
}

/*
  Flip all pixels by setting display mode
*/
void LCD_invertDisplay() {
  // Keep track of display mode since we can't read from LCD
  static bool inverted = false;

  // Set new command byte based on current display mode
  byte val;
  if (inverted)
    val = B00001100;   // Normal display mode
  else
    val = B00001101;   // Inverted display mode

  // Send command byte and update variable
  LCD_sendCommand(val);
  inverted = !inverted;
}

/*
  Draw a 5x8 hex digit on the screen in one of eight positions
*/
void LCD_drawChar(byte charVal, int charPos) {
  // Eight positions spaced out equally across the screen
  int xPos[8] = {336 + 5, 336 + 15, 336 + 25, 336 + 35, 336 + 45, 336 + 55, 336 + 65, 336 + 75};

  // Write each of five bytes using char info from LCD_hexChars array
  for (int i = 0; i < 5; i++)
    LCD_displayArray[xPos[charPos]+i] = LCD_hexChars[charVal][i];
}

/*
  Draw a single pixel to LCD array.
  This converts an (x,y) coordinate to a bit and byte index in the LCD displayArray
*/
void LCD_drawPixel(int x, int y) {
  // Check if pixel is in bounds
  if (x < 0 || y < 0 || x >= LCD_width || y >= LCD_height)
    return;
  
  // Coordinate conversion math to find byte
  int arrayIndex = (y / 8) * LCD_width + x;

  // Retrieve current byte val from array
  byte val = LCD_displayArray[arrayIndex];

  // Set the correct pixel within the byte
  val |= 1 << (y % 8);

  // Write byte to array
  LCD_displayArray[arrayIndex] = val;
}

/*
  Draw a solid rectangle of pixels
*/
void LCD_drawRect(int x, int y, int width, int height) {
  // Check lower bounds
  if (x < 0 || y < 0)
    return;
  
  // For each pixel in width and height
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      // Draw pixel if in bounds
      if (x + i < LCD_width && y + j < LCD_height) {
        LCD_drawPixel(x+i, y+j);
      }
    }
  }
}

/*
  Updates the screensaver:
  A rectangle that bounces around the screen and inverts colors if it hits a corner
*/
void LCD_screenSaver() {
  // Clear the screen to draw a new rectangle
  LCD_clearDisplay();

  int boxWidth = 12;
  int boxHeight = 12;
  // static so values persist across function calls, only defined here the first time the function is called
  static int boxX = rand() % 84;
  static int boxY = rand() % 48;
  static int boxDx = 3;
  static int boxDy = 2;

  // Each time, move box
  boxX += boxDx;
  boxY += boxDy;

  // Flip velocities 
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

  // Invert colors if box hit one of the corners
  if (
    (boxX + boxWidth == LCD_width && boxY + boxHeight == LCD_height) ||
    (boxX == 0 && boxY + boxHeight == LCD_height) ||
    (boxX + boxWidth == LCD_width && boxY == 0) ||
    (boxX == 0 && boxY == 0) 
    )
    LCD_invertDisplay();
  
  // Draw new box
  LCD_drawRect(boxX, boxY, boxWidth, boxHeight);
  LCD_updateDisplay();
}

/*
  Draws a predetermined image asking the user to place ID near card reader
*/
void LCD_placeID() {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_placeIDArray[i];
  LCD_updateDisplay();
}

/*
  Draws a predetermined image overlaid with drawn hex chars of the UID
*/
void LCD_welcome(uint32_t uid) {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_welcomeArray[i];  
  for (int i = 7; i >= 0; i--) {
    byte uid_byte = ((uid >> (i*4)) & 0xF);
    LCD_drawChar(uid_byte, 7-i);
  }
  LCD_updateDisplay();
}

/*
  Draws a predetermined image overlaid with draw hex chars of the UID
*/
void LCD_invalid(uint32_t uid) {
  for (int i = 0; i < LCD_width*LCD_height/8; i++)
    LCD_displayArray[i] = LCD_invalidArray[i];  
  for (int i = 7; i >= 0; i--) {
    byte uid_byte = ((uid >> (i*4)) & 0xF);
    LCD_drawChar(uid_byte, 7-i);
  } 
  LCD_updateDisplay(); 
}

/*
 Intialize LCD screen
*/
void LCD_init() {
  // Configure digital pins
  pinMode(LCD_SS, OUTPUT); 
  pinMode(LCD_RST, OUTPUT); 
  pinMode(LCD_MODE, OUTPUT); 
  digitalWrite(LCD_SS, HIGH);     // Active Low chip select
  digitalWrite(LCD_RST, HIGH);    // Active High enabled, reset on low
  digitalWrite(LCD_MODE, LOW);    // Start command mode
  // Full voltage to backlight (this is limited by resistors to less than 10mA)
  dacWrite(LCD_BL, 255);          

  // Clean reset signal
  digitalWrite(LCD_RST, LOW);
  delay(1); // Wait 1 ms
  digitalWrite(LCD_RST, HIGH);

  // Intialize hardware SPI object 
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