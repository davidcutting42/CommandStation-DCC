// OLED Implementation of LCDDisplay class 
// Note: this file is optionally included by LCD_Implenentation.h
// It is NOT a .cpp file to prevent it being compiled and demanding libraraies even when not needed.
  
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 LCDDriver(OLED_DRIVER);
  
// DEVICE SPECIFIC LCDDisplay Implementation for OLED

LCDDisplay::LCDDisplay() {
    if(LCDDriver.begin(SSD1306_SWITCHCAPVCC, 0x3C) || LCDDriver.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      DIAG(F("\nOLED display found")); 
      delay(2000); // painful Adafruit splash pants! 
      lcdDisplay=this;
      LCDDriver.clearDisplay();
      LCDDriver.setTextSize(1);             // Normal 1:1 pixel scale
      LCDDriver.setTextColor(SSD1306_WHITE);            // Draw white text
      interfake(OLED_DRIVER,0);
      return;  
      }
    DIAG(F("\nOLED display not found\n"));
    }

  void LCDDisplay::interfake(int p1, int p2, int p3) {(void)p1; lcdRows=p2/8; (void)p3;}   

  void LCDDisplay::clearNative() {LCDDriver.clearDisplay();}

  void LCDDisplay::setRowNative(byte row) {
    // Positions text write to start of row 1..n and clears previous text 
    int y=8*row;
    LCDDriver.fillRect(0, y, LCDDriver.width(),   8, SSD1306_BLACK);
    LCDDriver.setCursor(0, y); 
    }
  
  void LCDDisplay::writeNative(char * b){ LCDDriver.print(b); }    
  
  void LCDDisplay::displayNative() { LCDDriver.display(); }  
  
