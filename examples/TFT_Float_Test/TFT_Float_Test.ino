/*
Tests the updated floating point plot function
The datum is set to the centre of the 320 x 240 screen
so numbers are printed the middle.

The last test shows the datum point as a red dot.

Normally strings are printed relative to the top left corner but this can be
changed with the setTextDatum() function. The library has #defines for:

TL_DATUM  0 //Top left
TC_DATUM  1 //Top centre
TR_DATUM  2 //Top right
ML_DATUM  3 //Middle left
MC_DATUM  4 //Middle centre
MR_DATUM  5 //Middle right
BL_DATUM  6 //Bottom left
BC_DATUM  7 //Bottom centre
BR_DATUM  8 //Bottom right

 
 Needs fonts 2 and 6

 Make sure all the required fonts are loaded by editting the
 User_Setup.h file in the TFT_ILI9341 library folder.

 If using an UNO or Mega (ATmega328 or ATmega2560 processor) then for best
 performance use the F_AS_T option found in the User_Setup.h file in the
 TFT_ILI9341 library folder.

 The library uses the hardware SPI pins only:
   For UNO, Nano, Micro Pro ATmega328 based processors
      MOSI = pin 11, SCK = pin 13
   For Mega:
      MOSI = pin 51, SCK = pin 52

 The pins used for the TFT chip select (CS) and Data/command (DC) and Reset (RST)
 signal lines to the TFT must also be defined in the library User_Setup.h file.

 Sugested TFT connections for UNO and Atmega328 based boards
   sclk 13  // Don't change, this is the hardware SPI SCLK line
   mosi 11  // Don't change, this is the hardware SPI MOSI line
   cs   10  // Chip select for TFT display
   dc   9   // Data/command line
   rst  7   // Reset, you could connect this to the Arduino reset pin

 Suggested TFT connections for the MEGA and ATmega2560 based boards
   sclk 52  // Don't change, this is the hardware SPI SCLK line
   mosi 51  // Don't change, this is the hardware SPI MOSI line
   cs   47  // TFT chip select line
   dc   48  // TFT data/command line
   rst  44  // you could alternatively connect this to the Arduino reset

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  ######       TO SELECT THE FONTS AND PINS YOU USE, SEE ABOVE       ######
  #########################################################################
 */


#include <TFT_ILI9341.h> // Hardware-specific library
#include <SPI.h>

TFT_ILI9341 tft = TFT_ILI9341();       // Invoke custom library

unsigned long drawTime = 0;

void setup(void) {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);

}

void loop() {

  char tmp[12];
  
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // Datum is middle centre
  tft.setTextDatum(MC_DATUM);

  // Test floating point drawing function:
  // drawFloat(value, precision, x, y, font);
  
  float test = 67.125;
  tft.drawFloat(test, 4, 160, 120, 6);
    tft.drawString(dtostrf(test,4,4,tmp), 100, 200, 6);

  delay(1000);
  tft.fillScreen(TFT_BLACK);
  
  test = -0.555555;
  tft.drawFloat(test, 3, 160, 120, 6);
    tft.drawString(dtostrf(test,2,2,tmp), 100, 200, 6);

  delay(1000);
  tft.fillScreen(TFT_BLACK);
  
  test = 0.123;
  tft.drawFloat(test, 4, 160, 120, 6);

  tft.drawString(dtostrf(test,4,4,tmp), 100, 200, 6);

  delay(1000);
  tft.fillScreen(TFT_BLACK);

  // This does not work at the moment....
  test = 9999999;
  tft.drawFloat(test, 0, 160, 120, 6);
  tft.drawString(dtostrf(test,4,4,tmp), 100, 200, 6);
  delay(1000);

  //Plot the datum for the last number
  tft.fillCircle(160,120,5,TFT_RED);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("X", 160, 120, 2);
 
  delay(4000);
}








