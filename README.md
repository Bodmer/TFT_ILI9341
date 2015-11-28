# TFT_ILI9341

An Arduino IDE compatible graphics and fonts library for AVR processors with a driver for the ILI9341 based TFT displays.

The library contains proportional fonts, different sizes can be enabled/disabled at compile time to optimise the use of FLASH memory.  The library has been tested with the UNO, Mega (ATmega328 or ATmega2560 processor) and the Leonardo compatible boards (ATmega32u4 processor).

The library is designed to be slim and fast. Typically a clear screen for a 320 x 240 TFT will complete in only 174ms. Images (320 x 240) can be pulled from a SD Card and drawn in 400ms, not bad for a humble UNO!

The library is based on the Adafruit GFX library and the aim is to retain compatibility. Significant additions have been made to the library to boost the speed for AVR processors (it is typically 3 to 10 times faster) and to add new features. The new graphics functions include different size proportional fonts and formatting features. There are a significant number of example sketches to demonstrate the different features.

Configuration of the library font selections, pins used to interface with the TFT and other features is made by editting the User_Setup.h file in the library folder.  Fonts and features can easily be disabled by commenting out lines.

Example connections between the Arduino board and the TFT can be found here:

http://www.instructables.com/id/Arduino-TFT-display-and-font-library/

Soon a dedicated Instructable will be available to document the library features, until then do explore the example sketches.
Have fun!
