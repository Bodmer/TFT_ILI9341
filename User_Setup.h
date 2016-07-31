//                            USER DEFINED SETTINGS V16
//            Set fonts to be loaded, pins used and SPI control method


// ##################################################################################
//
// Define the fonts that are to be used here
//
// ##################################################################################

// Comment out the #defines below with // to stop that font being loaded
// As supplied font 8 is disabled by commenting out
//
// If all fonts are loaded the extra FLASH space required is about 17000 bytes...
// To save FLASH space only enable the fonts you need!

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
//#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.


// ##################################################################################
//
// Define the pins that are used to interface with the display here
//
// ##################################################################################

// We must use hardware SPI
// FYI Mega SCK is pin 52, MOSI is 51, UNO/NanoPro micro etc SCK is pin 13 and MOSI is 11
// Leonardo Pro micro SCK is pin 15 and MOSI is 16

// These are the control pins I use on my Mega setup
//   TFT_CS  47  // Chip select control pin
//   TFT_DC  48  // Data Command control pin
//   TFT_RST 44  // Reset pin (could connect to Arduino RESET pin)

// These are the control pins I use on my UNO/Nano/Pro Micro/ATmega328 setup
//   TFT_CS  10  // Chip select control pin
//   TFT_DC   9  // Data Command control pin
//   TFT_RST  7  // Reset pin (could connect to Arduino RESET pin)

// ###### EDIT THE PIN NUMBERS IN THE 3 LINES FOLLOWING TO SUIT YOUR SETUP ######

#define TFT_CS   10  // Chip select control pin
#define TFT_DC   8  // Data Command control pin
#define TFT_RST  7  // Reset pin (could connect to Arduino RESET pin)


// ##################################################################################
//
// Other speed up options
//
// ##################################################################################

// If your sketch uses the GLCD font in size 1 with background then uncomment
// this next line will speed up rendering x5, code size will increase 136 bytes
// Only worth it if you print lots of GLCD font text...

#define FAST_GLCD

// Uncomment the following #define to invoke a 20% faster drawLine() function
// This speeds up other funtions such as triangle outline drawing too
// Code size penalty is about 72 bytes

#define FAST_LINE

// Comment out the following #define to stop boundary checking and clipping
// for fillRectangle()and fastH/V lines. This speeds up other funtions such as text
// rendering where size>1. Sketch then must not draw graphics/text outside screen
// boundary. Code saving for no bounds check (i.e. commented out) is 316 bytes

//#define CLIP_CHECK

// Comment out the following #define if "SPI Transactions" do not need to be
// supported. Tranaction support is required if other SPI devices use interrupts.
// When commented out the code size will be ~700 bytes smaller and sketches will
// run slightly faster, so leave it commented out unless you need it!
// Transaction support is needed to work with SD libraru but not needed with TFT_SdFat

// #define SUPPORT_TRANSACTIONS

