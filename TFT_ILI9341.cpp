/***************************************************
  Arduino TFT graphics library targetted at the UNO
  and Mega boards.

  This library has been derived from the Adafruit_GFX
  library and the associated driver library. See text
  at the end of this file.

  This is a standalone library that contains the
  hardware driver, the graphics funtions and the
  proportional fonts.

  The larger fonts are Run Length Encoded to reduce their
  size.

  13/6/15 Merged the TFT_GFX and TFT_driver libraries
  14/6/15 Posted as version 10 on Instructables
  29/6/15 Updated to work with F_AS_T mode on Mega
  1/7/15  Minor bug fix and rendering RLE characters with true width
  26/7/15 Updated to use FastPin.h for fast toggling with any pin
  27/7/15 Added commandList to replace long setup function call list
  30/7/15 Added configurable coordinate datum for fonts
  31/7/15 Added Spellbinders improved line draw algorithm
  5/8/15  Added backupSPCR() and restoreSPCR() functions
  25/8/15 Added new setTextPadding() function to background pad text/numbers
 ****************************************************/

#include "TFT_ILI9341.h"

#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// Constructor , we must use hardware SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
TFT_ILI9341::TFT_ILI9341(int16_t w, int16_t h)
{
  _cs   = TFT_CS;
  _dc   = TFT_DC;
  _rst  = TFT_RST;
  _mosi  = _sclk = 0;

  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  *dcport |=  dcpinmask;
  pinMode(_dc, OUTPUT);

  *csport |= cspinmask;
  pinMode(_cs, OUTPUT);

  _width    = w;
  _height   = h;
  rotation  = 0;
  cursor_y  = cursor_x    = 0;
  textfont  = 1;
  textsize  = 1;
  textcolor   = 0xFFFF;
  textbgcolor = 0x0000;
  padX = 0;
  textwrap  = true;
  textdatum = 0; // Left text alignment is default
}

void TFT_ILI9341::spiwrite(uint8_t c)
{
  uint8_t backupSPCR = SPCR;
  SPCR = mySPCR;
  SPDR = c;
  while (!(SPSR & _BV(SPIF)));
  SPCR = backupSPCR;
}


void TFT_ILI9341::writecommand(uint8_t c)
{
  *dcport &=  ~dcpinmask;
  *csport &= ~cspinmask;
  spiwrite(c);
  *csport |= cspinmask;
}


void TFT_ILI9341::writedata(uint8_t c)
{
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  spiwrite(c);
  *csport |= cspinmask;
}

void TFT_ILI9341::writeEnd() {
#ifdef F_AS_T
  TFT_CS_H;
#else
  *csport |= cspinmask;
#endif
}

void TFT_ILI9341::backupSPCR() {
  savedSPCR = SPCR;
  SPCR = mySPCR;
}

void TFT_ILI9341::restoreSPCR() {
 SPCR = savedSPCR;
}


// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.

#ifdef SPI_HAS_TRANSACTION
  #ifdef SUPPORT_TRANSACTIONS

static inline void spi_begin(void) __attribute__((always_inline));

static inline void spi_begin(void) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

static inline void spi_end(void) __attribute__((always_inline));

static inline void spi_end(void) {
  SPI.endTransaction();
}
  #else // we do not want to SUPPORT_TRANSACTIONS

#define spi_begin()
#define spi_end()

  #endif // SUPPORT_TRANSACTIONS

#else
#define spi_begin()
#define spi_end()
#endif


void TFT_ILI9341::init(void)
{
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  SPI.begin();

  savedSPCR = SPCR;
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  mySPCR = SPCR;

  spi_end();

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }


	// Initialization commands for ILI9341 screens
	static const uint8_t ILI9341_cmds[] PROGMEM =
	{
		#if (TFT_RST > 0)
		  21,
		#else
		  22,
		  ILI9341_SWRESET, ILI9341_INIT_DELAY,       // 1
		  5,
		#endif
		0xEF, 3,                        // 2
		0x03, 0x80, 0x02,
		0xCF, 3,                        // 3
		0x00, 0xC1, 0x30,
		0xED, 4,                        // 4
		0x64, 0x03, 0x12, 0x81,
		0xE8, 3,                        // 5
		0x85, 0x00, 0x78,
		0xCB, 5,                        // 6
		0x39, 0x2C, 0x00, 0x34, 0x02,
		0xF7, 1,                        // 7
		0x20,
		0xEA, 2,                        // 8
		0x00, 0x00,
		ILI9341_PWCTR1, 1,              // 9 power control 
		0x23,                           // VRH[5:0] 
		ILI9341_PWCTR2, 1,              // 10 power control 
		0x10,                           // SAP[2:0];BT[3:0]  
		ILI9341_VMCTR1, 2,              // 11 VCM control 
		0x3e, 0x28,
		ILI9341_VMCTR2, 1,              // 12 VCM control2 
		0x86,                           // --
		ILI9341_MADCTL, 1,              // 13
                (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR),
                ILI9341_PIXFMT, 1,              // 14
                0x55,
                ILI9341_FRMCTR1, 2,             // 15
                0x00, 0x18,
                ILI9341_DFUNCTR, 3,             // 16
                0x08, 0x82, 0x27,
                0xF2, 1,                        // 17 3Gamma Function Disable 
                0x00,
                ILI9341_GAMMASET, 1,            // 18 Gamma curve selected 
                0x01,
		ILI9341_GMCTRP1, 15,            // 19 Set Gamma 
		0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
		ILI9341_GMCTRN1, 15,            // 20
		0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
		ILI9341_SLPOUT, ILI9341_INIT_DELAY,          // 21
		120,
		ILI9341_DISPON, 0,              // 22
	};

	commandList(ILI9341_cmds);
}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void TFT_ILI9341::commandList (const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint8_t  ms;

	spi_begin();
	numCommands = pgm_read_byte(addr++);            // Number of commands to follow
	while (numCommands--)                           // For each command...
	{
		writecommand(pgm_read_byte(addr++));    // Read, issue command
		numArgs = pgm_read_byte(addr++);        // Number of args to follow
		ms = numArgs & ILI9341_INIT_DELAY;      // If hibit set, delay follows args
		numArgs &= ~ILI9341_INIT_DELAY;         // Mask out delay bit
		while (numArgs--)                       // For each argument...
		{
			writedata(pgm_read_byte(addr++)); // Read, issue argument
		}

		if (ms)
		{
			ms = pgm_read_byte(addr++);     // Read post-command delay time (ms)
			delay( (ms==255 ? 500 : ms) );
		}
	}
	spi_end();
}

// Draw a circle outline
void TFT_ILI9341::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = - r - r;
  int16_t x = 0;

  drawPixel(x0  , y0 + r, color);
  drawPixel(x0  , y0 - r, color);
  drawPixel(x0 + r, y0  , color);
  drawPixel(x0 - r, y0  , color);

  while (x < r) {
    if (f >= 0) {
      r--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + r, color);
    drawPixel(x0 - x, y0 + r, color);
    drawPixel(x0 + x, y0 - r, color);
    drawPixel(x0 - x, y0 - r, color);
    drawPixel(x0 + r, y0 + x, color);
    drawPixel(x0 - r, y0 + x, color);
    drawPixel(x0 + r, y0 - x, color);
    drawPixel(x0 - r, y0 - x, color);
  }
}

void TFT_ILI9341::drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;

  while (x < r) {
    if (f >= 0) {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + r, color);
      drawPixel(x0 + r, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - r, color);
      drawPixel(x0 + r, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - r, y0 + x, color);
      drawPixel(x0 - x, y0 + r, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - r, y0 - x, color);
      drawPixel(x0 - x, y0 - r, color);
    }
  }
}

void TFT_ILI9341::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  drawFastVLine(x0, y0 - r, r + r + 1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void TFT_ILI9341::fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -r - r;
  int16_t x     = 0;

  delta++;
  while (x < r) {
    if (f >= 0) {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0 + x, y0 - r, r + r + delta, color);
      drawFastVLine(x0 + r, y0 - x, x + x + delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0 - x, y0 - r, r + r + delta, color);
      drawFastVLine(x0 - r, y0 - x, x + x + delta, color);
    }
  }
}


void TFT_ILI9341::fillScreen(uint16_t color)
{
  fillRect(0, 0, _width, _height, color);
}

// Draw a rectangle
void TFT_ILI9341::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y + h - 1, w, color);
  drawFastVLine(x, y, h, color);
  drawFastVLine(x + w - 1, y, h, color);
}

// Draw a rounded rectangle
void TFT_ILI9341::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  drawFastHLine(x + r  , y    , w - r - r, color); // Top
  drawFastHLine(x + r  , y + h - 1, w - r - r, color); // Bottom
  drawFastVLine(x    , y + r  , h - r - r, color); // Left
  drawFastVLine(x + w - 1, y + r  , h - r - r, color); // Right
  // draw four corners
  drawCircleHelper(x + r    , y + r    , r, 1, color);
  drawCircleHelper(x + w - r - 1, y + r    , r, 2, color);
  drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  drawCircleHelper(x + r    , y + h - r - 1, r, 8, color);
}

// Fill a rounded rectangle
void TFT_ILI9341::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  fillRect(x + r, y, w - r - r, h, color);

  // draw four corners
  fillCircleHelper(x + w - r - 1, y + r, r, 1, h - r - r - 1, color);
  fillCircleHelper(x + r    , y + r, r, 2, h - r - r - 1, color);
}

// Draw a triangle
void TFT_ILI9341::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}


// Fill a triangle - original Adafruit function works well and code footprint is small
void TFT_ILI9341::fillTriangle ( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)      a = x1;
    else if (x1 > b) b = x1;
    if (x2 < a)      a = x2;
    else if (x2 > b) b = x2;
    drawFastHLine(a, y0, b - a + 1, color);
    return;
  }

  int16_t
  dx01 = x1 - x0,
  dy01 = y1 - y0,
  dx02 = x2 - x0,
  dy02 = y2 - y0,
  dx12 = x2 - x1,
  dy12 = y2 - y1,
  sa   = 0,
  sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (y1 == y2) last = y1;  // Include y1 scanline
  else         last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b) swap(a, b);
    drawFastHLine(a, y, b - a + 1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b) swap(a, b);
    drawFastHLine(a, y, b - a + 1, color);
  }
}

void TFT_ILI9341::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        drawPixel(x + i, y + j, color);
      }
    }
  }
}


void TFT_ILI9341::setCursor(int16_t x, int16_t y)
{
  cursor_x = x;
  cursor_y = y;
}

void TFT_ILI9341::setCursor(int16_t x, int16_t y, uint8_t font)
{
  textfont = font;
  cursor_x = x;
  cursor_y = y;
}

void TFT_ILI9341::setTextSize(uint8_t s)
{
  if (s>7) s = 7; // Limit the maximum size multiplier so byte variables can be used for rendering
  textsize = (s > 0) ? s : 1; // Don't allow font size 0
}

void TFT_ILI9341::setTextFont(uint8_t f)
{
  textfont = (f > 0) ? f : 1; // Don't allow font 0
}

void TFT_ILI9341::setTextColor(uint16_t c)
{
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void TFT_ILI9341::setTextColor(uint16_t c, uint16_t b)
{
  textcolor   = c;
  textbgcolor = b;
}

void TFT_ILI9341::setTextWrap(boolean w)
{
  textwrap = w;
}

void TFT_ILI9341::setTextDatum(uint8_t d)
{
  textdatum = d;
}

void TFT_ILI9341::setTextPadding(uint16_t x_width)
{
  padX = x_width;
}


uint8_t TFT_ILI9341::getRotation(void)
{
  return rotation;
}


// Return the size of the display (per current rotation)
int16_t TFT_ILI9341::width(void)
{
  return _width;
}

int16_t TFT_ILI9341::height(void)
{
  return _height;
}


// Draw a character - only used for the original Adafruit font to retain backwards compatibility
void TFT_ILI9341::drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
#ifdef LOAD_GLCD
  if ((x >= _width)            || // Clip right
      (y >= _height)           || // Clip bottom
      ((x + 6 * size - 1) < 0) || // Clip left
      ((y + 8 * size - 1) < 0))   // Clip top
    return;
  boolean fillbg = (bg != color);

spi_begin();

#ifdef FAST_GLCD
// This is about 5 times faster for textsize=1 with background (at 210us per character)
// but it is not really worth the extra 168 bytes needed...
  if ((size==1) && fillbg)
  {
    byte column[6];
    byte mask = 0x1;
    setAddrWindow(x, y, x+5, y+8);
    for (int8_t i = 0; i < 5; i++ ) column[i] = pgm_read_byte(font + (c * 5) + i);
    column[5] = 0;

    for (int8_t j = 0; j < 8; j++) {
      for (int8_t k = 0; k < 5; k++ ) {
        if (column[k] & mask) {
          while (!(SPSR & _BV(SPIF)));
          SPDR = color >> 8; asm volatile( "nop\n\t" ::); // Sync to SPIF bit
          while (!(SPSR & _BV(SPIF)));
          SPDR = color;
        }
        else {
          while (!(SPSR & _BV(SPIF)));
          SPDR = bg >> 8; asm volatile( "nop\n\t" ::);
          while (!(SPSR & _BV(SPIF)));
          SPDR = bg;
        }
      }

      mask <<= 1;
      while (!(SPSR & _BV(SPIF)));
      SPDR = bg >> 8; while (!(SPSR & _BV(SPIF)));
      SPDR = bg;
    }
    while (!(SPSR & _BV(SPIF)));
  }
  else
#endif // FAST_GLCD

  {
    for (int8_t i = 0; i < 6; i++ ) {
      uint8_t line;
      if (i == 5)
        line = 0x0;
      else
        line = pgm_read_byte(font + (c * 5) + i);

      if (size == 1) // default size
      {
        for (int8_t j = 0; j < 8; j++) {
          if (line & 0x1) drawPixel(x + i, y + j, color);
        #ifndef FAST_GLCD
          else if (fillbg) drawPixel(x + i, y + j, bg); // Comment out this line if using fast code above
        #endif
          line >>= 1;
        }
      }
      else {  // big size
        for (int8_t j = 0; j < 8; j++) {
          if (line & 0x1) fillRect(x + (i * size), y + (j * size), size, size, color);
          else if (fillbg) fillRect(x + i * size, y + j * size, size, size, bg);
          line >>= 1;
        }
      }
    }
  }
spi_end();

#endif // LOAD_GLCD
}

void TFT_ILI9341::setWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  spi_begin();
  setAddrWindow(x0, y0, x1, y1);
  TFT_CS_H;
  while (!(SPSR & _BV(SPIF)));
  spi_end();
}

void TFT_ILI9341::setAddrWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
spi_begin();
#ifdef F_AS_T
  // Column addr set
  TFT_DC_C;
  TFT_CS_L;
  SPDR = ILI9341_CASET;
  spiWait15();

  TFT_DC_D;
  SPDR = x0 >> 8; spiWait17();
  SPDR = x0; spiWait17();
  SPDR = x1 >> 8; spiWait17();
  SPDR = x1; spiWait14();

  // Row addr set
  TFT_DC_C;
  SPDR = ILI9341_PASET; spiWait15();

  TFT_DC_D;
  SPDR = y0 >> 8; spiWait17();
  SPDR = y0; spiWait17();
  SPDR = y1 >> 8; spiWait17();
  SPDR = y1; spiWait14();

  // write to RAM
  TFT_DC_C;
  SPDR = ILI9341_RAMWR; spiWait14();

  //CS, HIGH;
  //TFT_CS_H;
  TFT_DC_D;
#else // F_AS_T not defined
  writecommand(ILI9341_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y0 >> 8);
  writedata(y0);     // YSTART
  writedata(y1 >> 8);
  writedata(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
#endif // F_AS_T
spi_end();
}

void TFT_ILI9341::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  // Faster range checking, possible because x and y are unsigned
  if ((x >= _width) || (y >= _height)) return;
  spi_begin();
#ifdef F_AS_T
  // Column addr set
  TFT_DC_C;
  TFT_CS_L;
  SPDR = ILI9341_CASET;
  spiWait15();

  TFT_DC_D;
  SPDR = x >> 8; spiWait17();
  SPDR = x; spiWait15(); x++; 
  SPDR = x >> 8; spiWait17();
  SPDR = x; spiWait14();

  // Row addr set
  TFT_DC_C;
  //TFT_CS_L;
  SPDR = ILI9341_PASET; spiWait15();

  TFT_DC_D;
  SPDR = y >> 8; spiWait17();
  SPDR = y; spiWait15(); y++; 
  SPDR = y >> 8; spiWait17();
  SPDR = y; spiWait14();

  // write to RAM
  TFT_DC_C;
  SPDR = ILI9341_RAMWR; spiWait15();

  TFT_DC_D;

  SPDR = color >> 8; spiWait17();
  SPDR = color; spiWait14();

  //CS, HIGH;
  TFT_CS_H;
  //TFT_DC_D;
#else // F_AS_T
  setAddrWindow(x, y, x + 1, y + 1);

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
#endif // F_AS_T
  spi_end();
}

void TFT_ILI9341::pushColor(uint16_t color)
{
  spi_begin();

#ifdef F_AS_T
  TFT_CS_L;
#else
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#endif
  //uint8_t backupSPCR =SPCR;
  //SPCR = mySPCR;

  SPDR = color;
  while (!(SPSR & _BV(SPIF)));
  SPDR = color>>8;
  while (!(SPSR & _BV(SPIF)));

  //SPCR = backupSPCR;

#ifdef F_AS_T
  TFT_CS_H;
#else
  *csport |= cspinmask;
#endif

  spi_end();
}

// This sends the "color" "len" times
void TFT_ILI9341::pushColor(uint16_t color, uint16_t len)
{
  spi_begin();

#ifdef F_AS_T
  TFT_CS_L;
  spiWrite16(color, len);
  TFT_CS_H;
  while (!(SPSR & _BV(SPIF)));
#else
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;

  uint8_t backupSPCR =SPCR;
  SPCR = mySPCR;

  while (len) {
    len--;
    // This order is fast as we loop back during the SPI wait period
    while (!(SPSR & _BV(SPIF)));
    SPDR = color>>8;
    while (!(SPSR & _BV(SPIF)));
    SPDR = color;
  }
  while (!(SPSR & _BV(SPIF)));
  SPCR = backupSPCR;
  *csport |= cspinmask;
#endif

  spi_end();
}

// Sends an array of 16-bit color values to the TFT; used
// externally by BMP examples.  Assumes that setWindow() has
// previously been called to define the bounds.  Max 255 pixels at
// a time (BMP examples read in small chunks due to limited RAM).
void TFT_ILI9341::pushColors(uint16_t *data, uint8_t len)
{
  uint16_t color;
  spi_begin();

#ifdef F_AS_T
  TFT_CS_L;
#else
  *csport &= ~cspinmask;
#endif

  while (len--) {
    color = *data++;
    // This order is fast as we loop back & fetch during the SPI wait period!
    while (!(SPSR & _BV(SPIF)));
    SPDR = color >> 8;
    asm volatile( "nop\n\t" ::); // Sync bit check
    while (!(SPSR & _BV(SPIF)));
    SPDR = color;
  }
  while (!(SPSR & _BV(SPIF)));

#ifdef F_AS_T
  TFT_CS_H;
#else
  *csport |= cspinmask;
#endif

  spi_end();
}

// This is the byte array version for 16 bit raw images.
void TFT_ILI9341::pushColors(uint8_t *data, uint16_t len)
{
  spi_begin();
  len = len<<1;

#ifdef F_AS_T
  TFT_CS_L;
  while (len--) {
    SPDR = *data++;
    // Wait 11 clock cycles
    asm volatile
    (
      "	adiw	r24,0  \n"	// 2
      "	rcall	1f     \n"	// 9
      "	rjmp 	2f     \n"	// 11
      "1:	ret    \n"	//
      "2:	       \n"	//
    );
  }
  TFT_CS_H;
#else
  *csport &= ~cspinmask;
  while (len--) {
    asm volatile( "nop\n\t" ::); // Sync bit check
    while (!(SPSR & _BV(SPIF)));
    SPDR = *data++;
  }
  *csport |= cspinmask;
#endif

  while (!(SPSR & _BV(SPIF)));
  spi_end();
}

// Bresenham's algorithm - thx wikipedia - speed enhanced by Bodmer this uses
// the eficient FastH/V Line draw routine for segments of 2 pixels or more
// enhanced further using code from Xark and Spellbuilder (116 byte penalty)

// Select which version
#ifdef FAST_LINE

void TFT_ILI9341::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  spi_begin();

  int8_t steep = abs(y1 - y0) > abs(x1 - x0);
  int16_t xmax = _width, ymax = _height;

	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
           ymax = _width;
           xmax = _height;
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	if (x1 < 0) return;

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int8_t ystep = (y0 < y1) ? 1 : (-1);

	if (x1 >= xmax) x1 = xmax - 1;

	for (; x0 <= x1; x0++) {
		if ((x0 >= 0) && (y0 >= 0) && (y0 < ymax)) break;
		err -= dy;
		if (err < 0) {
			err += dx;
			y0 += ystep;
		}
	}

	if (x0 > x1) return;

	if (steep)	// y increments every iteration (y0 is x-axis, and x0 is y-axis)
	{
           setAddrWindow(y0, x0, y0, xmax);
		for (; x0 <= x1; x0++) {
			spiWrite16s(color);
			err -= dy;
			if (err < 0) {
				y0 += ystep;
				if ((y0 < 0) || (y0 >= ymax)) break;
				err += dx;
			     //while (!(SPSR & _BV(SPIF))); // Safe, but can comment out and rely on delay
                     setAddrWindow(y0, x0+1, y0, xmax);
			}
		}
	}
	else	// x increments every iteration (x0 is x-axis, and y0 is y-axis)
	{
           setAddrWindow(x0, y0, xmax, y0);
		for (; x0 <= x1; x0++) {
			spiWrite16s(color);
			err -= dy;
			if (err < 0) {
				y0 += ystep;
				if ((y0 < 0) || (y0 >= ymax)) break;
				err += dx;
			     //while (!(SPSR & _BV(SPIF))); // Safe, but can comment out and rely on delay
                     setAddrWindow(x0+1, y0, xmax, y0);
			}
		}
	}
      TFT_CS_H;
  spi_end();
}

#else // FAST_LINE not defined so use more compact version

// Slower but more compact line drawing function
void TFT_ILI9341::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  boolean steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx = x1 - x0, dy = abs(y1 - y0);;


  int16_t err = dx >> 1, ystep = -1, xs = x0, dlen = 0;
  if (y0 < y1) ystep = 1;

  // Split into steep and not steep for FastH/V separation
  if (steep) {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) drawPixel(y0, xs, color);
        else drawFastVLine(y0, xs, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) drawFastVLine(y0, xs, dlen, color);
  }
  else
  {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) drawPixel(xs, y0, color);
        else drawFastHLine(xs, y0, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) drawFastHLine(xs, y0, dlen, color);
  }
}

#endif // FAST_LINE option


void TFT_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
#ifdef CLIP_CHECK
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height)) return;
  if ((y + h - 1) >= _height) h = _height - y;
#endif

  spi_begin();

  setAddrWindow(x, y, x, y + h - 1);

#ifdef F_AS_T
  spiWrite16(color, h);
  TFT_CS_H;
#else

  uint8_t hi = color >> 8, lo = color;
  while (h) {
    h--;
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
#endif

  spi_end();
}


void TFT_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
#ifdef CLIP_CHECK
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height)) return;
  if ((x + w - 1) >= _width)  w = _width - x;
#endif

  spi_begin();
  setAddrWindow(x, y, x + w - 1, y);

#ifdef F_AS_T
  spiWrite16(color, w);
  TFT_CS_H;
#else
  uint8_t hi = color >> 8, lo = color;
  while (w) {
    w--;
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
#endif
  spi_end();
}

// fill a rectangle
void TFT_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
#ifdef CLIP_CHECK
  // rudimentary clipping (drawChar w/big text requires this)
  if ((x > _width) || (y > _height) || (w==0) || (h==0)) return;
  if ((x + w - 1) > _width)  w = _width  - x;
  if ((y + h - 1) > _height) h = _height - y;
#endif

  spi_begin();
  setAddrWindow(x, y, x + w - 1, y + h - 1);

#ifdef F_AS_T
  while (h--) spiWrite16(color, w);
  TFT_CS_H;
#else
  uint8_t hi = color >> 8, lo = color;

  while(h--) {
    while (w--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  *csport |= cspinmask;
#endif

  spi_end();
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t TFT_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void TFT_ILI9341::setRotation(uint8_t m)
{
  byte spsr = SPSR;// We need this here for some reason...
  rotation = m % 8;
  spi_begin();
  writecommand(ILI9341_MADCTL);
  switch (rotation) {
    case 0:
      writedata(ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      writedata(ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 2:
      writedata(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      writedata(ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
  // These next rotations are for bottum up BMP drawing
    case 4:
      writedata(ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 5:
      writedata(ILI9341_MADCTL_MV | ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 6:
      writedata(ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 7:
      writedata(ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;

  }
  spi_end();
}


void TFT_ILI9341::invertDisplay(boolean i)
{
  spi_begin();
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  spi_end();
}

size_t TFT_ILI9341::write(uint8_t uniCode)
{
  if (uniCode == '\r') return 1;
  unsigned int width = 0;
  unsigned int height = 0;
  //Serial.print((char) uniCode); // Debug line sends all printed TFT text to serial port

#ifdef LOAD_FONT2
  if (textfont == 2)
  {
      // This is 20us faster than using the fontdata structure (0.443ms per character instead of 0.465ms)
      width = pgm_read_byte(widtbl_f16 + uniCode-32);
      height = chr_hgt_f16;
      // Font 2 is rendered in whole byte widths so we must allow for this
      width = (width + 7) / 8;  // Width in whole bytes for font 2
      width = width * 8;        // Width converted back to pixles
  }
  #ifdef LOAD_RLE
  else
  #endif
#endif


#ifdef LOAD_RLE
  {
      // Uses the fontinfo struct array to avoid lots of 'if' or 'switch' statements
      // A tad slower than above but this is not significant and is more convenient for the RLE fonts
      // Yes, this code can be needlessly executed when textfont == 1...
      width = pgm_read_byte( pgm_read_word( &(fontdata[textfont].widthtbl ) ) + uniCode-32 );
      height= pgm_read_byte( &fontdata[textfont].height );
  }
#endif

#ifdef LOAD_GLCD
  if (textfont==1)
  {
      width =  6;
      height = 8;
  }
#endif

  height = height * textsize;

  if (uniCode == '\n') {
    cursor_y += height;
    cursor_x  = 0;
  }
  else
  {
    if (textwrap && (cursor_x + width * textsize >= _width))
    {
      cursor_y += height;
      cursor_x = 0;
    }
    cursor_x += drawChar(uniCode, cursor_x, cursor_y, textfont);
  }
  return 1;
}

/***************************************************************************************
** Function name:           drawChar
** Description:             draw a unicode onto the screen
***************************************************************************************/
int TFT_ILI9341::drawChar(unsigned int uniCode, int x, int y, int font)
{

#ifdef LOAD_GLCD
  if (font==1)
  {
      drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
      return 6 * textsize;
  }
#endif

  unsigned int width  = 0;
  unsigned int height = 0;
  unsigned int flash_address = 0; // 16 bit address OK for Arduino if font files <60K
  uniCode -= 32;

#ifdef LOAD_FONT2
  if (font == 2)
  {
      // This is 20us faster than using the fontdata structure (0.447ms per character instead of 0.467ms)
      flash_address = pgm_read_word(&chrtbl_f16[uniCode]);
      width = pgm_read_byte(widtbl_f16 + uniCode);
      height = chr_hgt_f16;
  }
  #ifdef LOAD_RLE
  else
  #endif
#endif

#ifdef LOAD_RLE
  {
      // This is slower than above but is more convenient for the RLE fonts
      flash_address = pgm_read_word( pgm_read_word( &(fontdata[font].chartbl ) ) + uniCode*sizeof(void *) );
      width = pgm_read_byte( pgm_read_word( &(fontdata[font].widthtbl ) ) + uniCode );
      height= pgm_read_byte( &fontdata[font].height );
  }
#endif

  int w = width;
  int pX      = 0;
  int pY      = y;
  byte line = 0;

//#ifndef F_AS_T
  byte tl = textcolor;
  byte th = textcolor >> 8;
  byte bl = textbgcolor;
  byte bh = textbgcolor >> 8;
//#endif

#ifdef LOAD_FONT2 // chop out 962 bytes of code if we do not need it
  if (font == 2) {
    w = w + 6; // Should be + 7 but we need to compensate for width increment
    w = w / 8;
    if (x + width * textsize >= _width) return width * textsize ;

    if (textcolor == textbgcolor || textsize != 1) {

      for (int i = 0; i < height; i++)
      {
        if (textcolor != textbgcolor) fillRect(x, pY, width * textsize, textsize, textbgcolor);

        for (int k = 0; k < w; k++)
        {
          line = pgm_read_byte(flash_address + w * i + k);
          if (line) {
            if (textsize == 1) {
              pX = x + k * 8;
              if (line & 0x80) drawPixel(pX, pY, textcolor);
              if (line & 0x40) drawPixel(pX + 1, pY, textcolor);
              if (line & 0x20) drawPixel(pX + 2, pY, textcolor);
              if (line & 0x10) drawPixel(pX + 3, pY, textcolor);
              if (line & 0x08) drawPixel(pX + 4, pY, textcolor);
              if (line & 0x04) drawPixel(pX + 5, pY, textcolor);
              if (line & 0x02) drawPixel(pX + 6, pY, textcolor);
              if (line & 0x01) drawPixel(pX + 7, pY, textcolor);
            }
            else {
              pX = x + k * 8 * textsize;
              if (line & 0x80) fillRect(pX, pY, textsize, textsize, textcolor);
              if (line & 0x40) fillRect(pX + textsize, pY, textsize, textsize, textcolor);
              if (line & 0x20) fillRect(pX + 2 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x10) fillRect(pX + 3 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x08) fillRect(pX + 4 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x04) fillRect(pX + 5 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x02) fillRect(pX + 6 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x01) fillRect(pX + 7 * textsize, pY, textsize, textsize, textcolor);
            }
          }
        }
        pY += textsize;
      }
    }
    else
      // Faster drawing of characters and background using block write
    {
      spi_begin();
      setAddrWindow(x, y, (x + w * 8) - 1, y + height - 1);

      byte mask;
      for (int i = 0; i < height; i++)
      {
        for (int k = 0; k < w; k++)
        {
          line = pgm_read_byte(flash_address + w * i + k);
          pX = x + k * 8;
          mask = 0x80;
          while (mask) {
            if (line & mask) {
              while (!(SPSR & _BV(SPIF)));
              SPDR = th; asm volatile( "nop\n\t" ::);
              while (!(SPSR & _BV(SPIF)));
              SPDR = tl;
            }
            else {
              while (!(SPSR & _BV(SPIF)));
              SPDR = bh; asm volatile( "nop\n\t" ::);
              while (!(SPSR & _BV(SPIF)));
              SPDR = bl;
            }
            mask = mask >> 1;
          }
        }
        pY += textsize;
      }
      while (!(SPSR & _BV(SPIF)));
      writeEnd();
      spi_end();
    }
  }

  #ifdef LOAD_RLE
  else
  #endif
#endif  //FONT2

#ifdef LOAD_RLE  //674 bytes of code
  // Font is not 2 and hence is RLE encoded
  {
    spi_begin();
    SPDR = 0; // Dummy write to ensure SPIF flag gets set for first check in while() loop
    w *= height; // Now w is total number of pixels in the character
    if ((textsize != 1) || (textcolor == textbgcolor)) {
      if (textcolor != textbgcolor) fillRect(x, pY, width * textsize, textsize * height, textbgcolor);
      int px = 0, py = pY, tpy = pY; // To hold character block start and end column and row values
      int pc = 0; // Pixel count
      byte np = textsize * textsize; // Number of pixels in a drawn pixel

      byte tnp = 0; // Temporary copy of np for while loop
      byte ts = textsize - 1; // Temporary copy of textsize
      // 16 bit pixel count so maximum font size is equivalent to 180x180 pixels in area
      // w is total number of pixels to plot to fill character block
      while (pc < w)
      {
        line = pgm_read_byte(flash_address);
        flash_address++;
        if (line & 0x80) {
          line &= 0x7F;
          line++;
          if (ts) {
            px = x + textsize * (pc % width); // Keep these px and py calculations outside the loop as they are slow
            py = y + textsize * (pc / width);
          }
          else {
            px = x + pc % width; // Keep these px and py calculations outside the loop as they are slow
            py = y + pc / width;
          }
          while (line--) { // In this case the while(line--) is faster
            pc++; // This is faster than putting pc+=line before while() as we use up SPI wait time
            while (!(SPSR & _BV(SPIF)));
            setAddrWindow(px, py, px + ts, py + ts);

            if (ts) {
              tnp = np;
              while (tnp--) {
                while (!(SPSR & _BV(SPIF)));
                SPDR = th;
                while (!(SPSR & _BV(SPIF)));
                SPDR = tl;
              }
            }
            else {
              while (!(SPSR & _BV(SPIF)));
              SPDR = th;
              while (!(SPSR & _BV(SPIF)));
              SPDR = tl;
            }
            px += textsize;

            if (px >= (x + width * textsize))
            {
              px = x;
              py += textsize;
            }
          }
        }
        else {
          line++;
          pc += line;
        }
      }
      while (!(SPSR & _BV(SPIF)));
      writeEnd();
      spi_end();
    }
    else // Text colour != background && textsize = 1
         // so use faster drawing of characters and background using block write
    {
      spi_begin();
      setAddrWindow(x, y, x + width - 1, y + height - 1);

      // Maximum font size is equivalent to 180x180 pixels in area
      while (w > 0)
      {
        line = pgm_read_byte(flash_address++);
        if (line & 0x80) {
          line &= 0x7F;
          line++; w -= line;
        #ifdef F_AS_T
          spiWrite16(textcolor, line);
        #else
          while (line--) {
            while (!(SPSR & _BV(SPIF)));
            SPDR = th;
            while (!(SPSR & _BV(SPIF)));
            SPDR = tl;
          }
        #endif
        }
        else {
          line++; w -= line;
        #ifdef F_AS_T
          spiWrite16(textbgcolor, line);
        #else
          while (line--) {
            while (!(SPSR & _BV(SPIF)));
            SPDR = bh;
            while (!(SPSR & _BV(SPIF)));
            SPDR = bl;
          }
        #endif
        }
      }
      while (!(SPSR & _BV(SPIF)));
      writeEnd();
      spi_end();
    }
  }
  // End of RLE font rendering
#endif
  return width * textsize;    // x +
}

/***************************************************************************************
** Function name:           drawString
** Description :            draw string
***************************************************************************************/
int TFT_ILI9341::drawString(char *string, int poX, int poY, int font)
{
  int16_t sumX = 0;
  uint8_t padding = 1;
  unsigned int cheight = 0;

  if (textdatum || padX)
  {
    char *pointer = string;
    unsigned int cwidth  = 0;
    char uniCode;
    char *widthtable;

    if (font>1 && font<9)
    widthtable = (char *)pgm_read_word( &(fontdata[font].widthtbl ) );

    while (*pointer)
    {
      uniCode = *pointer++;
#ifdef LOAD_GLCD
      if (font == 1) cwidth += 6;
      else
#endif
      cwidth += pgm_read_byte( widthtable + uniCode - 32);
    }

#ifdef LOAD_GLCD
    if (font == 1) cheight = 8 * textsize;
    else
#endif
    cheight = pgm_read_byte( &fontdata[font].height ) * textsize;

    cwidth  = cwidth  * textsize;

    switch(textdatum) {
      case TC_DATUM:
        poX -= cwidth/2;
        padding = 2;
        break;
      case TR_DATUM:
        poX -= cwidth;
        padding = 3;
        break;
      case ML_DATUM:
        poY -= cheight/2;
        padding = 1;
        break;
      case MC_DATUM:
        poX -= cwidth/2;
        poY -= cheight/2;
        padding = 2;
        break;
      case MR_DATUM:
        poX -= cwidth;
        poY -= cheight/2;
        padding = 3;
        break;
      case BL_DATUM:
        poY -= cheight;
        padding = 1;
        break;
      case BC_DATUM:
        poX -= cwidth/2;
        poY -= cheight;
        padding = 2;
        break;
      case BR_DATUM:
        poX -= cwidth;
        poY -= cheight;
        padding = 3;
        break;
    }
    if (poX < 0) poX = 0;
    if (poX+cwidth>_width)   poX = _width - cwidth;
    if (poY < 0) poY = 0;
    if (poY+cheight>_height) poY = _height - cheight;
  }

  while (*string) sumX += drawChar(*string++, poX+sumX, poY, font);

//#define PADDING_DEBUG

#ifndef PADDING_DEBUG
  if((padX>sumX) && (textcolor!=textbgcolor))
  {
    int padXc = poX+sumX; // Maximum left side padding
    switch(padding) {
      case 1:
        fillRect(padXc,poY,padX-sumX,cheight, textbgcolor);
        break;
      case 2:
        fillRect(padXc,poY,(padX-sumX)>>1,cheight, textbgcolor);
        padXc = (padX-sumX)>>1;
        if (padXc>poX) padXc = poX;
        fillRect(poX - padXc,poY,(padX-sumX)>>1,cheight, textbgcolor);
        break;
      case 3:
        if (padXc>padX) padXc = padX;
        fillRect(poX + sumX - padXc,poY,padXc-sumX,cheight, textbgcolor);
        break;
    }
  }
#else

  // This is debug code to show text (green box) and blanked (white box) areas
  // to show that the padding areas are being correctly sized and positioned
  if((padX>sumX) && (textcolor!=textbgcolor))
  {
    int padXc = poX+sumX; // Maximum left side padding
    drawRect(poX,poY,sumX,cheight, TFT_GREEN);
    switch(padding) {
      case 1:
        drawRect(padXc,poY,padX-sumX,cheight, TFT_WHITE);
        break;
      case 2:
        drawRect(padXc,poY,(padX-sumX)>>1, cheight, TFT_WHITE);
        padXc = (padX-sumX)>>1;
        if (padXc>poX) padXc = poX;
        drawRect(poX - padXc,poY,(padX-sumX)>>1,cheight, TFT_WHITE);
        break;
      case 3:
        if (padXc>padX) padXc = padX;
        drawRect(poX + sumX - padXc,poY,padXc-sumX,cheight, TFT_WHITE);
        break;
    }
  }
#endif

return sumX;
}

/***************************************************************************************
** Function name:           drawCentreString
** Descriptions:            draw string centred on dX
***************************************************************************************/
int TFT_ILI9341::drawCentreString(char *string, int dX, int poY, int font)
{
  byte tempdatum = textdatum;
  int sumX = 0;
  textdatum = TC_DATUM;
  sumX = drawString(string, dX, poY, font);
  textdatum = tempdatum;
  return sumX;
}

/***************************************************************************************
** Function name:           drawRightString
** Descriptions:            draw string right justified to dX
***************************************************************************************/
int TFT_ILI9341::drawRightString(char *string, int dX, int poY, int font)
{
  byte tempdatum = textdatum;
  int sumX = 0;
  textdatum = TR_DATUM;
  sumX = drawString(string, dX, poY, font);
  textdatum = tempdatum;
  return sumX;
}

/***************************************************************************************
** Function name:           drawNumber
** Description:             draw a long integer
***************************************************************************************/
int TFT_ILI9341::drawNumber(long long_num, int poX, int poY, int font)
{
  char str[12];
  ltoa(long_num, str, 10);
  return drawString(str, poX, poY, font);
}

/***************************************************************************************
** Function name:           drawFloat
** Descriptions:            drawFloat, prints 7 non zero digits maximum
***************************************************************************************/
// Adapted to assemble and print a string, this permits alignment relative to a datum
// looks complicated but much more compact and actually faster than using print class
int TFT_ILI9341::drawFloat(float floatNumber, int decimal, int poX, int poY, int font)
{
  char str[12] = "...";       // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  float rounding = 0.5;

  if (decimal >7) decimal = 7; // Limit the size of decimal

  // Adjust the rounding value
  for (unsigned char i = 0; i < decimal; ++i) rounding /= 10.0;

  if (floatNumber < -rounding)    // add sign, avoid - sign on 0.0
  {
    str[ptr]='-'; // Negative number
    ptr++;        // Increment pointer
    str[ptr] = 0; // Put a null in the array
    digits=0;     // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding;

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  if (temp <= 10000000) // No more than 7 digits
  {
    // Put integer part into array
    ltoa(temp, str + ptr, 10);

    // Find out where the null is to get the digit count loaded
    while((byte)str[ptr]!=0) ptr++; // Move the pointer along
    digits+=ptr;                    // Count the digits

    str[ptr] = '.'; // Add decimal point
    ptr++;
    str[ptr] = '0'; //Add a dummy zero in case integer value has 8 digits
    str[ptr+1] = 0; //Add a null but don't increment pointer so zero can be overwritten

    // Get the decimal portion
    floatNumber = floatNumber - temp;
    byte i = 0;

    // Get decimal digits one by one and put in array, limit total digits to 8
    while ((i < decimal) && (digits < 9))
    {
      i++;
      floatNumber *= 10;       // for the next decimal
      temp = floatNumber;      // get the decimal
      ltoa(temp, str + ptr, 10);
      ptr++; digits++;         // Increment pointer and digits count
      floatNumber -= temp;     // Remove that digit
    }
  }
  // Now we can plot the string and return pixel length
  return drawString(str, poX, poY, font);
}

/***************************************************************************************
** Function name:           spiWrite16
** Descriptions:            Delay based assembler loop for fast SPI write
***************************************************************************************/
inline void TFT_ILI9341::spiWrite16(uint16_t data, int16_t count)
{
// We can enter this loop with 0 pixels to draw, so we need to check this
// if(count<1) { Serial.print("#### Less than 1 ####"); Serial.println(count);}

  uint8_t temp;
  asm volatile
  (
    "	sbiw	%[count],0\n"			// test count
    //"	brmi	2f\n"					// if < 0 then done
    "	breq	2f\n"					// if == 0 then done

    "1:	out	%[spi],%[hi]\n"		// write SPI data
    "	rcall	3f      \n" // 7
    "	rcall	3f      \n" // 14
    "	rjmp 	4f      \n" // 16
    "3:	ret     \n" // 
    "4:	nop     \n" // 17

    "	out	%[spi],%[lo]\n"			// write SPI data

    "	adiw	r24,0	  \n"	// 2
    "	adiw	r24,0  \n"	// 4
    "	rcall	5f     \n"	// 11
    "	rjmp 	6f     \n"	// 13
    "5:	ret    \n"	// 
    "6:	       \n"

    "	sbiw	%[count],1 \n" // 15 decrement count
    "	brne	1b         \n" // 17 if != 0 then loop

    "2:\n"

    : [temp] "=d" (temp), [count] "+w" (count)
    : [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
    :
  );
}

/***************************************************************************************
** Function name:           spiWrite16s
** Descriptions:            Write 16 bits, do not wait after last byte sent
***************************************************************************************/
inline void TFT_ILI9341::spiWrite16s(uint16_t data)
{
  uint8_t temp;
  asm volatile
  (
    "out	%[spi],%[hi]\n"		// write SPI data
    "	rcall	3f      \n" // 7
    "	rcall	3f      \n" // 14
    "	rjmp 	4f      \n" // 16
    "3:	ret     \n" // 
    "4:	nop     \n" // 17

    "	out	%[spi],%[lo]\n"			// write SPI data

    "5:\n"
    : [temp] "=d" (temp)
    : [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
    :
  );
}


/***************************************************************************************
** Function name:           spiWrite16R with hi<>lo reversed (not used)
** Descriptions:            Delay based assembler loop for fast SPI write
***************************************************************************************/
inline void TFT_ILI9341::spiWrite16R(uint16_t data, int16_t count)
{
// We can enter this loop with 0 pixels to draw, so we need to check this
// if(count<1) { Serial.print("#### Less than 1 ####"); Serial.println(count);}

  uint8_t temp;
  asm volatile
  (
    "sbiw %[count],0   \n"	// test count
    //"brmi 2f         \n"	// if < 0 then done, we use unsigned though
    " breq	2f          \n" // if == 0 then done

    "1:	out	%[spi],%[hi]\n"		// write SPI data

    " rcall	3f     \n" // 7
    " rcall	3f     \n" // 14
    " rjmp 	4f     \n" // 16
    "3:	ret         \n"	// 
    "4:	nop         \n"	// 17

    " out	%[spi],%[lo]\n"			// write SPI data

    " adiw  r24,0	  \n"	// 2
    " adiw  r24,0      \n"	// 4
    " rcall 5f         \n"	// 11
    " rjmp  6f         \n"	// 13
    "5:  ret           \n"	// 
    "6:                \n"

    "	sbiw	%[count],1  \n" // 15 decrement count
    "	brne	1b          \n"	// 17 if != 0 then loop

    "2:\n"

    : [temp] "=d" (temp), [count] "+w" (count)
    : [spi] "i" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)(data>>8)), [hi] "r" ((uint8_t)data)
    :
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            17 cycle delay
***************************************************************************************/
inline void TFT_ILI9341::spiWait17(void)
{
  asm volatile
  (
    "	rcall	1f    \n" // 7
    "	rcall	1f    \n" // 14
    "	rjmp 	2f    \n" // 16
    "1:	ret   \n" // 
    "2:	nop	 \n" // 17
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            15 cycle delay
***************************************************************************************/
inline void TFT_ILI9341::spiWait15(void)
{
  asm volatile
  (
    "	adiw	r24,0  \n"	// 2
    "	adiw	r24,0  \n"	// 4
    "	adiw	r24,0  \n"	// 6
    "	rcall	1f     \n"	// 13
    "	rjmp 	2f     \n"	// 15
    "1:	ret    \n"	//
    "2:	       \n"	//
  );
}

/***************************************************************************************
** Function name:           spiWait
** Descriptions:            14 cycle delay
***************************************************************************************/
inline void TFT_ILI9341::spiWait14(void)
{
  asm volatile
  (
    "	nop         \n"	// 1
    "	adiw	r24,0  \n"	// 3
    "	adiw	r24,0  \n"	// 5
    "	rcall	1f     \n"	// 12
    "	rjmp 	2f     \n"	// 14
    "1:	ret    \n"	//
    "2:	       \n"	//
  );
}

/***************************************************

  ORIGINAL LIBRARY HEADER

  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

 ****************************************************/