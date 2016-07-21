// This sketch tests a function to draw elliptical (or circular) arcs
// of a defined width

#include <TFT_ILI9341.h> // Hardware-specific library
#include <SPI.h>

TFT_ILI9341 tft = TFT_ILI9341();       // Invoke custom library

#define DEG2RAD 0.0174532925

unsigned long td = 0;

void setup(void) {
  Serial.begin(115200);

  delay(4000);
    
  tft.begin();

  tft.setRotation(1);

  tft.fillScreen(ILI9341_BLACK);
}


void loop() {

  int w  = 12;
  int rx = 160;
  int ry = 100;
  td = millis();
 
  for (int n = 0; n < 5; n++) {
    fillArc(160, 100, 300, 20, rx-n*w, ry-n*w, w, 31-n*6);
  }

  Serial.println(millis()-td);
  while(1);
}

// #########################################################################
// Draw a circular or elliptical arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// ry = y axis radius
// w  = width (thickness) of arc in pixels
// colour = 16 bit colour value
// Note if rx and ry are the same then an arc of a circle is drawn

int fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{

  byte seg = 6; // Segments are 3 degrees wide = 120 segments for 360 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring

    // Calculate first pair of coordinates for segment start
    float sx = cos((start_angle - 90) * DEG2RAD);
    float sy = sin((start_angle - 90) * DEG2RAD);
    uint16_t x0 = sx * (rx - w) + x;
    uint16_t y0 = sy * (ry - w) + y;
    uint16_t x1 = sx * rx + x;
    uint16_t y1 = sy * ry + y;

  // Draw colour blocks every inc degrees
  for (int i = start_angle; i < start_angle + seg * seg_count; i += inc) {

    // Calculate pair of coordinates for segment end
    float sx = cos((i + seg - 90) * DEG2RAD);
    float sy = sin((i + seg - 90) * DEG2RAD);
    uint16_t x2 = sx * (rx - w) + x;
    uint16_t y2 = sy * (ry - w) + y;
    uint16_t x3 = sx * rx + x;
    uint16_t y3 = sy * ry + y;

    tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);

    // Copy segment end to sgement start for next segment
    x0 = x2;
    y0 = y2;
    x1 = x3;
    y1 = y3;
  }
}


