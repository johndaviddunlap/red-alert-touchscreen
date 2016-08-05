// BMP-loading example specifically for the TFTLCD breakout board.
// If using the Arduino shield, use the tftbmp_shield.pde sketch instead!
// If using an Arduino Mega make sure to use its hardware SPI pins, OR make
// sure the SD library is configured for 'soft' SPI in the file Sd2Card.h.

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>
//#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

#define MINPRESSURE 10
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Comment this out, if you don't want debugging output in the serial console
//#define DEBUG

// Comment this out if you don't need REL4 bitmaps
//#define BMP_REL4

// Comment this out if you don't need 24 bit bitmaps
#define BMP_24

// Comment this out if you don't need the inline test image
//#define BMP_INLINE_TEST

// These are the sizes of the BMP headers that we might encounter, in bytes.
#define BITMAPCOREHEADER 12
#define OS21XBITMAPHEADER 12
#define OS22XBITMAPHEADER 64
#define BITMAPINFOHEADER 40
#define BITMAPV2INFOHEADER 52
#define BITMAPV3INFOHEADER 56
#define BITMAPV4HEADER 108
#define BITMAPV5HEADER 124

// Define possible compression methods that we might encounter.
#define BI_RGB 0
#define BI_RLE8 1
#define BI_RLE4 2
#define BI_BITFIELDS 3
#define BI_JPEG 4
#define BI_PNG 5
#define BI_ALPHABITFIELDS 6
#define BI_CMYK 11
#define BI_CMYKRLE8 12
#define BI_CMYKRLE4 13

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// For Arduino Uno/Duemilanove, etc
//  connect the SD card with DI going to pin 11, DO going to pin 12 and SCK going to pin 13 (standard)
//  Then pin 10 goes to CS (or whatever you have set up)
#define SD_CS 10     // Set the chip select line to whatever you use (10 doesnt conflict with the library)

// In the SD card, place 24 bit color BMP files (be sure they are 24-bit!)
// There are examples in the sketch folder

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define BOXSIZE 40

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.
#define BUFFPIXEL 20

// our TFT wiring
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, A4);

#define LCARS_PURPLE 0xCCD9
#define LCARS_ORANGE 0xFCC0
#define LCARS_RED 0xCB2C

#define BUTTON_LEFT_EDGE 24
#define BUTTON_RIGHT_EDGE 216
#define ALARM_TOP_EDGE 259
#define ALARM_BOTTOM_EDGE 295
#define LOGS_TOP_EDGE 24
#define LOGS_BOTTOM_EDGE 61
#define SOUND_TOP_EDGE 71
#define SOUND_BOTTOM_EDGE 107
#define NETWORK_TOP_EDGE 118
#define NETWORK_BOTTOM_EDGE 155
#define ACL_TOP_EDGE 165
#define ACL_BOTTOM_EDGE 202

#define STATE_MENU 0
#define STATE_ALARM 1
#define STATE_BUTTON_DOWN 2
#define STATE_BUTTON_UP 3
#define STATE_WAITING 4
#define STATE_LOGS 5
#define STATE_SOUND 6
#define STATE_NETWORK 7
#define STATE_ACL 8

// The current state of the touchscreen's state machine
int currentState = STATE_WAITING;

// Track the previous state
int lastObservedState = currentState;

TSPoint pressed;

// Track the number of ticks since the last state change
unsigned int ticksInButtonUp = 0;
unsigned int ticksInButtonDown = 0;

void setup()
{
  Serial.begin(9600);
  
  // Start I2C bus
  //Wire.begin();

  tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    #ifdef DEBUG
    Serial.println(F("Found ILI9325 LCD driver"));
    #endif
  } else if(identifier == 0x9328) {
    #ifdef DEBUG
    Serial.println(F("Found ILI9328 LCD driver"));
    #endif
  } else if(identifier == 0x7575) {
    #ifdef DEBUG
    Serial.println(F("Found HX8347G LCD driver"));
    #endif
  } else if(identifier == 0x9341) {
    #ifdef DEBUG
    Serial.println(F("Found ILI9341 LCD driver"));
    #endif
  } else if(identifier == 0x8357) {
    #ifdef DEBUG
    Serial.println(F("Found HX8357D LCD driver"));
    #endif
  } else {
#ifdef DEBUG
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
#endif
    // Default to the identifier of the touchscreen(it fails to auto-detect for some reason)
    identifier=0x9341;
  }

  tft.begin(identifier);
/*
  #ifdef DEBUG
  Serial.print(F("Initializing SD card..."));
  #endif
  if (!SD.begin(SD_CS)) {
    #ifdef DEBUG
    Serial.println(F("failed!"));
    #endif
    return;
  }
  
  #ifdef DEBUG
  Serial.println(F("OK!"));
  #endif
*/
  //bmpDrawRLE4("fed_rle4.bmp", 0, 0);
  //bmp24Draw("keypad.bmp",0,0);
  
  drawKeypad();
}

void drawKeypad() {
  tft.fillScreen(BLACK);
  tft.fillRect(40, 24, 91, 36, LCARS_PURPLE);
  drawPurpleUpperLeftCorner(23, 24);
  drawPurpleLowerLeftCorner(23,42);
}

void drawPurpleUpperLeftCorner(unsigned int x, unsigned int y) {
tft.drawFastHLine(12+x,0+y,1,0x51EA);tft.drawFastHLine(13+x,0+y,1,0x72AE);tft.drawFastHLine(14+x,0+y,1,0x9BB3);tft.drawFastHLine(15+x,0+y,2,0xB436);tft.drawFastHLine(10+x,1+y,1,0x72AE);tft.drawFastHLine(11+x,1+y,1,0xB436);tft.drawFastHLine(12+x,1+y,5,0xCCD9);tft.drawFastHLine(8+x,2+y,1,0x51EA);tft.drawFastHLine(9+x,2+y,1,0xB436);tft.drawFastHLine(10+x,2+y,7,0xCCD9);tft.drawFastHLine(7+x,3+y,1,0x72AE);tft.drawFastHLine(8+x,3+y,9,0xCCD9);tft.drawFastHLine(6+x,4+y,1,0xB436);tft.drawFastHLine(7+x,4+y,10,0xCCD9);tft.drawFastHLine(5+x,5+y,1,0xB436);tft.drawFastHLine(6+x,5+y,11,0xCCD9);tft.drawFastHLine(4+x,6+y,1,0xB436);tft.drawFastHLine(5+x,6+y,12,0xCCD9);tft.drawFastHLine(3+x,7+y,1,0x9BB3);tft.drawFastHLine(4+x,7+y,13,0xCCD9);tft.drawFastHLine(2+x,8+y,1,0x51EA);tft.drawFastHLine(3+x,8+y,14,0xCCD9);tft.drawFastHLine(2+x,9+y,1,0xB436);tft.drawFastHLine(3+x,9+y,14,0xCCD9);tft.drawFastHLine(1+x,10+y,1,0x72AE);tft.drawFastHLine(2+x,10+y,15,0xCCD9);tft.drawFastHLine(1+x,11+y,1,0xB436);tft.drawFastHLine(2+x,11+y,15,0xCCD9);tft.drawFastHLine(17+x,11+y,0,0x51EA);tft.drawFastHLine(0+x,12+y,1,0x51EA);tft.drawFastHLine(1+x,12+y,16,0xCCD9);tft.drawFastHLine(17+x,12+y,0,0x72AE);tft.drawFastHLine(0+x,13+y,1,0x72AE);tft.drawFastHLine(1+x,13+y,16,0xCCD9);tft.drawFastHLine(17+x,13+y,0,0x9BB3);tft.drawFastHLine(0+x,14+y,1,0x9BB3);tft.drawFastHLine(1+x,14+y,16,0xCCD9);tft.drawFastHLine(17+x,14+y,0,0xB436);tft.drawFastHLine(0+x,15+y,1,0xB436);
}

void drawPurpleLowerLeftCorner(unsigned int x, unsigned int y) {
tft.drawFastHLine(0+x,0+y,18,0xCCD9);tft.drawFastHLine(0+x,1+y,18,0xCCD9);tft.drawFastHLine(0+x,2+y,15,0xCCD9);tft.drawFastHLine(15+x,2+y,1,0xB436);tft.drawFastHLine(16+x,2+y,2,0xCCD9);tft.drawFastHLine(0+x,3+y,14,0xCCD9);tft.drawFastHLine(14+x,3+y,1,0x9BB3);tft.drawFastHLine(15+x,3+y,3,0xCCD9);tft.drawFastHLine(0+x,4+y,13,0xCCD9);tft.drawFastHLine(13+x,4+y,1,0x72AE);tft.drawFastHLine(14+x,4+y,4,0xCCD9);tft.drawFastHLine(0+x,5+y,12,0xCCD9);tft.drawFastHLine(12+x,5+y,1,0x51EA);tft.drawFastHLine(13+x,5+y,5,0xCCD9);tft.drawFastHLine(0+x,6+y,11,0xCCD9);tft.drawFastHLine(12+x,6+y,1,0xB436);tft.drawFastHLine(13+x,6+y,5,0xCCD9);tft.drawFastHLine(0+x,7+y,10,0xCCD9);tft.drawFastHLine(11+x,7+y,1,0x72AE);tft.drawFastHLine(12+x,7+y,6,0xCCD9);tft.drawFastHLine(0+x,8+y,9,0xCCD9);tft.drawFastHLine(11+x,8+y,1,0xB436);tft.drawFastHLine(12+x,8+y,6,0xCCD9);tft.drawFastHLine(0+x,9+y,8,0xCCD9);tft.drawFastHLine(10+x,9+y,1,0x51EA);tft.drawFastHLine(11+x,9+y,7,0xCCD9);tft.drawFastHLine(0+x,10+y,7,0xCCD9);tft.drawFastHLine(10+x,10+y,1,0x9BB3);tft.drawFastHLine(11+x,10+y,7,0xCCD9);tft.drawFastHLine(0+x,11+y,6,0xCCD9);tft.drawFastHLine(10+x,11+y,1,0xB436);tft.drawFastHLine(11+x,11+y,7,0xCCD9);tft.drawFastHLine(0+x,12+y,5,0xCCD9);tft.drawFastHLine(10+x,12+y,1,0xB436);tft.drawFastHLine(11+x,12+y,7,0xCCD9);tft.drawFastHLine(0+x,13+y,4,0xCCD9);tft.drawFastHLine(10+x,13+y,1,0xB436);tft.drawFastHLine(11+x,13+y,7,0xCCD9);tft.drawFastHLine(0+x,14+y,3,0xCCD9);tft.drawFastHLine(10+x,14+y,1,0x72AE);tft.drawFastHLine(11+x,14+y,7,0xCCD9);tft.drawFastHLine(0+x,15+y,2,0xCCD9);tft.drawFastHLine(10+x,15+y,1,0x51EA);tft.drawFastHLine(11+x,15+y,1,0xB436);tft.drawFastHLine(12+x,15+y,6,0xCCD9);tft.drawFastHLine(0+x,16+y,1,0xCCD9);tft.drawFastHLine(11+x,16+y,1,0x72AE);tft.drawFastHLine(12+x,16+y,1,0xB436);tft.drawFastHLine(13+x,16+y,5,0xCCD9);tft.drawFastHLine(12+x,17+y,1,0x51EA);tft.drawFastHLine(13+x,17+y,1,0x72AE);tft.drawFastHLine(14+x,17+y,1,0x9BB3);
}

void loop()
{
  // Attempt to read a value from the touchscreen
  digitalWrite(13, HIGH);
  TSPoint tmp = ts.getPoint();
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  //pinMode(XP, OUTPUT);
  //pinMode(XM, OUTPUT);
  //pinMode(YP, OUTPUT);

  switch (currentState) {
    case STATE_WAITING:
      executeWaitingState(tmp);
      break;
    case STATE_BUTTON_DOWN:
      executeButtonDownState(tmp);
      break;
    case STATE_BUTTON_UP:
      executeButtonUpState(tmp);
      break;
    case STATE_MENU:
      executeMenuState(pressed);
      break; 
    case STATE_ALARM:
      executeAlarmState();
      break;
    case STATE_LOGS:
      executeLogsState();
      break;
    case STATE_SOUND:
      executeSoundState();
      break;
    case STATE_NETWORK:
      executeNetworkState();
      break;
    case STATE_ACL:
      executeAclState();
      break;
    default:
      Serial.print(F("Unknown state: "));
      Serial.println(currentState);
      break;
  }
  
  // Slow things down a little to try to prevent phantom touch events
  delay(1);
}

void executeWaitingState(TSPoint tmp) {
    // Scale from 0->1023 to tft.width, if the touchscreen has been touched
    if (isTouched(tmp)) {
      tmp.x = tft.width() - map(tmp.x, TS_MINX, TS_MAXX, tft.width(), 0);
      tmp.y = map(tmp.y, TS_MINY, TS_MAXY, tft.height(), 0);
      pressed = tmp;
      currentState = STATE_BUTTON_DOWN;
    }
}

void executeButtonUpState(TSPoint p) {
    // Go back to button down if it was only a momentary button up
    if (isTouched(p)) {
      currentState = STATE_BUTTON_DOWN;
      ticksInButtonUp = 0;
    }
    
    // Don't leave this state unless 5 ticks
    // have passed since the last observed state change
    if (ticksInButtonUp > 100) {
      currentState = STATE_MENU;
    } else {
      ticksInButtonUp++;
    }
}

void executeButtonDownState(TSPoint p) {
   if (!isTouched(p) && ticksInButtonDown > 100) {
      currentState = STATE_BUTTON_UP;
      ticksInButtonDown = 0;
    } else {
      ticksInButtonDown++;
    }
 }

void executeMenuState(TSPoint p) {
  #ifdef DEBUG
  Serial.print(F("X"));
  Serial.print(p.x);
  Serial.print(F(" Y"));
  Serial.println(p.y);
  #endif
  
  // Has the alarm button been pressed?
  if (
    p.x >= BUTTON_LEFT_EDGE && 
    p.x <= BUTTON_RIGHT_EDGE && 
    p.y >= ALARM_TOP_EDGE && 
    p.y <= ALARM_BOTTOM_EDGE
  ) {
    currentState = STATE_ALARM;
    return;
  } else if (
    p.x >= BUTTON_LEFT_EDGE && 
    p.x <= BUTTON_RIGHT_EDGE && 
    p.y >= LOGS_TOP_EDGE && 
    p.y <= LOGS_BOTTOM_EDGE
  ) {
    currentState = STATE_LOGS;
    return;
  } else if (
    p.x >= BUTTON_LEFT_EDGE && 
    p.x <= BUTTON_RIGHT_EDGE && 
    p.y >= SOUND_TOP_EDGE && 
    p.y <= SOUND_BOTTOM_EDGE
  ) {
    currentState = STATE_SOUND;
    return;
  } else if (
    p.x >= BUTTON_LEFT_EDGE && 
    p.x <= BUTTON_RIGHT_EDGE && 
    p.y >= NETWORK_TOP_EDGE && 
    p.y <= NETWORK_BOTTOM_EDGE
  ) {
    currentState = STATE_NETWORK;
    return;
  } else if (
    p.x >= BUTTON_LEFT_EDGE && 
    p.x <= BUTTON_RIGHT_EDGE && 
    p.y >= ACL_TOP_EDGE && 
    p.y <= ACL_BOTTOM_EDGE
  ) {
    currentState = STATE_ACL;
    return;
  } else {
    currentState = STATE_WAITING;
  }
}

void executeAlarmState() {
  Serial.println(F("Alarm button pressed"));
  
  // Return the state machine to the waiting state
  currentState = STATE_WAITING;
}

void executeLogsState() {
  Serial.println(F("Logs button pressed"));
  
  // Return the state machine to the waiting state
  currentState = STATE_WAITING;
}

void executeSoundState() {
  Serial.println(F("Sound button pressed"));
  
  // Return the state machine to the waiting state
  currentState = STATE_WAITING;
}

void executeNetworkState() {
  Serial.println(F("Network button pressed"));
  
  // Return the state machine to the waiting state
  currentState = STATE_WAITING;
}

void executeAclState() {
  Serial.println(F("ACL button pressed"));
  
  // Return the state machine to the waiting state
  currentState = STATE_WAITING;
}

boolean isTouched(TSPoint p) {
  return p.z > MINPRESSURE && p.z < MAXPRESSURE;
}
/*
void bmp24Draw(char *filename, int x, int y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpFileSize;
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t bmpHeaderSize;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;
  boolean  first = true;

  if((x >= tft.width()) || (y >= tft.height())) return;

  #ifdef DEBUG
  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
  #endif
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    #ifdef DEBUG
    Serial.println(F("File not found"));
    #endif
    return;
  }

  // Parse BMP header
  if(readLittleEndian16(bmpFile) == 0x4D42) { // BMP signature
    bmpFileSize = readLittleEndian32(bmpFile);
    #ifdef DEBUG
    Serial.println(F("File size: ")); Serial.println(bmpFileSize);
    #endif
    (void)readLittleEndian32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = readLittleEndian32(bmpFile); // Start of image data
    bmpHeaderSize = readLittleEndian32(bmpFile);
    #ifdef DEBUG
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(bmpHeaderSize);
    #endif
    bmpWidth  = readLittleEndian32(bmpFile);
    bmpHeight = readLittleEndian32(bmpFile);
    if(readLittleEndian16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = readLittleEndian16(bmpFile); // bits per pixel
      #ifdef DEBUG
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      #endif
      if((bmpDepth == 24) && (readLittleEndian32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        
        #ifdef DEBUG
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);
        #endif

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.pushColors(lcdbuffer, lcdidx, first);
                lcdidx = 0;
                first  = false;
              }
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = tft.color565(r,g,b);
          } // end pixel
        } // end scanline
        // Write any remaining data to LCD
        if(lcdidx > 0) {
          tft.pushColors(lcdbuffer, lcdidx, first);
        } 
        
        #ifdef DEBUG
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        #endif
      } // end goodBmp
    }
  }

  bmpFile.close();
  
  #ifdef DEBUG
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
  #endif
}
*/
// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
/*uint16_t readLittleEndian16(File f) {
  int counter;
  return readLittleEndian16(f, &counter);
}

uint16_t readLittleEndian16(File f, int* counter) {
  uint16_t result;
  ((uint8_t *)&result)[0] = readByte(f, counter); // LSB
  ((uint8_t *)&result)[1] = readByte(f, counter); // MSB
  return result;
}

uint32_t readLittleEndian32(File f) {
  int counter = 0;
  return readLittleEndian32(f, &counter);
}

uint32_t readLittleEndian32(File f, int* counter) {
  uint32_t result;
  ((uint8_t *)&result)[0] = readByte(f, counter); // LSB
  ((uint8_t *)&result)[1] = readByte(f, counter);
  ((uint8_t *)&result)[2] = readByte(f, counter);
  ((uint8_t *)&result)[3] = readByte(f, counter); // MSB

  return result;
}

uint32_t readBigEndian32(File f, int* counter) {
  uint32_t result;
  ((uint8_t *)&result)[3] = readByte(f, counter); // LSB
  ((uint8_t *)&result)[2] = readByte(f, counter);
  ((uint8_t *)&result)[1] = readByte(f, counter);
  ((uint8_t *)&result)[0] = readByte(f, counter); // MSB

  return result;
}

uint32_t readBigEndian32(File f) {
  int counter = 0;
  return readBigEndian32(f, &counter);
}

unsigned char readByte(File f) {
  int counter = 0;
  return readByte(f, &counter);
}

unsigned char readByte(File f, int* counter) {
  unsigned char c = f.read();
  counter[0]++;
  return c;
}
*/
#ifdef BMP_RLE4
void bmpDrawRLE4(char *filename, int x, int y) {
  return bmpDrawRLE4(filename, x, y, -1);
}


void bmpDrawRLE4(char *filename, int x, int y, int transparentColor) {
  File bmpFile;
  boolean goodBmp = false; // Set to true on valid header parse
  int readCounter = 0;

  // Needed BMP header fields
  int bmpDIBHeaderSize = -1;
  short bmpColorPlanes = -1;
  short bmpBitsPerPixel = -1;  
  int bmpCompression = -1;
  int bmpColorCount = -1;

  // Abort, if the requested coordinates are out of bounds
  if((x >= tft.width()) || (y >= tft.height())) return;

#ifdef DEBUG
  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
#endif

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found: ")); Serial.println(filename);
    return;
  }
  
  // Read the basic 14 byte header that all BMP's begin with
  if(readLittleEndian16(bmpFile, &readCounter) == 0x4D42) { // BMP signature
    readLittleEndian32(bmpFile, &readCounter); // Size of file
    readLittleEndian32(bmpFile, &readCounter); // Read & ignore creator bytes
    readLittleEndian32(bmpFile, &readCounter); // Start of image data

    // We should have read 14 bytes, at this point
    if (readCounter == 14) {
      // The size of the DIB header(including this value)
      bmpDIBHeaderSize = readLittleEndian32(bmpFile, &readCounter);

      readLittleEndian32(bmpFile, &readCounter); // Image width
      readLittleEndian32(bmpFile, &readCounter); // Image height
    
      // We expect the DIB header to be 40 bytes
      if (bmpDIBHeaderSize == BITMAPINFOHEADER) {
        if ((bmpColorPlanes = readLittleEndian16(bmpFile, &readCounter)) == 1) {
          if ((bmpBitsPerPixel = readLittleEndian16(bmpFile, &readCounter)) == 4) { // We expect 4 bit color depth
            #ifdef DEBUG
            Serial.println(F("Image has 4 bit color depth"));
            #endif
          
            if ((bmpCompression = readLittleEndian32(bmpFile, &readCounter)) == BI_RLE4) {
              readLittleEndian32(bmpFile, &readCounter); // Raw BMP size including padding

              readLittleEndian32(bmpFile, &readCounter); // pixels/meter horizontal
              readLittleEndian32(bmpFile, &readCounter); // pixels/meter vertical              
              bmpColorCount = readLittleEndian32(bmpFile, &readCounter); // Number of colors in palette
              readLittleEndian32(bmpFile, &readCounter); // Number of important colors

              if (readCounter == BITMAPINFOHEADER + 14) { // Should have read all 54 header bytes, at this point
                goodBmp = true;

                // Allocate 2 bytes for each color in the color table
                unsigned short colorPalette[bmpColorCount];
    
                #ifdef DEBUG            
                Serial.print(F("Building color table with "));
                Serial.print(bmpColorCount);
                Serial.println(F(" colors"));
                #endif
                
                // Build the color table
                for (unsigned char i = 0; i < bmpColorCount; i++) {
                  unsigned char blue = readByte(bmpFile, &readCounter);
                  unsigned char green = readByte(bmpFile, &readCounter);
                  unsigned char red = readByte(bmpFile, &readCounter);
                  readByte(bmpFile, &readCounter); // Always zero
                  colorPalette[i] = tft.color565(red, green, blue);

                  #ifdef DEBUG            
                  Serial.print(F(" * rgb["));
                  Serial.print(i, DEC);
                  Serial.print(F("](red="));
                  Serial.print(red, DEC);
                  Serial.print(F(", green="));
                  Serial.print(green, DEC);
                  Serial.print(F(", blue="));
                  Serial.print(blue, DEC);
                  Serial.println(F(")"));
                  #endif

                  //tft.fillScreen(colorPalette[i]);
                  //delay(1000);
                }
                                
                unsigned char firstByte = -1;
                unsigned char secondByte = -1;
                int x = 0, y = 0;
                
                // Start reading pixel data
                while(bmpFile.available()) {
                //while(readCounter < 0x1F1) {
                  firstByte = readByte(bmpFile, &readCounter);
                  secondByte = readByte(bmpFile, &readCounter);

                  // We're dealing with an escape
                  if (firstByte == 0) {
                    if (secondByte == 0) { // End of line
                      x = 0;
                      y++;
                    } else if (secondByte == 1) { // End of bitmap
                      break;
                    } else if (secondByte == 2) { // Delta (not supported)
                      #ifdef DEBUG
                      Serial.print(F("Delta escapes are not currently supported"));
                      Serial.print(F(" at offset ")); Serial.println(readCounter - 1, HEX);
                      #endif
                    }
                    
                    #ifdef DEBUG
                    else {
                      Serial.print(F("Unknown escape ")); Serial.print(secondByte);
                      Serial.print(F(" at offset ")); Serial.println(readCounter - 1, HEX);
                      
                      // We're in absolute mode now
                      int len = (secondByte / 2) + (secondByte) % 2;
                      
                    }
                    #endif
                  }
                  
                  // Otherwise, Use the first byte as a counter and the second byte as color information
                  else {
                    unsigned char higherNibble = secondByte & 0xF0;
                    unsigned char lowerNibble = secondByte & 0xF;
                    
                    // Attempt to draw a line
                    if (higherNibble >> 8 == lowerNibble) {
                      // If we have a transparent color, skip this bit if the color is our "transparent" color
                      if (colorPalette[higherNibble] == transparentColor || transparentColor == -1) {
                        #ifdef DEBUG
                        Serial.print(F("Drawing "));
                        Serial.print(firstByte, DEC);
                        Serial.print(F("px line at (x="));
                        Serial.print(x, DEC);
                        Serial.print(F(", y="));
                        Serial.print(y);
                        Serial.print(F(") "));
                        Serial.print(F(" with color index "));
                        Serial.println(higherNibble);
                        #endif
                        
                        if (higherNibble >= bmpColorCount) {
                          #ifdef DEBUG
                          Serial.print(F("ABORT: Color index "));
                          Serial.print(higherNibble, DEC);
                          Serial.print(F(" exceeds the bounds of the color index("));
                          Serial.print(bmpColorCount);
                          Serial.print(") at offset ");
                          Serial.println(readCounter - 1, HEX);
                          #endif
                          break;
                        }

                        tft.drawFastHLine(x, y, firstByte, colorPalette[higherNibble]);
                      }
                    } else {
                      // Alternate colors for the length of the run
                      for (int h = 0; h < firstByte; h++) {
                        if (firstByte % 2 == 0) {
                          tft.drawPixel(x,y, colorPalette[higherNibble]);
                        } else {
                          tft.drawPixel(x,y, colorPalette[lowerNibble]);
                        }
                      }
                    }

                    x = x + firstByte;
                  }
                }
              }
            #ifdef DEBUG
            else {
              Serial.print(F("Expected to have read 54 header bytes but found: ")); Serial.println(readCounter);
            }
            #endif
            }
            #ifdef DEBUG
            else {
              Serial.print(F("Expected RLE4(2) compression but found: ")); Serial.println(bmpCompression);
            }
            #endif
          }
          #ifdef DEBUG
          else {
            Serial.print(F("Expected 4 bit color depth but found: ")); Serial.println(bmpBitsPerPixel);
          }
          #endif
        }
        #ifdef DEBUG
        else {
          Serial.print(F("Expected 1 color plane but found: ")); Serial.println(bmpColorPlanes);
        }
        #endif
      }
      #ifdef DEBUG
      else {
        Serial.print(F("Unsupported DIB header size: ")); Serial.println(bmpDIBHeaderSize);
      }
      #endif
    }
    #ifdef DEBUG
    else {
        Serial.print(F("Expected 14 byte core header but found: ")); Serial.println(readCounter);
    }
    #endif
  }
  #ifdef DEBUG
  else {
      Serial.print(F("Could not find 0x4D42 BMP signature at the front of the file"));
  }
  #endif

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized.")); 
  else Serial.println("BMP drawn"); 
}
#endif

#ifdef BMP_INLINE_TEST
void graphicsTest() {
  int start;
  int duration;
  
  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(F("Inline C code: "));
  delay(1000);

  start = millis();
  drawImage();
  duration = millis() - start;
  tft.setCursor(0, 0);
  tft.print("Inline C code: ");
  tft.print(duration);
  tft.print("ms");

  Serial.print(F("Inline C code      - (millis="));
  Serial.print(millis());
  Serial.print(F(", start="));
  Serial.print(start);
  Serial.print(F(", duration="));
  Serial.print(duration);
  Serial.println(F(")"));

  delay(1000);

  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(F("SD-CARD BMP 24-bit: "));

  delay(1000);
  start = millis();
  bmp24Draw("lariat24.bmp", 0, 20);
  duration = millis() - start;
  tft.setCursor(0, 0);
  tft.print("SD-CARD BMP 24-bit: ");
  tft.print(duration);
  tft.print("ms");

  Serial.print(F("SD-CARD BMP 24-bit - (millis="));
  Serial.print(millis());
  Serial.print(F(", start="));
  Serial.print(start);
  Serial.print(F(", duration="));
  Serial.print(duration);
  Serial.println(F(")"));

  delay(1000);
}

void drawImage() {
	tft.drawFastHLine(123,80,1,0xDD45);tft.drawFastHLine(123,81,2,0xDD45);tft.drawFastHLine(122,82,3,0xDD45);tft.drawFastHLine(121,83,5,0xDD45);tft.drawFastHLine(121,84,5,0xDD45);tft.drawFastHLine(120,85,7,0xDD45);tft.drawFastHLine(119,86,9,0xDD45);tft.drawFastHLine(118,87,10,0xDD45);tft.drawFastHLine(118,88,11,0xDD45);tft.drawFastHLine(117,89,12,0xDD45);tft.drawFastHLine(116,90,14,0xDD45);tft.drawFastHLine(116,91,15,0xDD45);tft.drawFastHLine(115,92,16,0xDD45);tft.drawFastHLine(115,93,17,0xDD45);tft.drawFastHLine(114,94,18,0xDD45);tft.drawFastHLine(113,95,20,0xDD45);tft.drawFastHLine(113,96,20,0xDD45);tft.drawFastHLine(112,97,22,0xDD45);tft.drawFastHLine(111,98,23,0xDD45);tft.drawFastHLine(111,99,24,0xDD45);tft.drawFastHLine(110,100,25,0xDD45);tft.drawFastHLine(110,101,26,0xDD45);tft.drawFastHLine(109,102,27,0xDD45);tft.drawFastHLine(109,103,28,0xDD45);tft.drawFastHLine(108,104,29,0xDD45);tft.drawFastHLine(108,105,30,0xDD45);tft.drawFastHLine(107,106,31,0xDD45);tft.drawFastHLine(107,107,32,0xDD45);tft.drawFastHLine(106,108,33,0xDD45);tft.drawFastHLine(105,109,35,0xDD45);tft.drawFastHLine(105,110,35,0xDD45);tft.drawFastHLine(104,111,37,0xDD45);tft.drawFastHLine(104,112,37,0xDD45);tft.drawFastHLine(103,113,38,0xDD45);tft.drawFastHLine(103,114,39,0xDD45);tft.drawFastHLine(102,115,40,0xDD45);tft.drawFastHLine(102,116,41,0xDD45);tft.drawFastHLine(101,117,42,0xDD45);tft.drawFastHLine(101,118,43,0xDD45);tft.drawFastHLine(100,119,44,0xDD45);tft.drawFastHLine(100,120,44,0xDD45);tft.drawFastHLine(100,121,45,0xDD45);tft.drawFastHLine(99,122,46,0xDD45);tft.drawFastHLine(99,123,46,0xDD45);tft.drawFastHLine(98,124,48,0xDD45);tft.drawFastHLine(98,125,48,0xDD45);tft.drawFastHLine(97,126,50,0xDD45);tft.drawFastHLine(97,127,50,0xDD45);tft.drawFastHLine(96,128,51,0xDD45);tft.drawFastHLine(96,129,52,0xDD45);tft.drawFastHLine(96,130,52,0xDD45);tft.drawFastHLine(95,131,53,0xDD45);tft.drawFastHLine(95,132,54,0xDD45);tft.drawFastHLine(94,133,55,0xDD45);tft.drawFastHLine(94,134,55,0xDD45);tft.drawFastHLine(94,135,56,0xDD45);tft.drawFastHLine(93,136,57,0xDD45);tft.drawFastHLine(93,137,57,0xDD45);tft.drawFastHLine(93,138,58,0xDD45);tft.drawFastHLine(92,139,59,0xDD45);tft.drawFastHLine(92,140,59,0xDD45);tft.drawFastHLine(91,141,61,0xDD45);tft.drawFastHLine(91,142,61,0xDD45);tft.drawFastHLine(91,143,61,0xDD45);tft.drawFastHLine(90,144,63,0xDD45);tft.drawFastHLine(90,145,63,0xDD45);tft.drawFastHLine(90,146,63,0xDD45);tft.drawFastHLine(89,147,65,0xDD45);tft.drawFastHLine(89,148,65,0xDD45);tft.drawFastHLine(89,149,65,0xDD45);tft.drawFastHLine(88,150,66,0xDD45);tft.drawFastHLine(88,151,67,0xDD45);tft.drawFastHLine(88,152,67,0xDD45);tft.drawFastHLine(87,153,68,0xDD45);tft.drawFastHLine(87,154,68,0xDD45);tft.drawFastHLine(87,155,69,0xDD45);tft.drawFastHLine(86,156,70,0xDD45);tft.drawFastHLine(86,157,70,0xDD45);tft.drawFastHLine(86,158,71,0xDD45);tft.drawFastHLine(85,159,72,0xDD45);tft.drawFastHLine(85,160,72,0xDD45);tft.drawFastHLine(85,161,72,0xDD45);tft.drawFastHLine(85,162,73,0xDD45);tft.drawFastHLine(84,163,74,0xDD45);tft.drawFastHLine(84,164,74,0xDD45);tft.drawFastHLine(84,165,74,0xDD45);tft.drawFastHLine(83,166,76,0xDD45);tft.drawFastHLine(83,167,76,0xDD45);tft.drawFastHLine(83,168,76,0xDD45);tft.drawFastHLine(83,169,76,0xDD45);tft.drawFastHLine(82,170,77,0xDD45);tft.drawFastHLine(82,171,78,0xDD45);tft.drawFastHLine(82,172,78,0xDD45);tft.drawFastHLine(81,173,79,0xDD45);tft.drawFastHLine(81,174,79,0xDD45);tft.drawFastHLine(81,175,80,0xDD45);tft.drawFastHLine(81,176,80,0xDD45);tft.drawFastHLine(81,177,80,0xDD45);tft.drawFastHLine(80,178,81,0xDD45);tft.drawFastHLine(80,179,81,0xDD45);tft.drawFastHLine(80,180,82,0xDD45);tft.drawFastHLine(80,181,82,0xDD45);tft.drawFastHLine(79,182,55,0xDD45);tft.drawFastHLine(139,182,23,0xDD45);tft.drawFastHLine(79,183,52,0xDD45);tft.drawFastHLine(140,183,22,0xDD45);tft.drawFastHLine(79,184,49,0xDD45);tft.drawFastHLine(142,184,20,0xDD45);tft.drawFastHLine(79,185,47,0xDD45);tft.drawFastHLine(143,185,20,0xDD45);tft.drawFastHLine(78,186,46,0xDD45);tft.drawFastHLine(144,186,19,0xDD45);tft.drawFastHLine(78,187,44,0xDD45);tft.drawFastHLine(145,187,18,0xDD45);tft.drawFastHLine(78,188,43,0xDD45);tft.drawFastHLine(146,188,17,0xDD45);tft.drawFastHLine(78,189,41,0xDD45);tft.drawFastHLine(147,189,16,0xDD45);tft.drawFastHLine(78,190,40,0xDD45);tft.drawFastHLine(147,190,16,0xDD45);tft.drawFastHLine(77,191,40,0xDD45);tft.drawFastHLine(148,191,16,0xDD45);tft.drawFastHLine(77,192,38,0xDD45);tft.drawFastHLine(149,192,15,0xDD45);tft.drawFastHLine(77,193,37,0xDD45);tft.drawFastHLine(150,193,14,0xDD45);tft.drawFastHLine(77,194,36,0xDD45);tft.drawFastHLine(150,194,14,0xDD45);tft.drawFastHLine(77,195,34,0xDD45);tft.drawFastHLine(151,195,13,0xDD45);tft.drawFastHLine(76,196,34,0xDD45);tft.drawFastHLine(152,196,13,0xDD45);tft.drawFastHLine(76,197,33,0xDD45);tft.drawFastHLine(152,197,13,0xDD45);tft.drawFastHLine(76,198,32,0xDD45);tft.drawFastHLine(153,198,12,0xDD45);tft.drawFastHLine(76,199,31,0xDD45);tft.drawFastHLine(154,199,11,0xDD45);tft.drawFastHLine(76,200,30,0xDD45);tft.drawFastHLine(155,200,10,0xDD45);tft.drawFastHLine(75,201,30,0xDD45);tft.drawFastHLine(155,201,10,0xDD45);tft.drawFastHLine(75,202,29,0xDD45);tft.drawFastHLine(156,202,10,0xDD45);tft.drawFastHLine(75,203,28,0xDD45);tft.drawFastHLine(156,203,10,0xDD45);tft.drawFastHLine(75,204,27,0xDD45);tft.drawFastHLine(157,204,9,0xDD45);tft.drawFastHLine(75,205,26,0xDD45);tft.drawFastHLine(158,205,8,0xDD45);tft.drawFastHLine(75,206,25,0xDD45);tft.drawFastHLine(158,206,8,0xDD45);tft.drawFastHLine(74,207,25,0xDD45);tft.drawFastHLine(159,207,7,0xDD45);tft.drawFastHLine(74,208,24,0xDD45);tft.drawFastHLine(159,208,7,0xDD45);tft.drawFastHLine(74,209,23,0xDD45);tft.drawFastHLine(160,209,7,0xDD45);tft.drawFastHLine(74,210,22,0xDD45);tft.drawFastHLine(160,210,7,0xDD45);tft.drawFastHLine(74,211,21,0xDD45);tft.drawFastHLine(161,211,6,0xDD45);tft.drawFastHLine(74,212,20,0xDD45);tft.drawFastHLine(162,212,5,0xDD45);tft.drawFastHLine(74,213,19,0xDD45);tft.drawFastHLine(162,213,5,0xDD45);tft.drawFastHLine(73,214,19,0xDD45);tft.drawFastHLine(163,214,4,0xDD45);tft.drawFastHLine(73,215,18,0xDD45);tft.drawFastHLine(163,215,4,0xDD45);tft.drawFastHLine(73,216,17,0xDD45);tft.drawFastHLine(164,216,4,0xDD45);tft.drawFastHLine(73,217,16,0xDD45);tft.drawFastHLine(164,217,4,0xDD45);tft.drawFastHLine(73,218,15,0xDD45);tft.drawFastHLine(165,218,3,0xDD45);tft.drawFastHLine(73,219,15,0xDD45);tft.drawFastHLine(165,219,3,0xDD45);tft.drawFastHLine(73,220,14,0xDD45);tft.drawFastHLine(166,220,2,0xDD45);tft.drawFastHLine(72,221,14,0xDD45);tft.drawFastHLine(166,221,2,0xDD45);tft.drawFastHLine(72,222,13,0xDD45);tft.drawFastHLine(167,222,1,0xDD45);tft.drawFastHLine(72,223,12,0xDD45);tft.drawFastHLine(167,223,2,0xDD45);tft.drawFastHLine(72,224,11,0xDD45);tft.drawFastHLine(168,224,1,0xDD45);tft.drawFastHLine(72,225,10,0xDD45);tft.drawFastHLine(168,225,1,0xDD45);tft.drawFastHLine(72,226,10,0xDD45);tft.drawFastHLine(72,227,9,0xDD45);tft.drawFastHLine(72,228,8,0xDD45);tft.drawFastHLine(72,229,7,0xDD45);tft.drawFastHLine(71,230,7,0xDD45);tft.drawFastHLine(71,231,7,0xDD45);tft.drawFastHLine(71,232,6,0xDD45);tft.drawFastHLine(71,233,5,0xDD45);tft.drawFastHLine(71,234,4,0xDD45);tft.drawFastHLine(71,235,3,0xDD45);tft.drawFastHLine(71,236,3,0xDD45);tft.drawFastHLine(71,237,2,0xDD45);tft.drawFastHLine(71,238,1,0xDD45);
}
#endif

