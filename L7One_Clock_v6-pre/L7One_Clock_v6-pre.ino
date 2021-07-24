// Lazy 7 / One - Clock Sketch v6-pre - nodeMCU/ESP8266 (experimental) - 4D/6D combined
// https://www.thingiverse.com/thing:4363658
// https://www.instructables.com/id/Lazy-7-One/
// 06/2020 - Daniel Cikic
// Serial is set to 74880

// uncomment #define nodeMCU to compile this for nodeMCU v1.0 (ESP-12E) instead of Arduino Pro Mini/Nano
#define nodeMCU
// uncomment #define useWiFi to use WiFi/ntp, leave commented for nodeMCU + DS3231
// Also check other parameters like ntp server and utc time offset further down when using WiFi/ntp!
// #define useWiFi

// uncomment below to use 6 digits (HH:MM:SS), do NOT manually change LED_DIGITS for this!
// #define use6D

#ifdef nodeMCU
  #define FASTLED_ESP8266_RAW_PIN_ORDER           // this means we'll be using the raw esp8266 pin order -> GPIO_12, which is d6 on nodeMCU
  #define LED_PIN 0                              // led data in connected to GPIO_0 (d3/nodeMCU)
#else
  #define FASTLED_ALLOW_INTERRUPTS 0
  #define LED_PIN 6                               // led data in connected to d6 (arduino)
#endif

#define LED_PWR_LIMIT 750                         // 750mA - Power limit in mA (voltage is set in setup() to 5v)
#ifdef use6D
  #define LED_DIGITS 6                            // digit count (4 or 6, defined by use6D)
#else
  #define LED_DIGITS 4
#endif
#define LED_PER_DIGIT_STRIP 56                    // each digit is using 56 leds (7 segments / 8 leds each)
#define SPACING_LEDS 2                            // 2 unused leds between digits/center dots
#define LED_PER_CENTER_MODULE 12                  // 12 leds inside center module (6 upper dot/6 lower dot)
// Using all of the above we can calculate total led count:
#define LED_COUNT LED_DIGITS * LED_PER_DIGIT_STRIP + ( LED_DIGITS / 2 ) * ( SPACING_LEDS * 2 ) + ( LED_DIGITS / 3 ) * LED_PER_CENTER_MODULE + ( LED_DIGITS / 3 ) * ( SPACING_LEDS * 4 )

#include <FastLED.h>                              // these libraries will be included in all cases....
#include <TimeLib.h>
#include <EEPROM.h>

#ifdef nodeMCU
  #ifdef useWiFi                                  // include wifi library when set to use it, otherwise just load wire/rtc libs for nodeMCU
    #include <ESP8266WiFi.h>
    #include <NTPClient.h>
    #include <WiFiUdp.h>
    #define timeOffset 120                        // time offset to utc in minutes, 120 minutes -> CEST
    WiFiUDP ntpUDP;
    NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", timeOffset * 60, 60000);
  #else
    #include <Wire.h>
    #include <RtcDS3231.h>
    RtcDS3231<TwoWire> Rtc(Wire);                 // Define the Rtc object
  #endif
#else                                             // on Arduino we'll be using wire/DS3232RTC as in previous versions
  #include <Wire.h>
  #include <DS3232RTC.h>
#endif

CRGB leds[LED_COUNT];

CRGBPalette16 currentPalette;

const bool dbg = true;                            // debug, true = enable serial input/output - set to false to save memory

#ifdef nodeMCU
  int buttonA = 13;                               // momentary push button, 1 pin to gnd, 1 pin to d7 / GPIO_13
  int buttonB = 14;                               // momentary push button, 1 pin to gnd, 1 pin to d5 / GPIO_14
#else
  int buttonA = 3;                                // momentary push button, 1 pin to gnd, 1 pin to d3
  int buttonB = 4;                                // momentary push button, 1 pin to gnd, 1 pin to d4
#endif

byte brightness = 100;                            // default brightness if none saved to eeprom yet / first run
byte brightnessLevels[3] {100, 150, 230};         // 0 - 255, brightness Levels (min, med, max) - index (0-2) will get stored to eeprom
                                                  // Note: With brightnessAuto = 1 this will be the maximum brightness setting used!
byte brightnessAuto = 0;                          // 1 = enable brightness corrections using a photo resistor/readLDR();
byte upperLimitLDR = 140;                         // everything above this value will cause max brightness to be used (if it's higher than this)
byte lowerLimitLDR = 40;                          // everything below this value will cause minBrightness to be used
byte minBrightness = 20;                          // anything below this avgLDR value will be ignored
float factorLDR = 1.0;                            // try 0.5 - 2.0, compensation value for avgLDR. Set dbgLDR & dbg to true and watch serial console. Looking...
const bool dbgLDR = false;                        // ...for values in the range of 120-160 (medium room light), 40-80 (low light) and 0 - 20 in the dark
#ifdef nodeMCU
  int pinLDR = 0;                                 // LDR connected to A0 (nodeMCU only offers this one)
#else
  int pinLDR = 1;                                 // LDR connected to A1 (in case somebody flashes this sketch on arduino and already has an ldr connected to A1)
#endif
byte intervalLDR = 60;                            // read value from LDR every 60ms (most LDRs have a minimum of about 30ms - 50ms)
unsigned long valueLDRLastRead = 0;               // time when we did the last readout
int avgLDR = 0;                                   // we will average this value somehow somewhere in readLDR();
int lastAvgLDR = 0;

byte startColor = 0;                              // "index" for the palette color used for drawing
byte displayMode = 0;                             // 0 = 12h, 1 = 24h (will be saved to EEPROM once set using buttons)
byte colorOffset = 32;                            // default distance between colors on the color palette used between digits/leds (in overlayMode)
int colorChangeInterval = 1500;                   // interval (ms) to change colors when not in overlayMode (per pixel/led coloring uses overlayInterval)
byte overlayMode = 1;                             // switch on/off (1/0) to use void colorOverlay(); (will be saved to EEPROM once set using buttons)
int overlayInterval = 1500;                        // interval (ms) to change colors in overlayMode

byte btnRepeatCounter = 1;
byte lastKeyPressed = 0;
unsigned long btnRepeatStart = 0;

byte lastSecond = 0;
unsigned long lastLoop = 0;
unsigned long lastColorChange = 0;

/* these values will be stored to the EEPROM:
  0 = index for selectedPalette / switchPalette();
  1 = index for brightnessLevels / switchBrightness();
  2 = displayMode (when set using the buttons)
  3 = overlayMode (when set using the buttons)
*/

#ifdef use6D                                  // because of the way the first digit is built (first _and_ last led inside its segments) we need different "specs" for 4/6 digits
  const uint16_t segGroups[7][4] PROGMEM {    // Each segment has 1-x led(s). So lets assign them in a way we get the first digit (right one when seen from the front) completely.
    {   4,   7,  16,  19 },                   // top, a
    {   0,   3,  20,  23 },                   // top right, b
    { 368, 371, 372, 375 },                   // bottom right, c
    { 364, 367, 376, 379 },                   // bottom, d
    { 360, 363, 380, 383 },                   // bottom left, e
    {   8,  11,  12,  15 },                   // top left, f
    {  24,  27, 384, 387 },                   // center, g
  };
#else
  const uint16_t segGroups[7][4] PROGMEM = {  // Each segment has 1-x led(s). So lets assign them in a way we get the first digit (right one when seen from the front) completely.
    {   4,   7,  16,  19 },                   // top, a
    {   0,   3,  20,  23 },                   // top right, b
    { 232, 235, 236, 239 },                   // bottom right, c
    { 228, 231, 240, 243 },                   // bottom, d
    { 224, 227, 244, 247 },                   // bottom left, e
    {   8,  11,  12,  15 },                   // top left, f
    {  24,  27, 248, 251 },                   // center, g
  };
#endif
// All other digits/segments can be calculated based on this and the definitions on top of the sketch
// Note: The first number always has to be the lower one as they're subtracted later on... (fix by using abs()? ^^)

const byte digits[14][7] PROGMEM = {        // Lets define 10 numbers (0-9) with 7 segments each, 1 = segment is on, 0 = segment is off
  {   1,   1,   1,   1,   1,   1,   0 },    // 0 -> Show segments a - f, don't show g (center one)
  {   0,   1,   1,   0,   0,   0,   0 },    // 1 -> Show segments b + c (top and bottom right), nothing else
  {   1,   1,   0,   1,   1,   0,   1 },    // 2 -> and so on...
  {   1,   1,   1,   1,   0,   0,   1 },    // 3
  {   0,   1,   1,   0,   0,   1,   1 },    // 4
  {   1,   0,   1,   1,   0,   1,   1 },    // 5
  {   1,   0,   1,   1,   1,   1,   1 },    // 6
  {   1,   1,   1,   0,   0,   0,   0 },    // 7
  {   1,   1,   1,   1,   1,   1,   1 },    // 8
  {   1,   1,   1,   1,   0,   1,   1 },    // 9
  {   0,   0,   0,   1,   1,   1,   1 },    // t -> some letters from here on (index 10-13, so this won't interfere with using digits 0-9 by using index 0-9
  {   0,   0,   0,   0,   1,   0,   1 },    // r
  {   0,   1,   1,   1,   0,   1,   1 },    // y
  {   0,   1,   1,   1,   1,   0,   1 }     // d
};

void setup() {
  if (brightnessAuto == 1) pinMode(pinLDR, OUTPUT);
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  if (dbg) {
    Serial.begin(74880); Serial.println();    
    Serial.println(F("Lazy 7 / One starting up..."));
    Serial.print(F("Configured for: ")); Serial.print(LED_COUNT); Serial.println(F(" leds"));
    Serial.print(F("Power limited to (mA): ")); Serial.print(LED_PWR_LIMIT); Serial.println(F(" mA")); Serial.println();
  }
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT).setCorrection(TypicalSMD5050).setTemperature(DirectSunlight).setDither(1);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, LED_PWR_LIMIT);
  FastLED.clear();
  FastLED.show();
  #ifdef nodeMCU
    EEPROM.begin(512);
  #endif
  loadValuesFromEEPROM();
  switchPalette();
  switchBrightness();
  #ifdef nodeMCU                                                                              // if building for nodeMCU...
    #ifdef useWiFi                                                                            // ...and if using WiFi.....
      if ( dbg ) {
        Serial.println(F("Starting up WiFi..."));
      }
      byte i = 30;
      WiFi.mode(WIFI_STA);                                                                    // set WiFi mode to STA...
      WiFi.begin(WiFi.SSID().c_str(),WiFi.psk().c_str());                                     // ...and start connecting using saved credentials...
      while ( i > 0 ) {                                                                       // check for roughly 10 seconds (i = 30 * 333ms) and show a
        if ( WiFi.status() != WL_CONNECTED ) i--; else i = 0;                                 // countdown while waiting for WL_CONNECTED status
        showDigit(i / 3, startColor, 0);
        FastLED.show();
        FastLED.clear();
        delay(333);
      }
      if ( WiFi.status() == WL_CONNECTED ) {                                                  // if status is connected, initialize timeClient and sync to ntp
        timeClient.begin();
        syncNTP();
      }
      if ( dbg ) {
        if ( WiFi.status() != 0 ) {
          Serial.print(F("Connected to SSID: ")); Serial.println(WiFi.SSID());
        } else Serial.println(F("WiFi connection failed."));
      }
    #else                                                                                     // using nodeMCU but no WiFi, so do everything required for the RTC
      Wire.begin();
      Rtc.Begin();
      setTime(Rtc.GetDateTime());
    #endif
  #else                                                                                       // building for Arduino, so do everything as in previous sketches...
    Wire.begin();
    setSyncProvider(RTC.get);
    setSyncInterval(15);
  #endif
}

void loop() {
  if (  ( lastLoop - lastColorChange >= colorChangeInterval ) && ( overlayMode == 0 )         // if colorChangeInterval has been reached and overlayMode is disabled...
     || ( lastLoop - lastColorChange >= overlayInterval ) && ( overlayMode == 1 ) ) {         // ...or if overlayInterval has been reached and overlayMode is enabled...
    startColor++;                                                                             // increase startColor to "move" colors slowly across the digits/leds
    updateDisplay(startColor, colorOffset);
    lastColorChange = millis();
  }
  if ( lastSecond != second() ) {                                                             // if current second is different from last second drawn...
    updateDisplay(startColor, colorOffset);                                                   // lastSecond will be set in displayTime() and will be used for
    lastSecond = second();                                                                    // redrawing regardless the digits count (HH:MM or HH:MM:SS)
  }
  if ( lastKeyPressed == 1 ) {                                                                // if buttonA is pressed...
    switchBrightness();                                                                       // ...switch to next brightness level
    updateDisplay(startColor, colorOffset);
    if ( btnRepeatCounter >= 20 ) {                                                           // if buttonA is held for a few seconds change overlayMode 0/1 (using colorOverlay())
      if ( overlayMode == 0 ) overlayMode = 1; else overlayMode = 0;
      updateDisplay(startColor, colorOffset);
      EEPROM.put(3, overlayMode);                                                             // ...and write setting to eeprom
      #ifdef nodeMCU                                                                          // on nodeMCU we need to commit the changes from ram to flash to make them permanent
        EEPROM.commit();
      #endif
      btnRepeatStart = millis();
    }
  }
  if ( lastKeyPressed == 2 ) {                                                                // if buttonB is pressed...
    switchPalette();                                                                          // ...switch between color palettes
    updateDisplay(startColor, colorOffset);
    if ( btnRepeatCounter >= 20 ) {                                                           // if buttonB is held for a few seconds change displayMode 0/1 (12h/24h)...
      if ( displayMode == 0 ) displayMode = 1; else displayMode = 0;
      updateDisplay(startColor, colorOffset);
      EEPROM.put(2, displayMode);                                                             // ...and write setting to eeprom
      #ifdef nodeMCU
        EEPROM.commit();
      #endif
      btnRepeatStart = millis();
    }
  }
  if ( ( lastLoop - valueLDRLastRead >= intervalLDR ) && ( brightnessAuto == 1 ) ) {          // if LDR is enabled and sample interval has been reached...
    readLDR();                                                                                // ...call readLDR();
    if ( abs(avgLDR - lastAvgLDR) >= 5 ) {                                                    // only adjust current brightness if avgLDR has changed for more than +/- 5.
      updateDisplay(startColor, colorOffset);
      lastAvgLDR = avgLDR;
      if ( dbg ) { Serial.print(F("Updated display with avgLDR of: ")); Serial.println(avgLDR); }
    }
    valueLDRLastRead = millis();
  }
  if ( lastKeyPressed == 12 ) {                                                               // if buttonA + buttonB are pushed at the same time....
    #ifdef useWiFi                                                                            // ...and if using WiFi...
      initWPS();                                                                              // ...start WPS
    #else                                                                                     // otherwise (arduino + rtc or nodemcu + rtc)...
      setupClock();                                                                           // ...start manual setup
    #endif
  }
  #ifdef nodeMCU                                                                              // On Arduino SetSyncProvider will be used. So this will sync internal time to rtc/ntp on nodeMCU only
    if ( ( hour() == 3 || hour() == 9 || hour() == 15 || hour() == 21 ) &&                    // if hour is 3, 9, 15 or 21 and...
         ( minute() == 3 && second() == 0 ) ) {                                               // minute is 3 and second is 0....
      if ( dbg ) Serial.print(F("Current time: ")); Serial.println(now());
      #ifdef useWiFi
        syncNTP();                                                                            // ...either sync using ntp or...
      #else
        setTime(Rtc.GetDateTime());                                                           // ...set internal time to rtc time...
      #endif
      if ( dbg ) Serial.print(F("New time: ")); Serial.println(now());
    }
    ESP.wdtFeed();                                                                            // feed the watchdog each time loop() is cycled through, just in case...
  #endif
  FastLED.show();                                                                             // run FastLED.show() every time to avoid color flickering at low brightness settings
  lastKeyPressed = readButtons();
  lastLoop = millis();
  if ( dbg ) dbgInput();                                                                      // if dbg = true this will read serial input/keys
}

void readLDR() {                                                                                            // read LDR value 5 times and write average to avgLDR for use in updateDisplay();
  static byte runCounter = 1;
  static int tmp = 0;
  byte readOut = map(analogRead(pinLDR), 0, 1023, 0, 250);
  tmp += readOut;
  if (runCounter == 5) {
    avgLDR = (tmp / 5)  * factorLDR;
    tmp = 0; runCounter = 0;
    if ( dbg && dbgLDR ) { Serial.print(F("avgLDR value: ")); Serial.print(avgLDR); }
    avgLDR = max(avgLDR, int(minBrightness)); avgLDR = min(avgLDR, int(brightness));                        // this keeps avgLDR in a range between minBrightness and maximum current brightness
    if ( avgLDR >= upperLimitLDR && avgLDR < brightness ) avgLDR = brightness;                              // if avgLDR is above upperLimitLDR switch to max current brightness
    if ( avgLDR <= lowerLimitLDR ) avgLDR = minBrightness;                                                  // if avgLDR is below lowerLimitLDR switch to minBrightness
    if ( dbg && dbgLDR ) { Serial.print(F(" - adjusted to: ")); Serial.println(avgLDR); }
  }
  runCounter++;
}

void colorOverlay() {                                                                                       // This "projects" colors on already drawn leds before showing leds in updateDisplay();
  for (int i = 0; i < LED_COUNT; i++) {                                                                     // check each led...
    if (leds[i])                                                                                            // ...and if it is lit...
      leds[i] = ColorFromPalette(currentPalette, startColor * 2, brightness, LINEARBLEND);                  // ...assign color from current palette
  }
}

void updateDisplay(byte color, byte colorSpacing) {                                                         // this is what redraws the "screen"
  FastLED.clear();                                                                                          // clear whatever the leds might have assigned currently...
  displayTime(now(), color, colorSpacing);                                                                  // ...set leds to display the time...
  if (overlayMode == 1) colorOverlay();                                                                     // ...and if using overlayMode = 1 draw custom colors over single leds
  if (brightnessAuto == 1) {                                                                                // If brightness is adjusted automatically by using readLDR()...
    FastLED.setBrightness(avgLDR);                                                                          // ...set brightness to avgLDR
  } else {                                                                                                  // ...otherwise...
    FastLED.setBrightness(brightness);                                                                      // ...assign currently selected brightness
  }
}

byte readButtons() {
  // returns "1", "2" or "12" for button A, B or A+B
  // keeps track of long presses and increases repeat rate
  byte activeButton = 0;
  byte retVal = 0;
  static int btnRepeatDelay = 150;
  static unsigned long lastButtonPress = 0;
  if ( digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0) {
    if (digitalRead(buttonA) == 0) activeButton = 1;
    else if (digitalRead(buttonB) == 0) activeButton = 2;
    if ( digitalRead(buttonA) == 0 && digitalRead(buttonB) == 0 ) activeButton = 12;
    if (millis() - lastButtonPress >= btnRepeatDelay) {
      btnRepeatStart = millis();
      btnRepeatCounter = 0;
      retVal = activeButton;
    } else if (millis() - btnRepeatStart >= btnRepeatDelay * (btnRepeatCounter + 1) ) {
      btnRepeatCounter++;
      if (btnRepeatCounter > 5) retVal = activeButton;
    }
    lastButtonPress = millis();
  }
  return retVal;
}

void displayTime(time_t t, byte color, byte colorSpacing) {
  byte posOffset = 0;                                                                     // this offset will be used to move hours and minutes...
  if ( LED_DIGITS / 2 > 2) posOffset = 2;                                                 // ... to the left so we have room for the seconds when there's 6 digits available
  if ( displayMode == 0 ) {                                                               // if 12h mode is selected...
    if ( hourFormat12(t) >= 10 ) showDigit(1, color + colorSpacing * 2, 3 + posOffset);   // ...and hour > 10, display 1 at position 3
    showDigit((hourFormat12(t) % 10), color + colorSpacing * 3, 2  + posOffset);          // display 2nd digit of HH anyways
  } else if ( displayMode == 1 ) {                                                        // if 24h mode is selected...
    if ( hour(t) > 9 ) showDigit(hour(t) / 10, color + colorSpacing * 2, 3 + posOffset);  // ...and hour > 9, show 1st digit at position 3 (this is to avoid a leading 0 from 0:00 - 9:00 in 24h mode)
    showDigit(hour(t) % 10, color + colorSpacing * 3, 2 + posOffset);                     // again, display 2nd digit of HH anyways
  }
  showDigit((minute(t) / 10), color + colorSpacing * 4, 1 + posOffset);                   // minutes thankfully don't differ between 12h/24h, so this can be outside the above if/else
  showDigit((minute(t) % 10), color + colorSpacing * 5, 0 + posOffset);                   // each digit is drawn with an increasing color (*2, *3, *4, *5) (*6 and *7 for seconds only in HH:MM:SS)
  if ( posOffset > 0 ) {
    showDigit((second(t) / 10), color + colorSpacing * 6, 1);
    showDigit((second(t) % 10), color + colorSpacing * 7, 0);
  }
  showDots(2, second(t) * 4.25);                                // show : between hours and minutes with the color cycling through the palette once per minute
  lastSecond = second(t);
}

void showSegment(byte segment, byte color, byte segDisplay) {
  // This shows the segments from top of the sketch on a given position (segDisplay).
  // pos 0 is the most right one (seen from the front) where data in is connected
  int startLED = pgm_read_word_near(&segGroups[segment][0]);
  int endLED = pgm_read_word_near(&segGroups[segment][1]);
  int offsetLED = 0;
  if ( segDisplay > 0 ) offsetLED = segDisplay * LED_PER_DIGIT_STRIP / 2 + segDisplay * SPACING_LEDS;                 // if position/display is greater 0 we add half the leds of a digit
  if ( segDisplay >= 2 ) offsetLED += segDisplay / 2 * LED_PER_CENTER_MODULE / 2 + segDisplay / 2 * SPACING_LEDS; // if position/display is greater 1 we have to add offsets for the dots
  for (int i = startLED; i <= endLED; i++) {                                                  // light up group 1 (1st and 2nd value inside segGroups array)
    if ( segment == 0 || segment == 1 || segment == 5 ) leds[i + offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND); // upper 3 segments
    if ( segment == 2 || segment == 3 || segment == 4 ) leds[i - offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND); // lower 3 segments
    if ( segment == 6 ) leds[i + offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);                                  // upper part of center segment
  }
  startLED = pgm_read_word_near(&segGroups[segment][2]);
  endLED = pgm_read_word_near(&segGroups[segment][3]);
  for (int i = startLED; i <= endLED; i++) {                                                  // light up group 2 (3rd and 4th value inside segment array)
    if ( segment == 0 || segment == 1 || segment == 5 ) leds[i + offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);  // upper 3 segments
    if ( segment == 2 || segment == 3 || segment == 4 ) leds[i - offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);  // lower 3 segments
    if ( segment == 6 ) leds[i - offsetLED] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);                                  // lower part of center segment
  }
}

void showDigit(byte digit, byte color, byte pos) {
  // This draws numbers using the according segments as defined on top of the sketch (0 - 9)
  for (byte i = 0; i < 7; i++) {
    if ( pgm_read_byte_near(&digits[digit][i]) != 0) showSegment(i, color, pos);
  }
}

void showDots(byte dots, byte color) {
  // in 12h mode while in setup single/upper dot(s) resemble(s) AM, both/all dots resemble PM
  int startPos = LED_PER_DIGIT_STRIP + SPACING_LEDS * 2;
  byte distance = LED_PER_CENTER_MODULE / 2;
  for ( byte i = 0; i < distance; i++ )                                          // right upper dot
    leds[startPos + i] = ColorFromPalette(currentPalette, color + ( i + 4 ) * colorOffset, brightness, LINEARBLEND);
  if ( LED_DIGITS > 4 ) {
    startPos += LED_PER_DIGIT_STRIP + SPACING_LEDS * 6;                                             // left upper dot
    for ( byte i = 0; i < distance; i++ ) 
      leds[startPos + i] = ColorFromPalette(currentPalette, color + ( i + 4 ) * colorOffset, brightness, LINEARBLEND);
  }
  if ( dots == 2 ) {
    startPos = LED_COUNT - 1 - ( LED_PER_DIGIT_STRIP + SPACING_LEDS * 2 );                          // right lower dot
    for ( byte i = 0; i < distance; i++ )
      leds[startPos - i] = ColorFromPalette(currentPalette, color + ( i + 4 ) * colorOffset, brightness, LINEARBLEND);
    if ( LED_DIGITS > 4 ) {
      startPos -= LED_PER_DIGIT_STRIP + SPACING_LEDS * 8 + 1;                                       // left lower dot
      for ( byte i = 0; i < distance; i++ )
        leds[startPos + i] = ColorFromPalette(currentPalette, color + ( i + 4 ) * colorOffset, brightness, LINEARBLEND);
    }
  }
}

void dbgInput() {
  // this catches input from the serial console and translates it into some used variables/settings
  // this can also be used when there's no buttons connected: 7/8/9 -> Button A/B/A+B
  if ( dbg ) {
    if ( Serial.available() > 0 ) {
      byte incomingByte = 0;
      incomingByte = Serial.read();
      if ( incomingByte == 44 ) if ( overlayMode == 0 ) overlayMode = 1; else overlayMode = 0;  // ,
      if ( incomingByte == 48 ) if ( displayMode == 0 ) displayMode = 1; else displayMode = 0;  // 0
      if ( incomingByte == 49 ) FastLED.setTemperature(OvercastSky);                            // 1
      if ( incomingByte == 50 ) FastLED.setTemperature(DirectSunlight);                         // 2 
      if ( incomingByte == 51 ) FastLED.setTemperature(Halogen);                                // 3
      if ( incomingByte == 52 ) overlayInterval = 30;                                           // 4
      if ( incomingByte == 53 ) colorChangeInterval = 10;                                       // 5
      if ( incomingByte == 54 ) { overlayInterval = 200; colorChangeInterval = 1500; }          // 6
      if ( incomingByte == 55 ) lastKeyPressed = 1;                                             // 7
      if ( incomingByte == 56 ) lastKeyPressed = 2;                                             // 8
      if ( incomingByte == 57 ) lastKeyPressed = 12;                                            // 9
      if ( incomingByte == 43 ) colorOffset += 8;                                               // +
      if ( incomingByte == 45 ) colorOffset -= 8;                                               // -
      Serial.print(F("Input - ASCII: ")); Serial.println(incomingByte, DEC); Serial.println();
    }
  }
}

void loadValuesFromEEPROM() {
  byte tmp = EEPROM.read(2);
  if ( tmp <= 1 ) displayMode = tmp; else displayMode = 0;        // if no values have been stored to eeprom at position 2/3 then a read will give us something unusable...
  tmp = EEPROM.read(3);                                           // ...so we'll only take the value from eeprom if it's smaller than 1
  if ( tmp <= 1 ) overlayMode = tmp; else overlayMode = 0;        // (for colorMode/displayMode which can only bei 0/1)
}

void switchPalette() {                                            // Simply add palettes, make sure paletteCount increases accordingly
  byte paletteCount = 7;                                          // A few examples of gradients/solid colors by using RGB values or HTML Color Codes
  byte tmp = EEPROM.read(0);
  if ( dbg ) { Serial.print(F("switchPalette() EEPROM value: ")); Serial.println(tmp); }
  if ( tmp > paletteCount - 1 ) tmp = 0; else tmp = tmp;          // If value hasn't been written yet eeprom.read might return something > paletteCount, in that case set tmp = 0
  static byte selectedPalette = tmp;
  switch ( selectedPalette ) {
    case 0: currentPalette = CRGBPalette16( CRGB( 224,   0,  32 ),
                                            CRGB(   0,   0, 244 ),
                                            CRGB( 128,   0, 128 ),
                                            CRGB( 224,   0,  64 ) ); break;
    case 1: currentPalette = CRGBPalette16( CRGB( 224,  16,   0 ),
                                            CRGB( 192,  64,   0 ),
                                            CRGB( 128, 128,   0 ),
                                            CRGB( 224,  32,   0 ) ); break;
    case 2: currentPalette = CRGBPalette16( CRGB::Aquamarine,
                                            CRGB::Turquoise,
                                            CRGB::Blue,
                                            CRGB::DeepSkyBlue   ); break;
    case 3: currentPalette = RainbowColors_p; break;
    case 4: currentPalette = PartyColors_p; break;
    case 5: currentPalette = CRGBPalette16( CRGB::White ); break;
    case 6: currentPalette = CRGBPalette16( CRGB::LawnGreen ); break;
  }
  EEPROM.put(0, selectedPalette);
  #ifdef nodeMCU                                                  // on nodeMCU we need to commit the changes from ram to flash to make them permanent
    EEPROM.commit();
  #endif
  if (dbg) { Serial.print(F("switchPalette() EEPROM write: ")); Serial.println(selectedPalette); Serial.println(); }
  if (selectedPalette < paletteCount - 1) selectedPalette++; else selectedPalette = 0;
}

void switchBrightness() {
  byte tmp = EEPROM.read(1);
  if ( dbg ) { Serial.print(F("switchBrightness() EEPROM value: ")); Serial.println(tmp); }
  if ( tmp > 2 ) tmp = 0;                                         // If value hasn't been written yet eeprom.read might return something > brightnessLevels (0-2), in that case set tmp = 0
  static byte selectedBrightness = tmp;
  switch ( selectedBrightness ) {
    case 0: brightness = brightnessLevels[selectedBrightness]; break;
    case 1: brightness = brightnessLevels[selectedBrightness]; break;
    case 2: brightness = brightnessLevels[selectedBrightness]; break;
  }
  EEPROM.put(1, selectedBrightness);
  #ifdef nodeMCU                                                  // on nodeMCU we need to commit the changes from ram to flash to make them permanent
    EEPROM.commit();
  #endif
  if ( dbg ) { Serial.print(F("switchBrightness() EEPROM write: ")); Serial.println(selectedBrightness); Serial.println(); }
  if ( selectedBrightness < 2 ) selectedBrightness++; else selectedBrightness = 0;
}

DEFINE_GRADIENT_PALETTE (setupColors_gp) {                                                  // this color palette will only be used while in setup
    0, 240, 240,   0,                                                                       // unset values = red, current value = yellow, set values = green
   64, 240, 240,   0,
   96, 240,   0,   0,
  160, 240,   0,   0,
  224,   0, 240,   0,
  255,   0, 240,   0
};

void setupClock() {
  // finally not using a custom displayTime routine for setup, improvising a bit and using the setupColor-Palette defined on top of the sketch
  if ( dbg ) Serial.println(F("Entering setup mode..."));
  byte prevBrightness = brightness;                                                         // store current brightness and switch back after setup
  brightness = brightnessLevels[1];                                                         // select medium brightness level
  currentPalette = setupColors_gp;                                                          // use setupColors_gp palette while in setup
  tmElements_t setupTime;                                                                   // Create a time element which will be used. Using the current time would
  setupTime.Hour = 12;                                                                      // give some problems (like time still running while setting hours/minutes)
  setupTime.Minute = 0;                                                                     // Setup starts at 12 (12 pm)
  setupTime.Second = 1;                                                                     // 1 because displayTime() will always display both dots at even seconds
  setupTime.Day = 15;                                                                       // not really neccessary as day/month aren't used but who cares ^^
  setupTime.Month = 5;                                                                      // see above
  setupTime.Year = 2020 - 1970;                                                             // yes... .Year is set by the difference since 1970. So "50" is what we want.
  byte setting = 1;                                                                         // counter to keep track of what's currently adjusted: 1 = hours, 2 = minutes
  byte blinkStep = 0;
  int blinkInterval = 500;
  unsigned long lastBlink = millis();
  FastLED.clear();
  FastLED.show();
  while ( digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0 ) delay(20);               // this will keep the display blank while any of the keys is still pressed
  while ( setting <= LED_DIGITS / 2 ) {                                                     // 2 - only setup HH:MM. 3 - setup HH:MM:SS
    if ( lastKeyPressed == 1 ) setting += 1;                                                // one button will accept the current setting and proceed to the next one...
    if ( lastKeyPressed == 2 )                                                              // while the other button increases the current value
      if ( setting == 1 )                                                                   // if setting = 1 ...
        if ( setupTime.Hour < 23 ) setupTime.Hour += 1; else setupTime.Hour = 0;            // ...increase hour when buttonB is pressed
      else if ( setting == 2 )                                                              // else if setting = 2...
        if (setupTime.Minute < 59) setupTime.Minute += 1; else setupTime.Minute = 0;        // ...increase minute when buttonB is pressed
      else if ( setting == 3 )                                                              // else if setting = 3...
        if (setupTime.Second < 59) setupTime.Second += 1; else setupTime.Second = 0;        // ...increase second when buttonB is pressed
    if ( millis() - lastBlink >= blinkInterval ) {                                          // pretty sure there is a much easier and nicer way...
      if ( blinkStep == 0 ) { brightness = brightnessLevels[2]; blinkStep = 1; }            // ...to get the switch between min and max brightness (boolean?)
        else { brightness = brightnessLevels[0]; blinkStep = 0; }                           
      lastBlink = millis();
    }
    FastLED.clear();
    if ( setting == 1 ) {
      displayTime(makeTime(setupTime), 0, 24);
    } else if ( setting == 2 ) {
      displayTime(makeTime(setupTime), 170, 24);
    } else if ( setting == 3 ) {
      displayTime(makeTime(setupTime), 0, 100);
    }
    if ( ( setupTime.Hour < 12 ) && ( displayMode == 0 ) ) showDots(1, 220); else showDots(2, 220);
    FastLED.show();
    lastKeyPressed = readButtons();
    if ( dbg ) dbgInput();
  }
  #ifdef nodeMCU                                                                              // if building for nodeMCU...
    #ifndef useWiFi                                                                           // ...without WiFi...
      Rtc.SetDateTime(makeTime(setupTime));                                                   // ...write setupTime to the rtc
    #endif
  #else                                                                                       // if building for Arduino...
    RTC.write(setupTime);                                                                     // write setupTime to RTC
  #endif
  setTime(makeTime(setupTime));
  FastLED.clear(); displayTime(makeTime(setupTime), 95, 0); FastLED.show();
  brightness = prevBrightness;
  switchPalette();
  delay(500);                                                                                 // short delay followed by fading all leds to black
  for ( byte i = 0; i < 255; i++ ) {
    for ( int x = 0; x < LED_COUNT; x++ ) leds[x]--;
    FastLED.show(); delay(2);
  }
  if ( dbg ) Serial.println(F("Setup done..."));
}

// stuff below will only be used when compiled for nodeMCU _AND_ using WiFi

#ifdef useWiFi
  void syncNTP() {                                                                            // gets time from ntp and sets internal time accordingly, will return when no connection is established
    if ( dbg ) Serial.println(F("Entering syncNTP()..."));
    if ( WiFi.status() != WL_CONNECTED ) {
      if ( dbg ) Serial.println(F("No active WiFi connection!"));
      return;
    }                                                                                         // Sometimes the connection doesn't work right away although status is WL_CONNECTED...
    delay(1500);                                                                              // ...so we'll wait a moment before causing network traffic
    timeClient.update();
    setTime(timeClient.getEpochTime());
    if ( dbg ) {
      Serial.print(F("nodemcu time: ")); Serial.println(now());
      Serial.print(F("ntp time    : ")); Serial.println(timeClient.getEpochTime());
      Serial.println(F("syncNTP() done..."));
    }
  }
  
  void initWPS() {                                                                            // join network using wps. Will try for 5 times before exiting...
    byte i = 1;
    if ( dbg ) Serial.println(F("Initializing WPS setup..."));
    FastLED.clear();
    while ( i <= 5 ) {
      showDigit(10, 64, 3);
      showDigit(11, 64, 2);
      showDigit(12, 64, 1);
      showDigit(i, 0, 0);
      FastLED.show();
      FastLED.clear();
      if ( dbg ) Serial.println(F("Waiting for WiFi/WPS..."));
      ESP.wdtFeed();
      WiFi.beginWPSConfig();
      delay(500);
      if ( WiFi.SSID().length() <= 0 ) i++; else i = 6;
    }
    if ( WiFi.SSID().length() > 0 ) {                                                         // after getting the ssid we'll wait a few seconds and show a counter to make...
      showDigit(5, 0, 3);  showDigit(5, 0, 2);                                                // ...sure there's some time passing for dhcp/getting wifi settings
      showDigit(1, 0, 1);  showDigit(13, 0, 0);
      ESP.wdtFeed();
      FastLED.show(); delay(2000); FastLED.clear();
      if ( dbg ) {
        Serial.print(F("Connected to SSID: ")); Serial.println(WiFi.SSID());
        Serial.println(F("New WiFi connection, calling syncNTP..."));
      }
      for ( i = 5; i > 0; i--) {
        showDigit(i, 0, 0); FastLED.show(); delay(1000); FastLED.clear();
      }
      syncNTP();
    } else {
      if ( dbg ) { Serial.print(F("No connection established.")); Serial.println(WiFi.SSID()); }
    }
  }
#endif
