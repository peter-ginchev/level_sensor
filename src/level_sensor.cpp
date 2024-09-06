/* 
 * Project Level Sensor
 * Author: Peter Ginchev
 * Date: September, 2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

// setup() runs once, when the device is first turned on
void setup() {
  Log.info("Setup..");
  Serial.begin(9600);
  ads.setGain(GAIN_TWO);
  ads.begin();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.setTextColor(WHITE);
  display.setTextSize(3);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  int zero_value = 6425;
  int mm_inc_nom = 5;
  int mm_inc_denom = 2;
  static int secs_till_last_log = 0;

  short adc0 = ads.readADC_SingleEnded(0);
  int zero_based = adc0 - zero_value;
  int mm = (zero_based*mm_inc_denom)/mm_inc_nom;

  char output[10] = {};
  if (mm < 0)
  {
    snprintf(output, 9, "---- cm");
  }
  else
  {
    snprintf(output, 9, "%3d.%1dcm", mm / 10, mm % 10);
  }

  if (secs_till_last_log == 0)
  {
    char str_mm[10] = {};
    snprintf(str_mm, 9, "%d", mm);
    Particle.publish("depth", str_mm);
  }

  display.clearDisplay();
  display.setCursor(0,10);
  display.println(output);
  display.display();

  // reset counter after a minute, publish only then
  if (++secs_till_last_log == 120)
    secs_till_last_log = 0;

  delay( 500 ); // milliseconds and blocking - see docs for more info!
}
