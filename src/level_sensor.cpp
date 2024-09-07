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
  int mm_inc_nom = 643;
  int mm_inc_denom = 250;
  static int secs_till_last_log = 0;
  bool transmitted = false;
  bool connected = Particle.connected();

  short adc0 = ads.readADC_SingleEnded(0);
  int zero_based = adc0 - zero_value;
  int mm = (zero_based*mm_inc_denom)/mm_inc_nom;

  char output[10] = {};
  if (mm < 0)
  {
    // below zero
    snprintf(output, 9, " 0.0cm");
  }
  else if (mm > 10000)
  {
    // above max
    snprintf(output, 9, "10.0 m");
  }
  else if (mm < 1000)
  {
    // under a meter, show in cm
    snprintf(output, 9, "%2d.%1dcm", mm / 10, mm % 10);
  }
  else
  {
    // above a meter, show in m
    snprintf(output, 9, "%2d.%1d m", mm / 100, (mm/10) % 10);
  }

  if (secs_till_last_log == 0 && connected)
  {
    char str_mm[10] = {};
    char str_raw[10] = {};

    snprintf(str_mm, 9, "%d", mm);
    snprintf(str_raw, 9, "%d", adc0);

    transmitted = Particle.publish("depth_mm", str_mm);
    Particle.publish("raw_16bit", str_raw);
  }

  display.clearDisplay();
  display.setCursor(0,10);
  display.println(output);
  if (transmitted)
    display.println("v");
  else if (!connected)
    display.println("x");

  display.display();

  // reset counter after a minute, publish only then
  if (++secs_till_last_log == 120)
    secs_till_last_log = 0;

  delay( 500 ); // milliseconds and blocking - see docs for more info!
}
