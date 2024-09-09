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

// OLED Display
#include <Adafruit_SSD1306.h>

// ADC Ti ADS1115, used in NCD PR33-8
#include <Adafruit_ADS1X15.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

// There are 2 probe reads per sec, every minute transmission
#define CYCLES_TRANSMIT_SECS 60

// Delay between probes
#define DELAY_MS 500

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

  display.begin();
  display.setTextColor(WHITE);
  display.setTextSize(3);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  int zero_value = 6425;
  int mm_inc_nom = 643;
  int mm_inc_denom = 250;
  static int samples_count = 0;
  static long sum_mm = 0;
  static time32_t last_sent = Time.now();

  unsigned long long millis = System.millis();

  bool transmitted = false;
  bool connected = Particle.connected();

  short adc0 = ads.readADC_SingleEnded(0);
  int zero_based = adc0 - zero_value;
  int mm = (zero_based*mm_inc_denom)/mm_inc_nom;
  sum_mm += mm;
  samples_count++;

  char output[10] = {};
  if (mm < 0)
  {
    // below zero
    snprintf(output, 9, " 0.0cm");
  }
  else if (mm > 10000)
  {
    // above max
    snprintf(output, 9, "10.00m");
  }
  else if (mm < 1000)
  {
    // under a meter, show in cm
    snprintf(output, 9, "%2d.%1dcm", mm / 10, mm % 10);
  }
  else
  {
    // above a meter, show in m
    snprintf(output, 9, "%2d.%02dm", mm / 1000, (mm/10) % 100);
  }

  if (Time.now() >= last_sent + CYCLES_TRANSMIT_SECS)
  {
    if (connected)
    {
      char str_mm[10] = {};
      char str_raw[10] = {};

      int average_mm = sum_mm / samples_count;
      snprintf(str_mm, 9, "%d", average_mm);
      snprintf(str_raw, 9, "%d", adc0);

      transmitted = Particle.publish("depth_mm", str_mm);
      Particle.publish("raw_16bit", str_raw);
    }

    // reset counter after CYCLES_TRANSMIT_SECS, publish only then, but only if connected
    samples_count = 0;
    sum_mm = 0;
    last_sent = Time.now();
  }

  display.clearDisplay();
  display.setCursor(0,10);
  display.println(output);
  if (transmitted)
    display.println("v");
  else if (!connected)
    display.println("x");

  display.display();

  unsigned long long millis_diff = System.millis() - millis;
  if (millis_diff < DELAY_MS)
    delay(DELAY_MS - millis_diff); // milliseconds and blocking - see docs for more info!
}
