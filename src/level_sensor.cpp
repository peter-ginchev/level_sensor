/* 
 * Project Level Sensor
 * Author: Peter Ginchev
 * Date: September, 2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

PRODUCT_VERSION(1);

#include <Wire.h>

// OLED Display
#include <Adafruit_SSD1306.h>

// ADC Ti ADS1115, used in NCD PR33-8
#include <Adafruit_ADS1X15.h>

// E-Paper Display
#include <PDLS_Common.h>
#include <Pervasive_Wide_Small.h>
#include <PDLS_Basic.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

Pervasive_Wide_Small epdDriver(eScreen_EPD_417_KS_0D, boardParticlePhoton2);
Screen_EPD epdScreen(&epdDriver);

// Delay between probes
#define DELAY_MS 500

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

int relayPin = S4;

class DisplayLine {
public:
  // Negative line numbers are counted from the bottom of the screen
  // Numbers start from 1 and -1
  DisplayLine(Screen_EPD *screen, int line): screen(screen), line(line)
  {
  }
  void display(String text)
  {
    updateCoordinates();

    if (line < 0)
      screen->gText(x, y, text);
    else
      screen->gTextLarge(x, y, text);
  }
  void display(String text, String param)
  {
    updateCoordinates();

    if (line < 0)
    {
      screen->gText(x, y, text);
      screen->gText(x + 2*screen->stringSizeX(text + " "), y, param);
    }
    else
    {
      screen->gTextLarge(x, y, text);
      screen->gTextLarge(x + 2*screen->stringSizeX(text + " "), y, param);
    }
  }

private:
  Screen_EPD *screen;
  int line;

  const uint16_t xOffset = 10;
  const uint16_t yOffset = 10;
  const uint16_t yPitch = 10;
  uint16_t x, y;

  void updateCoordinates(void)
  {
    x = xOffset;
    if (line < 0)
      y = screen->screenSizeY() - yOffset + line * screen->characterSizeY() + (line+1) * yPitch;
    else
      y = yOffset + (line-1) * (2*screen->characterSizeY() + yPitch);
  }
};

class DisplayTime {
public:
  DisplayTime(Screen_EPD *screen): line(screen, -1)
  {
    Time.zone(+2.);
  }
  void display(void)
  {
    updateDst();
    line.display(Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL));
  }
private:
  DisplayLine line;
  bool inDST = false;

  void updateDst(void)
  {
    bool nowDST = isNowDST();
    if (inDST == nowDST)
      return;

    inDST = nowDST;

    if (inDST)
      Time.beginDST();
    else
      Time.endDST();
  }

  // Function to check if DST is active in Bulgaria (EU rules)
  bool isNowDST(void) {
    int month = Time.month();
    int day = Time.day();
    int dow = Time.weekday() - 1; // 0 = Sunday, 6 = Saturday
    int hour = Time.hour();

    // DST starts: Last Sunday of March at 1:00 UTC (3:00 local, moves to 4:00)
    if (month > 3 && month < 10) return true; // April to September: always DST
    if (month < 3 || month > 10) return false; // Before March or after October: no DST

    // March: Check if it's after the last Sunday
    if (month == 3) {
      int lastSunday = day - dow; // Day of the last Sunday so far
      if (lastSunday + 7 <= 31) lastSunday += 7; // Ensure it's the last Sunday
      if (day < lastSunday) return false;
      if (day == lastSunday && hour < 3) return false; // Before 3:00 local time
      return true;
    }

    // October: Check if it's before the last Sunday
    if (month == 10) {
      int lastSunday = day - dow;
      if (lastSunday + 7 <= 31) lastSunday += 7;
      if (day > lastSunday) return false;
      if (day == lastSunday && hour >= 2) return false; // After 2:00 local (back to 3:00)
      return true;
    }

    return false; // Default case (shouldn’t hit)
  }
};

DisplayTime displayTime(&epdScreen);

class TransmitAverage {
public:
  // Every minute transmission
  const int CYCLES_TRANSMIT_SECS = 60;

  TransmitAverage(String name): name(name)
  {
    last_sent = Time.now();
  }

  bool loop(unsigned value)
  {
    bool transmitted = false;

    sum += value;
    samples_count++;

    if (Time.now() < last_sent + CYCLES_TRANSMIT_SECS)
      return false;

    if (Particle.connected())
    {
      char str[10] = {};
      snprintf(str, 9, "%ld", sum / samples_count);
      transmitted = Particle.publish(name, str);
    }

    samples_count = 0;
    sum = 0;
    last_sent = Time.now();
    return transmitted;
  }
private:
  String name;
  time32_t last_sent;
  long sum = 0;
  unsigned samples_count = 0;
};

class WaterLevel {
public:
  WaterLevel(Screen_EPD *screen, unsigned screenLine)
  : line(screen, screenLine)
  {}

  void display()
  {
    char output[10] = {};
    if (level_mm < 1000)
    {
      // under a meter, show in cm
      snprintf(output, sizeof(output) - 1, "%02d.%1dcm", level_mm / 10, level_mm % 10);
    }
    else
    {
      // above a meter, show in m
      snprintf(output, sizeof(output) - 1, "%2d.%02dm", level_mm / 1000, (level_mm/10) % 100);
    }

    line.display("Level:", output);
  }

  void setValue(unsigned level_mm)
  {
    this->level_mm = level_mm;
    transmit.loop(level_mm);
  }

private:
  DisplayLine line;
  unsigned level_mm;
  TransmitAverage transmit = TransmitAverage("depth_mm");
};

WaterLevel waterLevel(&epdScreen, 1);
DisplayLine pressureLine(&epdScreen, 2);
DisplayLine flowLine(&epdScreen, 3);
DisplayLine pumpOnLine(&epdScreen, 4);

void updateScreen(uint16_t pressure, uint16_t flow, bool pumpOn)
{
  epdScreen.clear();

  waterLevel.display();
  pressureLine.display("Pressure:");
  flowLine.display("Flow:");
  pumpOnLine.display("Pump:", pumpOn ? "ON " : "OFF");

  displayTime.display();

  epdScreen.flush();
}

class CurrentSensor {
public:
  CurrentSensor(void)
  {
    ads.setGain(GAIN_TWO);
    ads.begin();
  }
  void setRange(uint8_t id, uint64_t min, uint64_t max)
  {
    this->min[id] = min;
    this->range[id] = max - min;
  }
  uint32_t read(uint8_t id)
  {
    uint16_t adc = ads.readADC_SingleEnded(id);
    if (adc < zero_value)
      return min[id]; // usually means the sensor is not connected

    int zero_based = adc - zero_value;

    return min[id] + (zero_based*inc_denom*range[id])/(inc_nom*inc_output);
  }
private:
  Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
  static const uint16_t zero_value = 6425;
  static const uint64_t inc_nom = 643;
  static const uint64_t inc_denom = 250;
  static const uint64_t inc_output = 10000;
  uint64_t min[2];
  uint64_t range[2];
};

CurrentSensor *currentSensor;

// setup() runs once, when the device is first turned on
void setup() {
  Log.info("Setup..");
  Serial.begin(9600);
  currentSensor = new CurrentSensor();
  // hydrostatic pressure sensor connected to output 0,
  // range 0-10m, 0-10000 received in mm
  currentSensor->setRange(0, 0, 10000);

  display.begin();
  display.setTextColor(WHITE);
  display.setTextSize(3);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  hV_HAL_SPI3_define();
  epdScreen.begin();
  epdScreen.regenerate();
  epdScreen.setOrientation(ORIENTATION_LANDSCAPE);
  epdScreen.selectFont(Font_Terminal12x16);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  unsigned long long millis = System.millis();

  bool connected = Particle.connected();
  bool high = false;

  waterLevel.setValue(currentSensor->read(0));

  if ( ((Time.now() / 60) % 2 == 0) != high )
  {
    // change relay state
    high = !high;
  }
  digitalWrite(relayPin, high ? HIGH : LOW);

  display.clearDisplay();
  display.setCursor(0,10);
  if (!connected)
    display.println("x");
  else
  {
    String isHigh = high ? "h" : "l";
    display.println(isHigh);
  }

  display.display();
  updateScreen(0, 0, high);

  unsigned long long millis_diff = System.millis() - millis;
  if (millis_diff < DELAY_MS)
    delay(DELAY_MS - millis_diff); // milliseconds and blocking - see docs for more info!
}
