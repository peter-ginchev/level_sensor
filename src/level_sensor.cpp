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

#include <list>

#include <Wire.h>

// ADC Ti ADS1115, used in NCD PR33-8
#include <Adafruit_ADS1X15.h>

// E-Paper Display
#include <PDLS_Common.h>
#include <Pervasive_Wide_Small.h>
#include <PDLS_Basic.h>

Pervasive_Wide_Small epdDriver(eScreen_EPD_417_KS_0D, boardParticlePhoton2);

// Delay between probes
#define DELAY_MS 500

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

class DisplayLineInterface
{
public:
  virtual void update(Screen_EPD *screen) = 0;
};

class Display
{
public:
  Display(Pervasive_Wide_Small *driver): screen(driver)
  {
    hV_HAL_SPI3_define();
    screen.begin();
    screen.regenerate();
    screen.setOrientation(ORIENTATION_LANDSCAPE);
    screen.selectFont(Font_Terminal12x16);
  }

  void tick(void)
  {
    display();
  }

protected:
  friend class DisplayLine;

  void addLine(DisplayLineInterface *line)
  {
    lines.push_back(line);
  }
private:
  Screen_EPD screen;
  std::list<DisplayLineInterface*> lines;

  void display(void)
  {
    screen.clear();
    for (auto line : lines)
    {
      line->update(&screen);
    }
    screen.flush();
  }
};

class DisplayLine : public DisplayLineInterface
{
public:
  // Negative line numbers are counted from the bottom of the screen
  // Numbers start from 1 and -1
  DisplayLine(Display *display, int line): display(display), line(line)
  {
    display->addLine(this);
  }
  void setText(String text)
  {
    this->text = text;
    this->param = "";
  }
  void setText(String text, String param)
  {
    this->text = text;
    this->param = param;
  }

protected:
  friend class Display;

  void update(Screen_EPD *screen)
  {
    updateCoordinates(screen);

    if (line < 0)
    {
      screen->gText(x, y, text);
      if (param.length() > 0)
        screen->gText(x + 2*screen->stringSizeX(text + " "), y, param);
    }
    else
    {
      screen->gTextLarge(x, y, text);
      if (param.length() > 0)
        screen->gTextLarge(x + 2*screen->stringSizeX(text + " "), y, param);
    }
  }

private:
  Display *display;
  int line;

  String text, param;

  const uint16_t xOffset = 10;
  const uint16_t yOffset = 10;
  const uint16_t yPitch = 10;
  uint16_t x, y;

  void updateCoordinates(Screen_EPD *screen)
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
  DisplayTime(Display *display): line(display, -1)
  {
    Time.zone(+2.);
  }
  void tick(void)
  {
    updateDst();
    line.setText(Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL));
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

class CurrentSensor {
public:
  CurrentSensor(void)
  {
    ads.setGain(GAIN_TWO);
    ads.begin();
  }
protected:
  friend class CurrentSensorReading;

  uint16_t read(uint8_t id)
  {
    return ads.readADC_SingleEnded(id);
  }
private:
  Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
};

class CurrentSensorReading
{
public:
  CurrentSensorReading(CurrentSensor *sensor, uint8_t id, uint64_t min, uint64_t max)
  : sensor(sensor), id(id)
  {
    setRange(min, max);
  }

  uint32_t read(void)
  {
    uint16_t value = sensor->read(id);

    return translate(value);
  }
private:
  CurrentSensor *sensor;
  uint8_t id;

  void setRange(uint64_t min, uint64_t max)
  {
    this->min = min;
    this->range = max - min;
  }

  uint32_t translate(uint16_t adc)
  {
    if (adc < zero_value)
      return min; // usually means the sensor is not connected

    uint64_t zero_based = adc - zero_value;

    return min + (zero_based*inc_denom*range)/(inc_nom*inc_output);
  }
private:
  static const uint16_t zero_value = 6425;
  static const uint64_t inc_nom = 643;
  static const uint64_t inc_denom = 250;
  static const uint64_t inc_output = 10000;
  uint64_t min;
  uint64_t range;
};

class WaterLevel {
public:
  WaterLevel(Display *display, unsigned screenLine, CurrentSensor *sensor)
  : line(display, screenLine)
    // hydrostatic pressure sensor connected to output 0,
    // range 0-10m, 0-10000 received in mm
  , sensor(sensor, 0, 0, 10000)
  , level_mm(0)
  {}

  void readValue(void)
  {
    level_mm = sensor.read();
    setText();
    transmit.loop(level_mm);
  }

private:
  DisplayLine line;
  CurrentSensorReading sensor;
  unsigned level_mm;
  TransmitAverage transmit = TransmitAverage("depth_mm");

  void setText()
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

    line.setText("Level:", output);
  }
};

class Pressure {
public:
  Pressure(Display *display, unsigned screenLine, CurrentSensor *sensor)
  : line(display, screenLine)
    // water pressure sensor connected to output 1,
    // range 0-6bar, 0-6000 received in mbar
  , sensor(sensor, 1, 0, 6000)
  , mbar(0)
  {}

  void readValue(void)
  {
    mbar = sensor.read();
    setText();
    transmit.loop(mbar);
  }

private:
  DisplayLine line;
  CurrentSensorReading sensor;
  unsigned mbar;
  TransmitAverage transmit = TransmitAverage("pressure_mbar");

  void setText()
  {
    char output[10] = {};
    snprintf(output, sizeof(output) - 1, "%01d.%2dbar", mbar / 1000, (mbar/10) % 100);
    line.setText("Pressure:", output);
  }
};

class Relay {
public:
  Relay(void)
  {
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
  }
  void set(bool high)
  {
    digitalWrite(relayPin, high ? HIGH : LOW);
  }
private:
  const int relayPin = S4;
};

Relay *relay;

CurrentSensor *currentSensor;

DisplayTime *displayTime;

WaterLevel *waterLevel;
Pressure *pressure;
DisplayLine *flowLine;
DisplayLine *pumpOnLine;

Display *display;

// setup() runs once, when the device is first turned on
void setup() {
  Log.info("Setup..");
  Serial.begin(9600);
  currentSensor = new CurrentSensor();

  display = new Display(&epdDriver);

  displayTime = new DisplayTime(display);

  waterLevel = new WaterLevel(display, 1, currentSensor);
  pressure = new Pressure(display, 2, currentSensor);

  flowLine = new DisplayLine(display, 3);
  flowLine->setText("Flow:");

  pumpOnLine = new DisplayLine(display, 4);
  pumpOnLine->setText("Pump:", "OFF");

  relay = new Relay();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  unsigned long long millis = System.millis();

  // TODO: display: Particle.connected();
  bool high = false;

  waterLevel->readValue();
  pressure->readValue();

  if ( ((Time.now() / 60) % 2 == 0) != high )
  {
    // change relay state
    high = !high;
  }
  relay->set(high);
  pumpOnLine->setText("Pump:", high ? "ON " : "OFF");
  displayTime->tick();
  display->tick();

  unsigned long long millis_diff = System.millis() - millis;
  if (millis_diff < DELAY_MS)
    delay(DELAY_MS - millis_diff); // milliseconds and blocking - see docs for more info!
}
