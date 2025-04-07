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
#include <deque>
#include <numeric>
#include <atomic>
#include <functional>

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
  DisplayLine(Display *display, int line)
  : display(display)
  , line(line)
  {
    gText = line > 0 ?
      [](Screen_EPD *screen, uint16_t x0, uint16_t y0, String text) { screen->gTextLarge(x0, y0, text); } :
      [](Screen_EPD *screen, uint16_t x0, uint16_t y0, String text) { screen->gText(x0, y0, text); };
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

    gText(screen, x, y, text);
    if (param.length() > 0)
      gText(screen, x + 2*screen->stringSizeX(text + " "), y, param);
  }

private:
  Display *display;
  int line;

  String text, param;

  const uint16_t xOffset = 10;
  const uint16_t yOffset = 10;
  const uint16_t yPitch = 10;
  uint16_t x, y;

  using TextFunction = std::function<void(Screen_EPD *screen, uint16_t x0, uint16_t y0, String text)>;
  TextFunction gText;

  void updateCoordinates(Screen_EPD *screen)
  {
    x = xOffset;
    if (line < 0)
      y = screen->screenSizeY() - yOffset + line * screen->characterSizeY() + (line+1) * yPitch;
    else
      y = yOffset + (line-1) * (2*screen->characterSizeY() + yPitch);
  }
};

class DisplayTime
{
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
  bool isNowDST(void)
  {
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

class TransmitAverage
{
public:
  // Every minute transmission
  const int CYCLES_TRANSMIT_SECS = 60;

  TransmitAverage(String name): name(name)
  {
    last_sent = Time.now();
  }

  bool loop(uint32_t value)
  {
    bool transmitted = false;

    sum += value;
    samples_count++;

    if (Time.now() < last_sent + CYCLES_TRANSMIT_SECS)
      return false;

    if (Particle.connected())
    {
      char str[10] = {};
      snprintf(str, 9, "%lld", sum / samples_count);
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
  uint64_t sum = 0;
  unsigned samples_count = 0;
};

class CurrentSensor
{
public:
  CurrentSensor(void)
  {
    ads.setGain(GAIN_TWO);
    ads.begin();
  }
protected:
  friend class CurrentSensorChannel;

  uint16_t read(uint8_t id)
  {
    return ads.readADC_SingleEnded(id);
  }
private:
  Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
};

class CurrentSensorChannel
{
public:
  CurrentSensorChannel(CurrentSensor *sensor, uint8_t id, uint64_t min, uint64_t max)
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

enum class SensorDecisionTriState
{
  STOP = 0,
  OK_TO_STOP = 1,
  START = 2
};

class SensorInterface
{
public:
  SensorInterface(Display *display, unsigned screenLine, String name, String transmitName)
  : line(display, screenLine), displayName(name), transmit(transmitName)
  {
    line.setText("N/A");
    // Aligning all display names to the same length
    if (displayName.length() > displayLen)
    {
      // abbreviate
      displayName = displayName.substring(0, displayLen - 1) + ".";
    }
    else if (displayName.length() < displayLen) {
      // pad with spaces
      for (unsigned i = displayName.length(); i < displayLen; i++)
        displayName += ' ';
    }
    displayName += ":";
  }

  virtual SensorDecisionTriState decide(uint32_t value, bool lastPumpOn) = 0;

  uint32_t update_and_get(void)
  {
    uint32_t value = readValue();
    String txtValue = getText(value);
    line.setText(displayName, txtValue);
    transmit.loop(value);
    return value;
  }

protected:
  virtual uint32_t readValue(void) = 0;
  virtual String getText(uint32_t) = 0;

private:
  DisplayLine line;
  String displayName;
  TransmitAverage transmit;
  static constexpr unsigned displayLen = 6;
};

class WaterLevel: public SensorInterface
{
public:
  WaterLevel(Display *display, unsigned screenLine, CurrentSensor *sensor)
  : SensorInterface(display, screenLine, "Level", "depth_mm")
    // hydrostatic pressure sensor connected to output 0,
    // range 0-10m, 0-10000 received in mm
  , sensor(sensor, 0, 0, 10000) { }

  virtual SensorDecisionTriState decide(uint32_t level_mm, bool lastPumpOn) override
  {
    /* Water level is used just as preventive -- if level is low, stop before dry pump overheats */
    if (level_mm < 1500)
      return SensorDecisionTriState::STOP;
    return SensorDecisionTriState::OK_TO_STOP;
  }

protected:
  uint32_t readValue(void) override
  {
    return sensor.read();
  }

  virtual String getText(uint32_t level_mm) override
  {
    char output[16] = {};
    if (level_mm < 1000)
    {
      // under a meter, show in cm
      snprintf(output, sizeof(output) - 1, "%2lu.%1lu cm", level_mm / 10, level_mm % 10);
    }
    else
    {
      // above a meter, show in m
      snprintf(output, sizeof(output) - 1, "%2lu.%02lu m", level_mm / 1000, (level_mm/10) % 100);
    }
    return output;
  }

private:
  CurrentSensorChannel sensor;
};

class Pressure: public SensorInterface
{
public:
  Pressure(Display *display, unsigned screenLine, CurrentSensor *sensor)
  : SensorInterface(display, screenLine, "Pressure", "pressure_mbar")
    // water pressure sensor connected to output 1,
    // range 0-6bar, 0-6000 received in mbar
  , sensor(sensor, 1, 0, 6000) { }

  virtual SensorDecisionTriState decide(uint32_t pressure_mbar, bool lastPumpOn) override
  {
    /* Pressure below designed high pressure keeps the pump on,
     * the pump will stop when pressure is high and the flow is very low */
    if (lastPumpOn && pressure_mbar < 5000)
      return SensorDecisionTriState::START;
    /* The pump will start just when there could be no time to spool the pump, w/o experiencing it,
     * Otherwise high flow rate will start it anyway */
    if (!lastPumpOn && pressure_mbar < 4000)
      return SensorDecisionTriState::START;
    return SensorDecisionTriState::OK_TO_STOP;
}

protected:
  uint32_t readValue(void) override
  {
    return sensor.read();
  }

  virtual String getText(uint32_t mbar) override
  {
    char output[16] = {};
    snprintf(output, sizeof(output) - 1, "%01lu.%02lu bar", mbar / 1000, (mbar/10) % 100);
    return output;
  }

private:
  CurrentSensorChannel sensor;
};

class WaterMeter
{
public:
  WaterMeter(int pin)
  {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, &WaterMeter::pulseInterrupt, this, FALLING);
  }

  uint32_t getAndResetCount(void)
  {
    return pulseCount.exchange(0);
  }

  uint32_t getCount(void)
  {
    return pulseCount.load();
  }

private:
  const uint32_t debounceMs = 30;
  std::atomic_uint32_t pulseCount = 0;

  void pulseInterrupt(void)
  {
    static uint32_t lastPulse;
    uint32_t currentTime = millis();

    ATOMIC_BLOCK() {
      if (currentTime - lastPulse < debounceMs)
        return;
      lastPulse = currentTime;
    }

    pulseCount++;
  }
};

class FlowRate: public SensorInterface
{
public:
  FlowRate(Display *display, unsigned screenLine, WaterMeter *meter)
  : SensorInterface(display, screenLine, "Flow", "flow_lpm")
  , meter(meter)
  , lastPulseTime(System.millis())
  , flowLPM(0)
  , noFlow(true)
  , startFlow(false)
  {
    meter->getAndResetCount();
  }
  virtual SensorDecisionTriState decide(uint32_t flowLPM, bool lastPumpOn) override
  {
    /* Flow rate starts the pump, immediately after there's a significant flow,
     * the pump is expected to stop, when the flow is low and the pressure has built up */
    if (flowLPM > 5)
      return SensorDecisionTriState::START;
    return SensorDecisionTriState::OK_TO_STOP;
  }

protected:
  uint32_t readValue(void) override
  {
    uint64_t currentTime = System.millis();
    uint32_t pulseTime = currentTime - lastPulseTime;
    uint32_t pulses = 0;

    if (noFlow)
    {
      /* if we didn't have a flow and receive a pulse, it's hard to estimate the flow rate
       * so we have a flag that helps us to avoid looking at the first pulse and
       * estimate over a long period, when it just started.
       * If it was a single pulse over some large period, we consider it as no flow */
      if (meter->getCount() == 1)
      {
        lastPulseTime = currentTime;
        noFlow = false;
        startFlow = true;
        return 0;
      }
      if (meter->getCount() == 0)
      {
        lastPulseTime = currentTime;
        return 0;
      }

      /* if there was more than one pulse, we assume the first as the "unknown",
       * so, it's deduced and the other are assumed to has happened since the last check */
      pulses = meter->getAndResetCount() - 1;
      noFlow = false;
    }
    else if (startFlow)
    {
      // estimate over more than a second
      if (pulseTime < 1000)
        return 0;
      if (meter->getCount() == 0)
      {
        // single pulse over a minute is considered as no flow
        if (pulseTime > 60000)
          noFlow = true;
        return 0;
      }
      pulses = meter->getAndResetCount();
      startFlow = false;
    }
    else
    {
      /* default case, flow already started
       * we want to round the value over some time to have more realistic average */
      if (pulseTime < 3000)
        return flowLPM;
       pulses = meter->getAndResetCount();
    }
    lastPulseTime = currentTime;

    flowLPM = pulses * 60 * 1000 / pulseTime;
    noFlow = pulses == 0;
    return flowLPM;
  }

  // we are using the stored value, instead of the parameter, the value is the same
  virtual String getText(uint32_t) override
  {
    char output[10] = {};
    snprintf(output, sizeof(output) - 1, "%2ld L/min", flowLPM);
    return output;
  }

private:
  WaterMeter *meter;
  uint64_t lastPulseTime;
  uint32_t flowLPM;
  bool noFlow, startFlow;
};

class Relay
{
public:
  Relay(int pin): pin(pin)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
protected:
  friend class PumpControl;

  void set(bool high)
  {
    digitalWrite(pin, high ? HIGH : LOW);
  }
private:
  const int pin;
};

class PumpControl
{
public:
  PumpControl(Display *display, Relay *relay): relay(relay)
  {
    pumpOnLine = new DisplayLine(display, 4);
    setOff();

    // Initialize buffer with zeros (pump "off" decisions)
    historyBufer.resize(WINDOW_SIZE, 0);
  }

  void registerSensor(SensorInterface *sensor)
  {
    sensors.push_back(sensor);
  }

  void loop(void)
  {
    std::vector<SensorDecisionTriState> decisions(sensors.size());
    int i = 0;

    for (auto &sensor : sensors)
    {
      uint32_t value = sensor->update_and_get();
      decisions[i++] = sensor->decide(value, pumpState);
    }

    if (std::any_of(decisions.begin(), decisions.end(),
        [](SensorDecisionTriState d) { return d == SensorDecisionTriState::STOP; }))
    {
      // if any sensor says to stop, we stop
      setOff();
    }
    else if (std::any_of(decisions.begin(), decisions.end(),
             [](SensorDecisionTriState d) { return d == SensorDecisionTriState::START; }))
    {
      // if any sensor says to start and no sensor says 'stop', we start
      setOn();
    }
    else
    {
      // all sensors say to stop
      setOff();
    }
  }
private:
  std::vector<SensorInterface*> sensors;
  DisplayLine *pumpOnLine;
  Relay *relay;

  static const int WINDOW_SIZE = 1800 * (1000 / DELAY_MS);  // 1/2 hour in loop instances
  static const int MIN_ON_TIME = 90000; // 90 seconds
  std::deque<int> historyBufer;       // Rolling window of decisions (0 or 1)
  bool pumpState;                      // Current pump state (true = on, false = off)
  uint64_t lastOnMs;                     // Time pump was last turned on

  // Calculate fraction of "on" decisions in history buffer
  double calculateHysteresisMetric() const {
      auto sum = std::accumulate(historyBufer.begin(), historyBufer.end(), 0);
      return static_cast<double>(sum) / historyBufer.size();
  }

  void setOn(void)
  {
    setPump(true);
  }
  void setOff(void)
  {
    setPump(false);
  }
  void setPump(bool on)
  {
    bool keepOn = update(on);
    relay->set(keepOn);
    // print spaces in order to clear the previous text
    pumpOnLine->setText("Pump  :", on ? "ON       " : (keepOn ? "ON (hist)" : "OFF      "));
  }

  bool update(bool decision, double threshold = 0.7)
  {
    historyBufer.pop_front();
    historyBufer.push_back(decision ? 1 : 0);

    // Stay on if decision is on
    if (decision)
    {
      if (!pumpState)
      {
        pumpState = true;
        lastOnMs = System.millis();
      }
      return true;
    }

    // Pump is off
    if (!pumpState)
      return false;

    // decision is off, but we need to check the hysteresis

    auto timeOn = System.millis() - lastOnMs;

    // Force pump to stay on until min time reached
    if (timeOn < MIN_ON_TIME)
      return true;

    // Turn off if decision is off and hysteresis metric is below threshold
    pumpState = calculateHysteresisMetric() >= threshold;
    return pumpState;
  }
};

Display *display;

CurrentSensor *currentSensor;
WaterMeter *reedSensor;
Relay *pumpRelay;

DisplayTime *displayTime;

PumpControl *pumpControl;

// setup() runs once, when the device is first turned on
void setup(void)
{
  Log.info("Setup..");
  Serial.begin(9600);

  currentSensor = new CurrentSensor();
  display = new Display(&epdDriver);

  // The relay controls the pump power
  pumpRelay = new Relay(S4);

  // Lower pull-up resistance is better
  reedSensor = new WaterMeter(D10);

  displayTime = new DisplayTime(display);
  pumpControl = new PumpControl(display, pumpRelay);

  pumpControl->registerSensor(new WaterLevel(display, 1, currentSensor));
  pumpControl->registerSensor(new Pressure(display, 2, currentSensor));
  pumpControl->registerSensor(new FlowRate(display, 3, reedSensor));
}

// loop() runs over and over again, as quickly as it can execute.
void loop(void)
{
  unsigned long long millis = System.millis();

  // TODO: display: Particle.connected();

  pumpControl->loop();
  displayTime->tick();
  display->tick();

  unsigned long long millis_diff = System.millis() - millis;
  if (millis_diff < DELAY_MS)
    delay(DELAY_MS - millis_diff); // milliseconds and blocking - see docs for more info!
}
