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
#include <vector>
#include <deque>
#include <numeric>
#include <algorithm>
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
    stringSizeX = line > 0 ?
      [](Screen_EPD *screen, String text) -> uint16_t { return screen->stringSizeX(text) * 2; } :
      [](Screen_EPD *screen, String text) -> uint16_t { return screen->stringSizeX(text); };
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

    // TODO: write main text once, if there's a param,
    // calculate the space of the max param, to be able to erase the rest

    gText(screen, x, y, text);
    if (param.length() > 0)
      gText(screen, x + stringSizeX(screen, text + " "), y, param);
  }

private:
  Display *display;
  const int line;

  String text, param;

  const uint16_t xOffset = 10;
  const uint16_t yOffset = 10;
  static constexpr uint16_t yPitch = 10;
  uint16_t x, y;

  using TextFunction = std::function<void(Screen_EPD *screen, uint16_t x0, uint16_t y0, String text)>;
  using SizeFunction = std::function<uint16_t(Screen_EPD *screen, String)>;
  TextFunction gText;
  SizeFunction stringSizeX;

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
      snprintf(str, 9, "%llu", sum / samples_count);
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
  static constexpr uint16_t zero_value = 6425;
  static constexpr uint64_t inc_nom = 643;
  static constexpr uint64_t inc_denom = 250;
  static constexpr uint64_t inc_output = 10000;
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
  WaterLevel(Display *display, unsigned screenLine, CurrentSensor *sensor, uint32_t calibrationMultiplier = 100)
  : SensorInterface(display, screenLine, "Level", "depth_mm")
    // hydrostatic pressure sensor connected to output 0,
    // range 0-10m (proximately), 0-1000 received in mbar
  , sensor(sensor, 0, 0, 1000)
  , calibrationMultiplier(calibrationMultiplier)
  { }

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
    uint32_t mbar = (sensor.read() * calibrationMultiplier + 50) / 100;
    return calc_in_mm(mbar);
  }

  uint32_t calc_in_mm(uint32_t mbar)
  {
    // Assuming water density of 0.9807 kg/L, 1mbar corresponds to 1.0197 mm of water column
    return (mbar * 1019 + 500) / 100;
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
  uint32_t calibrationMultiplier;
};

class Pressure: public SensorInterface
{
public:
  Pressure(Display *display, unsigned screenLine, CurrentSensor *sensor, int32_t calibrationAdditive = 0)
  : SensorInterface(display, screenLine, "Pressure", "pressure_mbar")
    // water pressure sensor connected to output 1,
    // range 0-6bar, 0-6000 received in mbar
  , sensor(sensor, 1, 0, 6000)
  , calibrationAdditive(calibrationAdditive)
   { }

  virtual SensorDecisionTriState decide(uint32_t pressure_mbar, bool lastPumpOn) override
  {
    /* Pressure below designed high pressure keeps the pump on,
     * the pump will stop when pressure is high and the flow is very low */
    if (lastPumpOn && pressure_mbar < PUMP_OFF_PRESSURE)
      return SensorDecisionTriState::START;

    /* The pump will start just when there could be no time to spool the pump, w/o experiencing it,
     * Otherwise high flow rate will start it anyway, if 3.5bar won't be enough, could be raised to 4bar */
    if (!lastPumpOn && pressure_mbar < PUMP_ON_PRESSURE)
      return SensorDecisionTriState::START;

    /* There's a protection of high pressure, stop the pump in order to avoid damage, for example
     * the installed membrane tank max pressure could be as low as 8.6 bar */
    if (pressure_mbar > PUMP_STOP_PRESSURE)
      return SensorDecisionTriState::STOP;

    return SensorDecisionTriState::OK_TO_STOP;
}

protected:
  uint32_t readValue(void) override
  {
    int32_t calibrated = static_cast<int32_t>(sensor.read()) + calibrationAdditive;
    // Clamp: a raw reading below the calibration offset (sensor noise near zero or
    // a disconnected sensor) would otherwise underflow to a huge value and trip the
    // emergency high-pressure STOP.
    if (calibrated < 0)
      return 0;
    return static_cast<uint32_t>(calibrated);
  }

  virtual String getText(uint32_t mbar) override
  {
    char output[16] = {};
    snprintf(output, sizeof(output) - 1, "%01lu.%02lu bar", mbar / 1000, (mbar/10) % 100);
    return output;
  }

private:
  static constexpr int PUMP_ON_PRESSURE = 2500; // cut-in pressure in mbar
  static constexpr int PUMP_OFF_PRESSURE = 4000; // cut-out pressure in mbar
  static constexpr int PUMP_STOP_PRESSURE = 5500; // emergency stop pressure in mbar

  CurrentSensorChannel sensor;
  int32_t calibrationAdditive;
};

class WaterMeter
{
public:
    WaterMeter(int pin, Display *display)
        : pin(pin), pulseCount(0), lastPulseTime(0), totalEdges(0)
    {
        pinMode(pin, INPUT_PULLUP);
        pinSetDriveStrength(pin, DriveStrength::HIGH);
        attachInterrupt(pin, [this]() { handleInterrupt(); }, FALLING);
        statusLine = new DisplayLine(display, -3);
    }

    uint32_t getAndResetCount() {
        String debug = "Reed - P:" + String(getCount()) +
                       " E:" + String(getTotalEdges()) +
                       " Pin:" + (getRawPinState() ? "L" : "H") +
                       " LP:" + String(lowPulse.load(std::memory_order_relaxed)) + "ms";

        statusLine->setText(debug);
        lowPulse.store(0, std::memory_order_relaxed);

      return pulseCount.exchange(0, std::memory_order_relaxed);
    }
    uint32_t getCount() const { return pulseCount.load(); }

    uint32_t getTotalEdges() { return totalEdges.exchange(0); }
    bool     getRawPinState()  const { return digitalRead(pin) == LOW; }

private:
    static constexpr uint32_t DEBOUNCE_MS = 25;   // start very low

    const int pin;
    DisplayLine *statusLine;
    std::atomic<uint32_t> pulseCount{0};
    std::atomic<uint32_t> lastPulseTime{0};
    std::atomic<uint32_t> totalEdges{0};
    std::atomic<uint32_t> lowPulse{0};

    void handleInterrupt()
    {
        totalEdges.fetch_add(1, std::memory_order_relaxed);

        uint32_t now = System.millis();
        auto lastTime = lastPulseTime.load(std::memory_order_relaxed);
        if (now - lastTime < DEBOUNCE_MS)
        {
            lowPulse.store(now - lastTime, std::memory_order_relaxed);
            return;
        }

        lastPulseTime.store(now, std::memory_order_relaxed);
        pulseCount.fetch_add(1, std::memory_order_relaxed);
    }
};

class FlowRate : public SensorInterface
{
public:
    FlowRate(Display *display, unsigned screenLine, WaterMeter *meter)
        : SensorInterface(display, screenLine, "Flow", "flow_lpm")
        , meter(meter)
        , lastPulseTime(0)
        , flowLPM(0.0f)
        , noFlow(true)
    {
        meter->getAndResetCount();
    }

    virtual SensorDecisionTriState decide(uint32_t currentFlowLPM, bool lastPumpOn) override
    {
      /* Flow rate starts the pump, immediately after there's a significant flow,
       * the pump is expected to stop, when the flow is low and the pressure has built up */
      if (flowLPM > 12)
        return SensorDecisionTriState::START;
      return SensorDecisionTriState::OK_TO_STOP;
    }

protected:
    uint32_t readValue() override
    {
        constexpr uint32_t PULSES_PER_LITER = 10;
        constexpr float    SMOOTH_ALPHA     = 0.65f;
        constexpr uint32_t NO_FLOW_TIMEOUT_MS = 5000;

        uint32_t now = System.millis();
        uint32_t pulses = meter->getAndResetCount();

        if (pulses == 0)
        {
            if (noFlow)
                return (uint32_t)(flowLPM * 10);   // keep last value for display

            if ((now - lastPulseTime) > NO_FLOW_TIMEOUT_MS)
            {
                noFlow = true;
                flowLPM = 0.0f;
                return 0;
            }

            flowLPM *= 0.88f;   // gentle decay
            return (uint32_t)(flowLPM * 10);
        }

        // New pulses arrived
        uint32_t deltaMs = (lastPulseTime == 0) ? 1000 : (now - lastPulseTime);
        lastPulseTime = now;

        if (noFlow)
        {
            noFlow = false;
            return 0;   // wait for second pulse
        }

        // Calculate flow
        float liters = (float)pulses / PULSES_PER_LITER;
        float instantLPM = (60000.0f * liters) / deltaMs;

        // Smoothing
        flowLPM = instantLPM * SMOOTH_ALPHA + flowLPM * (1.0f - SMOOTH_ALPHA);

        return (uint32_t)(flowLPM * 10);   // return value ×10 for 1 decimal place
    }

    virtual String getText(uint32_t value) override
    {
        char buf[12];
        float displayFlow = value / 10.0f;
        snprintf(buf, sizeof(buf), "%4.1f L/min", displayFlow);
        return buf;
    }

private:
    WaterMeter *meter;
    uint64_t lastPulseTime;
    float    flowLPM;
    bool     noFlow;
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
    statusLine = new DisplayLine(display, -2);
    setOff();

    // Initialize buffer with zeros (no starts yet)
    pumpOnInHourBuffer.resize(HOUR_WINDOW_SIZE, 0);
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
      // if any sensor says to stop, we stop, immediately
      forceOff();
    }
    else if (std::any_of(decisions.begin(), decisions.end(),
             [](SensorDecisionTriState d) { return d == SensorDecisionTriState::START; }))
    {
      // if any sensor says to start and no sensor says 'stop', we start
      setOn();
    }
    else
    {
      // no sensor says to start or stop, so we can stop, ...
      setOff();
    }
  }
private:
  std::vector<SensorInterface*> sensors;
  DisplayLine *pumpOnLine;
  DisplayLine *statusLine;
  Relay *relay;

  static constexpr int HOUR_WINDOW_SIZE = 3600 * (1000 / DELAY_MS);
  static constexpr int MIN_ON_TIME = 45000; // minimum run time of 45 seconds

  /* Anti-short-cycle hysteresis (graduated, recency-weighted).
   *
   * Goal: keep pump starts at/under ~20 per hour without ever withholding water.
   * We keep a rolling one-hour history of starts and score it with a recency
   * weight -- a start "now" counts ~1, one an hour ago counts ~0. The more (and
   * the more recently) the pump has been starting, the longer we are willing to
   * "bridge" a gap by keeping an already-running pump on past the point it would
   * otherwise stop:
   *
   *   extra  = weightedRecentStarts - HYST_BRIDGE_FLOOR
   *   bridge = clamp(extra * HYST_MS_PER_START, 0, HYST_MAX_BRIDGE_MS)
   *
   * So an isolated long run (a single recent start) scores ~0 and is never bridged
   * -- it just stops -- while genuine short-cycling earns up to HYST_MAX_BRIDGE_MS
   * of extension. Real demand always keeps the pump on via decide()==START, and
   * the 5.5 bar emergency stop remains the hard pressure backstop. */
  static constexpr uint64_t HYST_MAX_BRIDGE_MS = 120000; // max bridge (extension) time, 2 min
  static constexpr double   HYST_BRIDGE_FLOOR  = 3.0;    // recency-weighted starts below this -> no bridging
  static constexpr uint64_t HYST_MS_PER_START  = 20000;  // bridge time earned per weighted start above the floor

  std::deque<int> pumpOnInHourBuffer;   // Hour long rolling window of starts (1 per off->on edge)
  bool pumpState = false;               // Current pump state (true = on, false = off)
  uint64_t lastOnMs = 0;                // Time pump was last turned on
  uint64_t bridgeSinceMs = 0;           // When the current bridge/extension started (0 = not bridging)
  int startsLastHour = 0;               // Unweighted starts in the trailing hour (for display)

  // Recency-weighted count of starts in the trailing hour: a start "now" weighs
  // ~1, one an hour ago ~0. Captures whether the pump is short-cycling *recently*.
  double weightedRecentStarts(void) const
  {
    size_t n = pumpOnInHourBuffer.size();
    if (n < 2)
      return 0.0;

    double sum = 0.0;
    size_t idx = 0;
    for (int started : pumpOnInHourBuffer)
    {
      if (started)
        sum += static_cast<double>(idx) / (n - 1); // front = oldest (~0), back = newest (~1)
      ++idx;
    }
    return sum;
  }

  // How long we're currently willing to keep an already-running pump on past its
  // natural stop, scaled by how much (and how recently) it has been short-cycling.
  uint64_t allowedBridgeMs(void) const
  {
    double extra = weightedRecentStarts() - HYST_BRIDGE_FLOOR;
    if (extra <= 0.0)
      return 0;
    uint64_t ms = static_cast<uint64_t>(extra * HYST_MS_PER_START);
    return ms < HYST_MAX_BRIDGE_MS ? ms : HYST_MAX_BRIDGE_MS;
  }

  void setOn(void)
  {
    setPump(true);
  }

  void setOff(void)
  {
    setPump(false);
  }

  void forceOff(void)
  {
    setPump(false, true);
  }

  void countOnTimes(void)
  {
    static bool pumpLastOn = false;
    static uint16_t lastRunTimeSecs = 0;
    char output[50] = {};
    bool turnedOn = pumpState && !pumpLastOn;

    pumpOnInHourBuffer.pop_front();
    pumpOnInHourBuffer.push_back(turnedOn ? 1 : 0);
    // the run time will be updated as long as the pump is on
    if (pumpLastOn)
    {
      // pumpState is updated on the next cycle, but since we truncate it, keep the calc like this
      lastRunTimeSecs = (System.millis() - lastOnMs) / 1000;
    }
    pumpLastOn = pumpState;

    startsLastHour = std::accumulate(pumpOnInHourBuffer.begin(), pumpOnInHourBuffer.end(), 0);
    snprintf(output, sizeof(output) - 1, "ON counter: %2d, run: %3d:%02d",
                                         startsLastHour, lastRunTimeSecs / 60, lastRunTimeSecs % 60);
    statusLine->setText(output);
  }

  void setPump(bool on, bool force=false)
  {
    bool keepOn;
    String onReason = "time";

    countOnTimes();

    if (force)
      keepOn = on;
    else
      keepOn = decideKeepOn(on);

    if (!force && !keepOn && pumpState)
    {
      // Raw decision is OFF, the minimum run time is satisfied, and the pump is
      // currently running. Bridge the gap only as long as recent short-cycling
      // justifies -- a graduated extension (see allowedBridgeMs) measured from the
      // moment the pump would have stopped, capped at HYST_MAX_BRIDGE_MS. An
      // isolated long run earns ~no bridge and stops. Only ever extends an
      // already-running pump; real restarts come from decide()==START.
      if (bridgeSinceMs == 0)
        bridgeSinceMs = System.millis();

      if ((System.millis() - bridgeSinceMs) < allowedBridgeMs())
      {
        keepOn = true;
        onReason = "hist";
      }
    }

    // Reset the bridge timer when real demand returns, or once the pump stops.
    if (on || !keepOn)
      bridgeSinceMs = 0;

    relay->set(keepOn);
    pumpState = keepOn;
    // print spaces in order to clear the previous text
    pumpOnLine->setText("Pump  :", on ? "ON      " : (keepOn ? "ON/ " + onReason : "OFF     "));
  }

  bool decideKeepOn(bool decision)
  {
    // Stay on if decision is on
    if (decision)
    {
      if (!pumpState)
        lastOnMs = System.millis();
      return true;
    }

    // Pump is off
    if (!pumpState)
      return false;

    // decision is off, but there's a minimum running time we should comply
    auto timeOn = System.millis() - lastOnMs;

    // Force pump to stay on until min time reached
    return timeOn < MIN_ON_TIME;
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

  WiFi.selectAntenna(ANT_EXTERNAL);

  currentSensor = new CurrentSensor();
  display = new Display(&epdDriver);

  // The relay controls the pump power
  pumpRelay = new Relay(S4);

  // Lower pull-up resistance is better
  reedSensor = new WaterMeter(D10, display);

  displayTime = new DisplayTime(display);
  pumpControl = new PumpControl(display, pumpRelay);

  pumpControl->registerSensor(new WaterLevel(display, 1, currentSensor, 117)); // calibrated to the reading on the display, 100 is 1
  pumpControl->registerSensor(new Pressure(display, 2, currentSensor, -130)); // calibrated to -0.13 bar offset
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
