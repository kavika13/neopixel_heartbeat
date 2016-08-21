#include <avr/pgmspace.h>
#include <FastLED.h>
#include <assert.h>

// ------- Undefine macros that will conflict with type definitions
#pragma push_macro("min")
#undef min

#pragma push_macro("max")
#undef max

// ------- Subset of algorithms, ala <algorithm>
namespace std {

template<typename T> const T& min(const T& a, const T& b) {
  return (b < a) ? b : a;
}

template<typename T> const T& max(const T& a, const T& b) {
  return (a < b) ? b : a;
}

template<typename T> constexpr const T& clamp(const T& value, const T& min_value, const T& max_value) {
  return assert(!(max_value < min_value)), std::max(min_value, std::min(max_value, value));
}

};

// ------- Global data
#define DATA_PIN 4
#define NUM_LEDS 60
#define BRIGHTNESS (128 - 1)

// ------- Heartbeat effect

//#define HEARTBEAT_DISABLE_FLASH

const CHSV backgroundCalmColor(230, 200, 182);
const CHSV backgroundExcitedColor(190, 200, 236);
const CHSV backgroundIntenseColor(190, 200, 236);

struct HeartbeatData {
  HeartbeatData()
    : updateTime(0)
    , animationTime(0)
    , animationOffset(0)
    , isStressed(false)
    , stress(0)
    , stressCounter(0)
    , timeSinceLastStress(0) {
  }

  unsigned long updateTime;
  unsigned long animationTime;
  unsigned long animationOffset;
  bool isStressed;
  uint16_t stress;
  uint32_t stressCounter;
  unsigned long timeSinceLastStress;
  unsigned long timeCurrentStress;
};

// Roughly approximate a heart beat:
// https://en.wikipedia.org/wiki/QT_interval
// https://meds.queensu.ca/central/assets/modules/ECG/normal_ecg.html
// R = 0-45 ms, 2.7 at peak (modeled with sine curve, zero-to-zero)
// T = 120-440 ms, 0.9 at peak (modeled with sine curve, bottom-to-top-to-bottom)

#define HEARTBEAT_OFFSET_R 0
#define HEARTBEAT_DURATION_R 200
#define HEARTBEAT_AMPLITUDE_R 2.7f
#define HEARTBEAT_WAVE_OFFSET_R 0.0f
#define HEARTBEAT_WAVE_DURATION_R PI

#define HEARTBEAT_OFFSET_T (250 - 1)
#define HEARTBEAT_DURATION_T 300
#define HEARTBEAT_AMPLITUDE_T 1.7f
#define HEARTBEAT_WAVE_OFFSET_T (PI * 1.5)
#define HEARTBEAT_WAVE_DURATION_T (PI * 2)

#define HEARTBEAT_WAVE_SCALE_T (HEARTBEAT_AMPLITUDE_T / HEARTBEAT_AMPLITUDE_R)

#define HEARTBEAT_UPDATES_PER_SECOND 50.0f
#define HEARTBEAT_DURATION_UPDATE (1000.0f / HEARTBEAT_UPDATES_PER_SECOND)
#define HEARTBEAT_DURATION_MAX_STRESS_CHANGE (2.0f * 1000.0f)
#define HEARTBEAT_NUM_UPDATES_MAX_STRESS_CHANGE (HEARTBEAT_DURATION_MAX_STRESS_CHANGE / HEARTBEAT_DURATION_UPDATE)
#define HEARTBEAT_STRESS_MAX_VELOCITY static_cast<uint16_t>(65535.0f / HEARTBEAT_NUM_UPDATES_MAX_STRESS_CHANGE)

// TODO: The noise is periodic based on the update frequency - somehow reflect this
#define HEARTBEAT_NOISE_COUNTER_INCREMENT 512

// Prototypes necessary to work around bug: https://github.com/arduino/arduino-builder/issues/170
constexpr float heartbeatWaveR(unsigned long milliseconds);
constexpr float heartbeatWaveR(unsigned long milliseconds) {
  return sin(
    static_cast<float>(milliseconds - HEARTBEAT_OFFSET_R) / HEARTBEAT_DURATION_R
    * HEARTBEAT_WAVE_DURATION_R
    + HEARTBEAT_WAVE_OFFSET_R);
}

constexpr float heartbeatWaveT(unsigned long milliseconds);
constexpr float heartbeatWaveT(unsigned long milliseconds) {
  return (
    HEARTBEAT_WAVE_SCALE_T
    * (sin(
        static_cast<float>(milliseconds - HEARTBEAT_OFFSET_T) / HEARTBEAT_DURATION_T
        * HEARTBEAT_WAVE_DURATION_T
        + HEARTBEAT_WAVE_OFFSET_T)
      + 1.0f)
    / 2.0f);
}

template<typename T, typename U> constexpr bool isWithinDuration(T value, U duration, U offset = 0);
template<typename T, typename U> constexpr bool isWithinDuration(T value, U duration, U offset = 0) {
  return value >= offset && value - offset < duration;
}

constexpr uint8_t heartbeatPulseAtTime(unsigned long milliseconds);
constexpr uint8_t heartbeatPulseAtTime(unsigned long milliseconds) {
  return static_cast<uint8_t>(
    255.0f
    * (isWithinDuration(milliseconds, HEARTBEAT_DURATION_R, HEARTBEAT_OFFSET_R)
        ? heartbeatWaveR(milliseconds)
        : (isWithinDuration(milliseconds, HEARTBEAT_DURATION_T, HEARTBEAT_OFFSET_T)
            ? heartbeatWaveT(milliseconds)
            : 0.0f)));
}

// Helpers to statically generate our tables
#define S4(i)    S1((i)),   S1((i)+1),     S1((i)+2),     S1((i)+3)
#define S8(i)    S4((i)),   S4((i)+4)
#define S16(i)   S8((i)),   S8((i)+8)
#define S32(i)   S16((i)),  S16((i)+16)
#define S64(i)   S32((i)),  S32((i)+32)
#define S128(i)  S64((i)),  S64((i)+64)
#define S256(i)  S128((i)), S128((i)+128)
#define S512(i)  S256((i)), S256((i)+256)
#define S1024(i)  S512((i)), S512((i)+512)

const uint8_t heartbeatPulse[] PROGMEM = {
#define S1(milliseconds) (heartbeatPulseAtTime(milliseconds))
  S1024(0)
#undef S1
};

CRGB heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long animationTime);
CRGB heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long animationTime) {
  unsigned long animationIndex = animationTime + currentPixelIndex;

  if (animationIndex < 1024) {
    uint8_t value = pgm_read_byte_near(heartbeatPulse + animationIndex);
    return CRGB(value, value, value);
  } else {
    return CRGB(0, 0, 0);
  }
}

void updateStress(HeartbeatData& data) {
  if (data.isStressed) {
    data.timeCurrentStress += HEARTBEAT_DURATION_UPDATE;

    uint16_t noise = inoise16(data.stressCounter);
    data.stressCounter += HEARTBEAT_NOISE_COUNTER_INCREMENT;

    int16_t stressVelocity = static_cast<int16_t>(scale16(noise, HEARTBEAT_STRESS_MAX_VELOCITY * 2))
      - HEARTBEAT_STRESS_MAX_VELOCITY;
    Serial.println(HEARTBEAT_STRESS_MAX_VELOCITY);

    int32_t newStress = data.stress;
    newStress += stressVelocity;

    newStress = std::clamp(
        newStress,
        static_cast<int32_t>(0),
        static_cast<int32_t>(65535));

    newStress -= scale16(256, newStress);

    if (data.timeCurrentStress > 6000) {  // TODO: Make constant
      newStress -= 512;      
    }

    if (newStress < 0) {
      newStress = 0;
    }

    if (data.timeCurrentStress > 8000) {  // TODO: Make constant
      data.isStressed = false;
      data.timeCurrentStress = 0;
      data.timeSinceLastStress = 0;
    }

    data.stress = newStress;
  } else {
    data.timeSinceLastStress += HEARTBEAT_DURATION_UPDATE;

    if (data.timeSinceLastStress > 15000) {  // TODO: Make constant
      data.isStressed = true;
      data.timeCurrentStress = 0;
      data.timeSinceLastStress = 0;
    }
  }
}

void heartbeatPattern(CRGB (&strip)[NUM_LEDS], const unsigned long deltaTime, HeartbeatData& data) {
  data.updateTime += deltaTime;

  if(data.updateTime > HEARTBEAT_DURATION_UPDATE) {
    data.updateTime -= HEARTBEAT_DURATION_UPDATE;
    updateStress(data);
  }

  // Integer version of: deltaTime * ([0.0, 2.0] + 1)
  data.animationTime += deltaTime * ((static_cast<uint32_t>(data.stress) + 1) * 1.3 + 65536) / 65536;
  // Using 1024ms per heartbeat because the compiler turns % 1024 into bitwise math instead of a divide
  data.animationTime %= 2048;

  // Double gradient
  CRGB backgroundColor = data.stress < 32768
    ? blend(backgroundCalmColor, backgroundExcitedColor, data.stress * 2 / 256)
    : blend(backgroundExcitedColor, backgroundIntenseColor, (data.stress - 32768) * 2 / 256);

  for (long currentPixelIndex = 0; currentPixelIndex < NUM_LEDS; ++currentPixelIndex) {
    CRGB pulseColor(heartbeatPixelColor(currentPixelIndex, data.animationTime + data.animationOffset));
    pulseColor.fadeToBlackBy(dim8_raw(222));

#ifdef HEARTBEAT_DISABLE_FLASH
    pulseColor = CRGB::Black;
#endif

    CRGB pixelColor(pulseColor + backgroundColor);

    strip[currentPixelIndex] = pixelColor;
  }
}

void setupHeartbeatPattern(const unsigned long offsetTime, HeartbeatData& data) {
  data.updateTime += offsetTime;

  while (data.updateTime > HEARTBEAT_DURATION_UPDATE) {
    data.updateTime -= HEARTBEAT_DURATION_UPDATE;
    updateStress(data);
  }
}

// ------- Main program
CRGB strip1[NUM_LEDS];
CRGB strip2[NUM_LEDS];
CRGB strip3[NUM_LEDS];
CRGB strip4[NUM_LEDS];
CRGB strip5[NUM_LEDS];
HeartbeatData heartbeatStripData1;
HeartbeatData heartbeatStripData2;
HeartbeatData heartbeatStripData3;
HeartbeatData heartbeatStripData4;
HeartbeatData heartbeatStripData5;

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(strip1, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN + 1>(strip2, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN + 2>(strip3, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN + 3>(strip4, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN + 4>(strip5, NUM_LEDS);

  setupHeartbeatPattern(2200, heartbeatStripData1);
  setupHeartbeatPattern(5800, heartbeatStripData2);
  setupHeartbeatPattern(8400, heartbeatStripData3);
  setupHeartbeatPattern(1700, heartbeatStripData4);
  setupHeartbeatPattern(11900, heartbeatStripData5);

  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  const unsigned long currentTime = millis();
  static unsigned long lastTime = currentTime;
  unsigned long deltaTime = currentTime - lastTime;

  heartbeatPattern(strip1, deltaTime, heartbeatStripData1);
  heartbeatPattern(strip2, deltaTime, heartbeatStripData2);
  heartbeatPattern(strip3, deltaTime, heartbeatStripData3);
  heartbeatPattern(strip4, deltaTime, heartbeatStripData4);
  heartbeatPattern(strip5, deltaTime, heartbeatStripData5);

  FastLED.show();

  lastTime = currentTime;

  FastLED.countFPS();
  static unsigned long fpsTime = 0;
  fpsTime += deltaTime;

  if(fpsTime > 1000) {
    fpsTime -= 1000;
    Serial.print("heartbeatPattern FPS: ");
    Serial.println(FastLED.getFPS());
  }
}

