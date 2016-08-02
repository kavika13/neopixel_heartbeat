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
#define DATA_PIN 7
#define NUM_LEDS 59
#define BRIGHTNESS (64 - 1)

// ------- Heartbeat effect

const CHSV backgroundCalmColor(95, 209, 182);
const CHSV backgroundExcitedColor(254, 92, 236);
const CHSV backgroundIntenseColor(222, 209, 127);

// Roughly approximate a heart beat:
// https://en.wikipedia.org/wiki/QT_interval
// https://meds.queensu.ca/central/assets/modules/ECG/normal_ecg.html
// R = 0-45 ms, 2.7 at peak (modeled with sine curve, zero-to-zero)
// T = 120-440 ms, 0.9 at peak (modeled with sine curve, bottom-to-top-to-bottom)

#define HEARTBEAT_OFFSET_R 0
#define HEARTBEAT_DURATION_R 45
#define HEARTBEAT_AMPLITUDE_R 2.7f
#define HEARTBEAT_WAVE_OFFSET_R 0.0f
#define HEARTBEAT_WAVE_DURATION_R PI

#define HEARTBEAT_OFFSET_T (120 - 1)
#define HEARTBEAT_DURATION_T 320
#define HEARTBEAT_AMPLITUDE_T 0.6f
#define HEARTBEAT_WAVE_OFFSET_T (PI * 1.5)
#define HEARTBEAT_WAVE_DURATION_T (PI * 2)

#define HEARTBEAT_WAVE_SCALE_T (HEARTBEAT_AMPLITUDE_T / HEARTBEAT_AMPLITUDE_R)

#define HEARTBEAT_UPDATES_PER_SECOND 50.0f
#define HEARTBEAT_DURATION_UPDATE (3.0f * 1000.0f / HEARTBEAT_UPDATES_PER_SECOND)
#define HEARTBEAT_DURATION_MAX_STRESS_CHANGE (1.0f * 1000.0f)
#define HEARTBEAT_NUM_UPDATES_MAX_STRESS_CHANGE (HEARTBEAT_DURATION_MAX_STRESS_CHANGE / HEARTBEAT_DURATION_UPDATE)
#define HEARTBEAT_SCALED_STRESS_MAX_VELOCITY static_cast<uint16_t>(65535.0f / HEARTBEAT_NUM_UPDATES_MAX_STRESS_CHANGE)
#define HEARTBEAT_SCALED_STRESS_MAX_IMPULSE 128

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
  // Using 1024ms per heartbeat because the compiler turns % 1024 into bitwise math instead of a divide
  uint8_t value = pgm_read_byte_near(heartbeatPulse + (animationTime + currentPixelIndex) % 1024);
  return CRGB(value, value, value);
}

void updateStress(uint8_t& stress) {
  static int16_t stressVelocity = 0;
  int16_t stressAcceleration = random(HEARTBEAT_SCALED_STRESS_MAX_IMPULSE * 2 + 1)
    - HEARTBEAT_SCALED_STRESS_MAX_IMPULSE;

  stressVelocity += stressAcceleration;
  stressVelocity = std::clamp(
    stressVelocity,
    static_cast<int16_t>(-HEARTBEAT_SCALED_STRESS_MAX_VELOCITY),
    static_cast<int16_t>(HEARTBEAT_SCALED_STRESS_MAX_VELOCITY));

  stress = std::clamp(
    (static_cast<int32_t>(stress) * 256 + stressVelocity) / 256,
    static_cast<int32_t>(0),
    static_cast<int32_t>(255));

  // If stress got clamped, reduce velocity. Greatly reduces top/bottom stickiness
  if((stress == 255 && stressVelocity > 0) || (stress == 0 && stressVelocity < 0)) {
    stressVelocity /= 2;
  }
}

void heartbeatPattern(CRGB (&strip)[NUM_LEDS], const unsigned long currentTime, unsigned long timeOffset = 0) {
  static unsigned long lastTime = currentTime;
  static unsigned long updateTime = 0;
  static unsigned long animationTime = 0;
  static unsigned long fpsTime = 0;

  unsigned long deltaTime = currentTime - lastTime;
  updateTime += deltaTime;
  fpsTime += deltaTime;
  lastTime = currentTime;

  static uint8_t stress = 0;

  if(updateTime > HEARTBEAT_DURATION_UPDATE) {
    updateTime -= HEARTBEAT_DURATION_UPDATE;
    updateStress(stress);
  }

  if(fpsTime > 1000) {
    fpsTime -= 1000;
    Serial.print("heartbeatPattern FPS: ");
    Serial.println(FastLED.getFPS());
  }

  // Integer version of: deltaTime * ([0.0, 2.0] + 1)
  animationTime += (deltaTime * ((stress + 1) * 2 + 256) * 256) / 65536;
  // Using 1024ms per heartbeat because the compiler turns % 1024 into bitwise math instead of a divide
  animationTime %= 1024;

  // Double gradient
  CRGB backgroundColor = stress < 128
    ? blend(backgroundCalmColor, backgroundExcitedColor, stress * 2)
    : blend(backgroundExcitedColor, backgroundIntenseColor, (stress - 128) * 2);

  for(long currentPixelIndex = 0; currentPixelIndex < NUM_LEDS; ++currentPixelIndex) {
    CRGB pulseColor(heartbeatPixelColor(currentPixelIndex, animationTime + timeOffset));
    pulseColor.fadeToBlackBy(dim8_raw(192));

    CRGB pixelColor(pulseColor + backgroundColor);

    strip[currentPixelIndex] = pixelColor;
  }
}

// ------- Main program
CRGB strip[NUM_LEDS];

void setup() {
  randomSeed(analogRead(0));

  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(strip, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  unsigned long currentTime = millis();

  heartbeatPattern(strip, currentTime);

  FastLED.countFPS();
  FastLED.show();
}

