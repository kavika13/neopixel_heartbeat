#include <avr/pgmspace.h>
#include <Adafruit_NeoPixel.h>
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

// ------- Define Color class
typedef uint8_t ColorComponent;

struct Color3 {
  constexpr Color3() : data { 0, 0, 0 } {}
  constexpr Color3(ColorComponent red, ColorComponent green, ColorComponent blue) : data { red, green, blue } {}
  explicit constexpr Color3(ColorComponent component) : data { component, component, component } {}

  static const Color3 White;
  static const Color3 Black;
  static const Color3 Red;
  static const Color3 Green;
  static const Color3 Blue;
  static const Color3 Magenta;
  static const Color3 Yellow;
  static const Color3 Cyan;

  bool operator==(const Color3 other) {
    return r == other.r && g == other.g && b == other.b;
  }

  bool operator!=(const Color3 other) {
    return !(*this == other);
  }

  bool operator<(const Color3& other) {
    return r < other.r && g < other.g && b < other.b;
  }

  bool operator>(const Color3& other) {
    return *this < other;
  }

  bool operator<=(const Color3& other) {
    return !(*this > other);
  }

  bool operator>=(const Color3& other) {
    return !(*this < other);
  }

  ColorComponent& operator[](size_t index) {
    return data[index];
  }

  const ColorComponent& operator[](size_t index) const {
    return data[index];
  }

  const Color3& operator+=(const Color3& other) {
    for(size_t i = 0; i < 3; ++i) {
      (*this)[i] = std::min(static_cast<uint16_t>((*this)[i]) + other[i], (uint16_t)255);
    }
    return *this;
  }

  const Color3& operator+=(ColorComponent component) {
    for(size_t i = 0; i < 3; ++i) {
      (*this)[i] = std::min(static_cast<uint16_t>((*this)[i]) + component, (uint16_t)255);
    }
    return *this;
  }

  const Color3& operator-=(const Color3& other) {
    for(size_t i = 0; i < 3; ++i) {
      (*this)[i] = std::max(static_cast<int16_t>((*this)[i]) - other[i], 0);
    }
    return *this;
  }

  const Color3& operator-=(ColorComponent component) {
    for(size_t i = 0; i < 3; ++i) {
      (*this)[i] = std::max(static_cast<int16_t>((*this)[i]) - component, 0);
    }
    return *this;
  }

  const Color3& operator*=(const Color3& other) {
    for(size_t i = 0; i < 3; ++i) {
      // Shifting math so the compiler can turn the divide into a bit-shift
      // This saves ~0.018ms per, 43.6% faster - on Arduino Nano
      uint32_t lhs = static_cast<uint32_t>((*this)[i]) + 1;
      uint32_t rhs = static_cast<uint32_t>(other[i]) + 1;
      uint32_t result = lhs * rhs / 256;
      result -= (result ? 1 : 0);
      (*this)[i] = result;
    }
    return *this;
  }

  const Color3& operator*=(uint8_t value) {
    for(size_t i = 0; i < 3; ++i) {
      // Shifting math so the compiler can turn the divide into a bit-shift
      // This saves ~0.019ms per, 47.2% faster - on Arduino Nano
      uint32_t lhs = static_cast<uint32_t>((*this)[i]) + 1;
      uint32_t rhs = static_cast<uint32_t>(value) + 1;
      uint32_t result = lhs * rhs / 256;
      result -= (result ? 1 : 0);
      (*this)[i] = result;
    }
    return *this;
  }

  // TODO: Does division by another color make any sense?

  const Color3& operator/=(uint8_t value) {
    for(size_t i = 0; i < 3; ++i) {
      (*this)[i] /= value;
    }
    return *this;
  }

  friend Color3 operator+(Color3 lhs, const Color3& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Color3 operator+(Color3 lhs, ColorComponent component) {
    lhs += component;
    return lhs;
  }

  friend Color3 operator+(ColorComponent component, Color3 rhs) {
    rhs += component;
    return rhs;
  }

  friend Color3 operator-(Color3 lhs, const Color3& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend Color3 operator-(Color3 lhs, ColorComponent component) {
    lhs -= component;
    return lhs;
  }

  friend Color3 operator-(ColorComponent component, const Color3& rhs) {
    return Color3(component) -= rhs;
  }

  friend Color3 operator*(Color3 lhs, const Color3& rhs) {
    lhs *= rhs;
    return lhs;
  }

  friend Color3 operator*(Color3 lhs, uint8_t value) {
    lhs *= value;
    return lhs;
  }

  friend Color3 operator*(uint8_t value, Color3 rhs) {
    rhs *= value;
    return rhs;
  }

  // TODO: Does division by another color make any sense?

  friend Color3 operator/(Color3 lhs, uint8_t value) {
    lhs /= value;
    return lhs;
  }

  // TODO: Does division by another color make any sense?
  //       For value divided by a vector case, w/ value -> vector promotion

  friend Color3 lerp(Color3 lhs, const Color3& rhs, uint8_t weight) {
    for(size_t i = 0; i < 3; ++i) {
      lhs[i] = Color3::lerpImpl(lhs[i], rhs[i], weight);
    }
    return lhs;
  }

  friend Color3 lerp(Color3 lhs, const Color3& rhs, const Color3& weight) {
    for(size_t i = 0; i < 3; ++i) {
      lhs[i] = Color3::lerpImpl(lhs[i], rhs[i], weight[i]);
    }
    return lhs;
  }

  friend Color3 mix(const Color3& lhs, const Color3& rhs, ColorComponent weight) {
    return lerp(lhs, rhs, weight);
  }

  friend Color3 mix(Color3 lhs, const Color3& rhs, const Color3& weight) {
    return lerp(lhs, rhs, weight);
  }

  union {
    struct { uint8_t data[3]; };
    struct { uint8_t r, g, b; };
  };

private:
  static uint8_t lerpImpl(uint32_t lhs, uint32_t rhs, uint16_t weight) {
    // Shifting math so the compiler can turn the divide into a bit-shift
    // This saves ~0.037ms per, 42.5% faster - on Arduino Nano
    ++weight;

    ++lhs;
    lhs *= 256 + 1 - weight;
    lhs /= 256;
    if(lhs) {
      --lhs;
    }

    ++rhs;
    rhs *= weight;
    rhs /= 256;
    if(rhs) {
      --rhs;
    }
 
    return lhs + rhs;
  }
};

const Color3 Color3::White = Color3(255, 255, 255);
const Color3 Color3::Black = Color3(0, 0, 0);
const Color3 Color3::Red = Color3(255, 0, 0);
const Color3 Color3::Green = Color3(0, 255, 0);
const Color3 Color3::Blue = Color3(0, 0, 255);
const Color3 Color3::Magenta = Color3(255, 0, 255);
const Color3 Color3::Yellow = Color3(255, 255, 0);
const Color3 Color3::Cyan = Color3(0, 255, 255);

// ------- Global data
#define DATA_PIN 7
#define NUM_LEDS 59
#define BRIGHTNESS (64 - 1)

// ------- Heartbeat effect

// Roughly approximate a heart beat:
// https://en.wikipedia.org/wiki/QT_interval
// https://meds.queensu.ca/central/assets/modules/ECG/normal_ecg.html
// R = 0-45 ms, 2.7 at peak (modeled with sine curve, zero-to-zero)
// T = 120-440 ms, 0.9 at peak (modeled with sine curve, bottom-to-top-to-bottom)

#define HEARTBEAT_COLOR_RED 114
#define HEARTBEAT_COLOR_GREEN 139
#define HEARTBEAT_COLOR_BLUE 27

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

#define HEARTBEAT_UPDATE_RATE 1000 / 60

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

const Color3 heartbeatBaseColor(
  HEARTBEAT_COLOR_RED,
  HEARTBEAT_COLOR_GREEN,
  HEARTBEAT_COLOR_BLUE);
const Color3 heartbeatStressColor(Color3::Red);

Color3 heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long currentTime);
Color3 heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long currentTime) {
  // Using 1024ms per "second" because the compiler turns % 1024 into bitwise math instead of a divide
  return Color3(pgm_read_byte_near(heartbeatPulse + (currentTime + currentPixelIndex) % 1024));
}

void updateStress(uint8_t& stress) {
  // TODO: Calculate good values for 60fps
  static int8_t stressVelocity = 0;
  int8_t stressAcceleration = random(10 + 1) - 5;

  stressVelocity += stressAcceleration;
  stressVelocity = std::clamp(stressVelocity, (int8_t)-85, (int8_t)85);

  stress = std::clamp(static_cast<int16_t>(stress) + stressVelocity, 0, 255);
}

void heartbeatPattern(Adafruit_NeoPixel& strip, const unsigned long currentTime, unsigned long timeOffset = 0) {
  static unsigned long lastTime = currentTime;
  static unsigned long updateTime = 0;
  static unsigned long fpsTime = 0;

  unsigned long deltaTime = currentTime - lastTime;
  updateTime += deltaTime;
  fpsTime += deltaTime;
  lastTime = currentTime;

  static uint8_t stress = 0;

  if(updateTime > HEARTBEAT_UPDATE_RATE) {
    updateTime -= HEARTBEAT_UPDATE_RATE;
    updateStress(stress);
  }

  static unsigned long frameCounter = 0;
  ++frameCounter;

  if(fpsTime > 1000) {
    fpsTime -= 1000;
    Serial.println(frameCounter);
    frameCounter = 0;
  }

  unsigned long animationTime = currentTime * (stress / 255 + 1); // TODO: stress / 255 = 0 or 1 right now...

  for(long currentPixelIndex = 0; currentPixelIndex < NUM_LEDS; ++currentPixelIndex) {
    Color3 pulseColor(
      heartbeatPixelColor(currentPixelIndex, animationTime + timeOffset)
      / 4);
    pulseColor.r = 0;
    pulseColor.b = 0;

    Color3 backgroundColor(
      lerp(heartbeatBaseColor, heartbeatStressColor, stress));

    Color3 pixelColor(pulseColor + backgroundColor);

    strip.setPixelColor(
      currentPixelIndex,
      pixelColor.r,
      pixelColor.g,
      pixelColor.b);
  }

  strip.show();
}

// ------- Main program
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  randomSeed(analogRead(0));

  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
}

void loop() {
  unsigned long currentTime = millis();

  heartbeatPattern(strip, currentTime);
}

