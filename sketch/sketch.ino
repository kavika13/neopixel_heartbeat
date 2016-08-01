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
#define HEARTBEAT_DURATION_UPDATE (1000.0f / HEARTBEAT_UPDATES_PER_SECOND)
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

const uint8_t heartbeatBackgroundColors[] PROGMEM = {
  // Generated lerp in LAB color space.

  // 1. Make a virtualenv and activate it
  // 2. `pip install colormath`
  // 3. run this script:

  // #!/usr/bin/env python
  // from colormath.color_objects import LabColor, sRGBColor
  // from colormath.color_conversions import convert_color

  // # Non-stressed color:
  // HEARTBEAT_COLOR_RED = 114
  // HEARTBEAT_COLOR_GREEN = 139
  // HEARTBEAT_COLOR_BLUE = 27

  // rgb1 = sRGBColor(
  //     HEARTBEAT_COLOR_RED / 255.0,
  //     HEARTBEAT_COLOR_GREEN / 255.0,
  //     HEARTBEAT_COLOR_BLUE / 255.0)
  // lab1 = convert_color(rgb1, LabColor)

  // rgb1 = sRGBColor(1.0, 0.0, 0.0)
  // lab1 = convert_color(rgb1, LabColor)

  // # Maximum stressed color
  // rgb2 = sRGBColor(0.0, 1.0, 0.0)
  // lab2 = convert_color(rgb2, LabColor)

  // def interpolate_component(source, target, weight):
  //     return source + (target - source) * weight

  // def interpolate_lab(source, target, weight):
  //     return LabColor(
  //         interpolate_component(source.lab_l, target.lab_l, weight),
  //         interpolate_component(source.lab_a, target.lab_a, weight),
  //         interpolate_component(source.lab_b, target.lab_b, weight),
  //         source.observer,
  //         source.illuminant)

  // def rgb8(srgb_color):
  //     return [
  //         int(255.0 * float('%.4f ' % srgb_color.rgb_r)),
  //         int(255.0 * float('%.4f ' % srgb_color.rgb_g)),
  //         int(255.0 * float('%.4f ' % srgb_color.rgb_b))]

  // gradient = [
  //     rgb8(convert_color(interpolate_lab(lab1, lab2, i / 255.0), sRGBColor))
  //     for i in range(256)]

  // for c in gradient:
  //     print '{}, {}, {},'.format(c[0], c[1], c[2])

  114, 139, 27,
  114, 138, 26,
  115, 138, 26,
  116, 138, 26,
  116, 138, 26,
  117, 138, 26,
  118, 137, 26,
  119, 137, 26,
  119, 137, 25,
  120, 137, 25,
  121, 137, 25,
  122, 136, 25,
  122, 136, 25,
  123, 136, 25,
  124, 136, 25,
  124, 136, 24,
  125, 135, 24,
  126, 135, 24,
  126, 135, 24,
  127, 135, 24,
  128, 135, 24,
  128, 134, 24,
  129, 134, 23,
  130, 134, 23,
  130, 134, 23,
  131, 134, 23,
  132, 133, 23,
  132, 133, 23,
  133, 133, 23,
  134, 133, 22,
  134, 133, 22,
  135, 132, 22,
  136, 132, 22,
  136, 132, 22,
  137, 132, 22,
  138, 132, 22,
  138, 131, 21,
  139, 131, 21,
  140, 131, 21,
  140, 131, 21,
  141, 130, 21,
  142, 130, 21,
  142, 130, 21,
  143, 130, 20,
  143, 130, 20,
  144, 129, 20,
  145, 129, 20,
  145, 129, 20,
  146, 129, 20,
  147, 128, 20,
  147, 128, 19,
  148, 128, 19,
  148, 128, 19,
  149, 128, 19,
  150, 127, 19,
  150, 127, 19,
  151, 127, 19,
  151, 127, 18,
  152, 126, 18,
  153, 126, 18,
  153, 126, 18,
  154, 126, 18,
  154, 125, 18,
  155, 125, 18,
  156, 125, 18,
  156, 125, 17,
  157, 124, 17,
  157, 124, 17,
  158, 124, 17,
  159, 124, 17,
  159, 123, 17,
  160, 123, 16,
  160, 123, 16,
  161, 123, 16,
  161, 122, 16,
  162, 122, 16,
  163, 122, 16,
  163, 122, 16,
  164, 121, 15,
  164, 121, 15,
  165, 121, 15,
  165, 121, 15,
  166, 120, 15,
  167, 120, 15,
  167, 120, 15,
  168, 120, 14,
  168, 119, 14,
  169, 119, 14,
  169, 119, 14,
  170, 118, 14,
  171, 118, 14,
  171, 118, 14,
  172, 118, 13,
  172, 117, 13,
  173, 117, 13,
  173, 117, 13,
  174, 116, 13,
  174, 116, 13,
  175, 116, 13,
  176, 116, 12,
  176, 115, 12,
  177, 115, 12,
  177, 115, 12,
  178, 114, 12,
  178, 114, 12,
  179, 114, 11,
  179, 114, 11,
  180, 113, 11,
  181, 113, 11,
  181, 113, 11,
  182, 112, 11,
  182, 112, 11,
  183, 112, 10,
  183, 111, 10,
  184, 111, 10,
  184, 111, 10,
  185, 110, 10,
  185, 110, 10,
  186, 110, 10,
  186, 109, 9,
  187, 109, 9,
  188, 109, 9,
  188, 108, 9,
  189, 108, 9,
  189, 108, 9,
  190, 107, 8,
  190, 107, 8,
  191, 107, 8,
  191, 106, 8,
  192, 106, 8,
  192, 106, 8,
  193, 105, 8,
  193, 105, 8,
  194, 105, 7,
  194, 104, 7,
  195, 104, 7,
  195, 104, 7,
  196, 103, 7,
  196, 103, 7,
  197, 102, 7,
  198, 102, 6,
  198, 102, 6,
  199, 101, 6,
  199, 101, 6,
  200, 101, 6,
  200, 100, 6,
  201, 100, 6,
  201, 99, 6,
  202, 99, 5,
  202, 99, 5,
  203, 98, 5,
  203, 98, 5,
  204, 97, 5,
  204, 97, 5,
  205, 97, 5,
  205, 96, 5,
  206, 96, 5,
  206, 95, 4,
  207, 95, 4,
  207, 95, 4,
  208, 94, 4,
  208, 94, 4,
  209, 93, 4,
  209, 93, 4,
  210, 92, 4,
  210, 92, 4,
  211, 91, 4,
  211, 91, 3,
  212, 91, 3,
  212, 90, 3,
  213, 90, 3,
  213, 89, 3,
  214, 89, 3,
  214, 88, 3,
  215, 88, 3,
  215, 87, 3,
  216, 87, 3,
  216, 86, 3,
  217, 86, 2,
  217, 85, 2,
  218, 85, 2,
  218, 84, 2,
  219, 84, 2,
  219, 83, 2,
  220, 83, 2,
  220, 82, 2,
  221, 82, 2,
  221, 81, 2,
  222, 80, 2,
  222, 80, 2,
  223, 79, 2,
  223, 79, 2,
  224, 78, 1,
  224, 78, 1,
  225, 77, 1,
  225, 76, 1,
  226, 76, 1,
  226, 75, 1,
  227, 75, 1,
  227, 74, 1,
  228, 73, 1,
  228, 73, 1,
  229, 72, 1,
  229, 71, 1,
  230, 71, 1,
  230, 70, 1,
  231, 69, 1,
  231, 69, 1,
  232, 68, 1,
  232, 67, 0,
  233, 67, 0,
  233, 66, 0,
  234, 65, 0,
  234, 64, 0,
  235, 64, 0,
  235, 63, 0,
  236, 62, 0,
  236, 61, 0,
  237, 61, 0,
  237, 60, 0,
  238, 59, 0,
  238, 58, 0,
  239, 57, 0,
  239, 56, 0,
  240, 55, 0,
  240, 55, 0,
  241, 54, 0,
  241, 53, 0,
  242, 52, 0,
  242, 51, 0,
  242, 50, 0,
  243, 49, 0,
  243, 47, 0,
  244, 46, 0,
  244, 45, 0,
  245, 44, 0,
  245, 43, 0,
  246, 42, 0,
  246, 40, 0,
  247, 39, 0,
  247, 37, 0,
  248, 36, 0,
  248, 35, 0,
  249, 33, 0,
  249, 31, 0,
  250, 30, 0,
  250, 28, 0,
  251, 26, 0,
  251, 23, 0,
  252, 21, 0,
  252, 19, 0,
  253, 16, 0,
  253, 12, 0,
  254, 8, 0,
  254, 4, 0,
  255, 0, 0,
};

Color3 heartbeatBackgroundColor(uint8_t stress) {
  return Color3(
    pgm_read_byte_near(heartbeatBackgroundColors + (stress * 3) + 0),
    pgm_read_byte_near(heartbeatBackgroundColors + (stress * 3) + 1),
    pgm_read_byte_near(heartbeatBackgroundColors + (stress * 3) + 2));
}

Color3 heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long animationTime);
Color3 heartbeatPixelColor(unsigned long currentPixelIndex, unsigned long animationTime) {
  // Using 1024ms per heartbeat because the compiler turns % 1024 into bitwise math instead of a divide
  return Color3(pgm_read_byte_near(heartbeatPulse + (animationTime + currentPixelIndex) % 1024));
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

void heartbeatPattern(Adafruit_NeoPixel& strip, const unsigned long currentTime, unsigned long timeOffset = 0) {
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

  static unsigned long frameCounter = 0;
  ++frameCounter;

  if(fpsTime > 1000) {
    fpsTime -= 1000;
    Serial.print("heartbeatPattern FPS: ");
    Serial.println(frameCounter);
    frameCounter = 0;
  }

  // Integer version of: deltaTime * ([0.0, 2.0] + 1)
  animationTime += (deltaTime * ((stress + 1) * 2 + 256) * 256) / 65536;
  // Using 1024ms per heartbeat because the compiler turns % 1024 into bitwise math instead of a divide
  animationTime %= 1024;

  for(long currentPixelIndex = 0; currentPixelIndex < NUM_LEDS; ++currentPixelIndex) {
    Color3 pulseColor(
      heartbeatPixelColor(currentPixelIndex, animationTime + timeOffset)
      / 4);
    pulseColor.r = 0;
    pulseColor.b = 0;

    Color3 backgroundColor(heartbeatBackgroundColor(stress));

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

