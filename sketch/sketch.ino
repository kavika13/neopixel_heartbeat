#include <avr/pgmspace.h>
#include <Adafruit_NeoPixel.h>

#define DATA_PIN 7
#define NUM_LEDS 59
#define BRIGHTNESS (64 - 1)

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

#define HEARTBEAT_OFFSET_T 120
#define HEARTBEAT_DURATION_T 320
#define HEARTBEAT_AMPLITUDE_T 0.9f
#define HEARTBEAT_WAVE_OFFSET_T (PI * 1.5)
#define HEARTBEAT_WAVE_DURATION_T (PI * 2)

// Helpers to statically generate our table
#define S4(i)    S1((i)),   S1((i)+1),     S1((i)+2),     S1((i)+3)
#define S8(i)    S4((i)),   S4((i)+4)
#define S16(i)   S8((i)),   S8((i)+8)
#define S32(i)   S16((i)),  S16((i)+16)
#define S64(i)   S32((i)),  S32((i)+32)
#define S128(i)  S64((i)),  S64((i)+64)
#define S256(i)  S128((i)), S128((i)+128)
#define S512(i)  S256((i)), S256((i)+256)
#define S1000(i) S512((i)), S256((i)+512), S128((i)+768), S64((i) + 896), S32((i) + 960), S8((i) + 992)

const uint8_t heartbeatPulse[] PROGMEM = {
#define S1(milliseconds) static_cast<uint8_t>( \
  255.0f * ( \
    (milliseconds >= HEARTBEAT_OFFSET_R && milliseconds - HEARTBEAT_OFFSET_R < HEARTBEAT_DURATION_R) \
      ? sin(static_cast<float>(milliseconds - HEARTBEAT_OFFSET_R) \
        / HEARTBEAT_DURATION_R * HEARTBEAT_WAVE_DURATION_R + HEARTBEAT_WAVE_OFFSET_R) \
      : ( \
        (milliseconds >= HEARTBEAT_OFFSET_T && milliseconds - HEARTBEAT_OFFSET_T < HEARTBEAT_DURATION_T) \
        ? (HEARTBEAT_AMPLITUDE_T / HEARTBEAT_AMPLITUDE_R) \
          * sin(static_cast<float>(milliseconds - HEARTBEAT_OFFSET_T) \
          / HEARTBEAT_DURATION_T * HEARTBEAT_WAVE_DURATION_T + HEARTBEAT_WAVE_OFFSET_T) \
        : 0.0f)))
  S1000(0)
#undef S1
};

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
}

void heartbeatPattern(Adafruit_NeoPixel& strip, unsigned long currentTime) {
  float red = HEARTBEAT_COLOR_RED;
  float green = HEARTBEAT_COLOR_GREEN;
  float blue = HEARTBEAT_COLOR_BLUE;

  for(long currentPixel = 0; currentPixel < NUM_LEDS; ++currentPixel) {
    float currentBrightness = static_cast<float>(
        pgm_read_byte_near(heartbeatPulse + (currentTime + currentPixel) % 1000))
      / 255.0f;

    strip.setPixelColor(
      currentPixel,
      red * currentBrightness,
      green * currentBrightness,
      blue * currentBrightness);
  }

  strip.show();  
}

void loop() {
  unsigned long currentTime = millis();

  heartbeatPattern(strip, currentTime);
}

