/*
 * Repulsor control sketch for Seeed XIAO nRF52840 Sense.
 *
 * Combines the LSM6DS3 accelerometer baseline demo with
 * button, lighting, and sound handling logic that was
 * originally written for an ESP32/ADXL345 project. The
 * motion logic has been ported to use the on-board LSM6DS3.
 */

#include <Arduino.h>
#include <FastLED.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>

#include <DFRobotDFPlayerMini.h>

#define BUTTON_PIN 1
#define NEOPIXEL_PIN 10
#define NUM_PIXELS 8
#define DFPLAYER_RX 7
#define DFPLAYER_TX 8

// Timing constants (milliseconds)
const unsigned long longPressThreshold = 1000;
const unsigned long veryLongPressThreshold = 5000;
const unsigned long extraLongPressThreshold = 8000;
const unsigned long fadeDuration = 500;
const unsigned long angleCheckInterval = 50;
const unsigned long blastDuration = 1500;
const unsigned long blastCooldown = 1000;

// Motion filtering constants
const int filterSize = 20;
const float movementThreshold = 0.8f;

enum Mode {
  ONBOARD_SOUND,
  WIRELESS_SOUND
};

enum FadeState {
  OFF,
  FADING_ON,
  ON,
  FADING_OFF,
  ALWAYS_ON,
  BLAST
};

CRGB leds[NUM_PIXELS];
LSM6DS3 myIMU(I2C_MODE, 0x6A);
DFRobotDFPlayerMini dfPlayer;

Mode currentMode = ONBOARD_SOUND;
FadeState fadeState = OFF;

// State variables for button and motion handling
unsigned long buttonPressStartTime = 0;
unsigned long fadeStartTime = 0;
unsigned long lastAngleCheckTime = 0;
unsigned long blastStartTime = 0;
unsigned long lastBlastTime = 0;
bool blastTriggered = false;
bool dfPlayerReady = false;

// Moving average buffer index
int bufferIndex = 0;

// Forward declarations
void handleButtonPress();
void handleAccelerometer();
void handleNeoPixels();
void playSound(uint8_t track);
void sendWirelessData(uint8_t data);
float calculateMovingAverage(float newValue);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("XIAO nRF52840 Sense Repulsor Controller");
  Serial.println("=======================================");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  Serial.println("Initializing LSM6DS3...");
  if (myIMU.begin() != 0) {
    Serial.println("ERROR: Failed to initialize LSM6DS3! Check wiring and library installation.");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("LSM6DS3 initialized successfully!");

  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, NUM_PIXELS);
  FastLED.clear();
  FastLED.show();
  Serial.println("FastLED initialized");

  Serial1.begin(9600);
  if (dfPlayer.begin(Serial1)) {
    dfPlayer.setTimeOut(500);
    dfPlayer.volume(25);
    Serial.println("DFPlayer Mini initialized");
    dfPlayerReady = true;
  } else {
    Serial.println("WARNING: DFPlayer Mini not detected. Sounds will be skipped.");
  }

  Serial.println("Setup complete. Ready for input.");
}

void loop() {
  handleButtonPress();
  handleAccelerometer();
  handleNeoPixels();
}

void handleButtonPress() {
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();

  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStartTime = currentTime;
  } else if (buttonState == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = currentTime - buttonPressStartTime;

    if (pressDuration >= extraLongPressThreshold) {
      currentMode = (currentMode == ONBOARD_SOUND) ? WIRELESS_SOUND : ONBOARD_SOUND;
      Serial.print("Mode switched to: ");
      Serial.println(currentMode == ONBOARD_SOUND ? "ONBOARD_SOUND" : "WIRELESS_SOUND");
    } else if (pressDuration >= veryLongPressThreshold) {
      if (fadeState == ALWAYS_ON) {
        fadeState = OFF;
        Serial.println("Exiting ALWAYS_ON mode");
      } else {
        fadeState = ALWAYS_ON;
        Serial.println("Entering ALWAYS_ON mode");
      }
    } else if (pressDuration >= longPressThreshold) {
      sendWirelessData(102);
      Serial.println("Long press detected");
    } else {
      sendWirelessData(101);
      Serial.println("Short press detected");
    }

    delay(200);
  }

  lastButtonState = buttonState;
}

void handleAccelerometer() {
  unsigned long currentTime = millis();
  if (currentTime - lastAngleCheckTime < angleCheckInterval) {
    return;
  }
  lastAngleCheckTime = currentTime;

  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();

  float angle = atan2(y, z) * 180.0f / PI;
  float filteredAngle = calculateMovingAverage(angle);
  static float lastFilteredAngle = 0.0f;

  if (fabs(filteredAngle - lastFilteredAngle) < movementThreshold) {
    return;
  }

  float accelerationChange = x;
  float yAngle = atan2(x, z) * 180.0f / PI;
  float zAngle = atan2(x, y) * 180.0f / PI;

  if (!blastTriggered && accelerationChange < -0.9f && fabs(yAngle) < 45.0f && fabs(zAngle) < 30.0f) {
    if (fadeState == ON && (currentTime - lastBlastTime > blastCooldown)) {
      fadeState = BLAST;
      blastTriggered = true;
      blastStartTime = currentTime;

      if (currentMode == WIRELESS_SOUND) {
        sendWirelessData(13);
      } else {
        playSound(3);
      }

      Serial.println("BLAST activated");
      lastBlastTime = currentTime;
    }
  }

  lastFilteredAngle = filteredAngle;

  if (filteredAngle >= 35.0f && fadeState == OFF) {
    if (currentMode == WIRELESS_SOUND) {
      sendWirelessData(12);
    } else {
      playSound(2);
    }
    fadeState = FADING_ON;
    fadeStartTime = currentTime;
    Serial.println("Starting to fade on");
  } else if (filteredAngle < 35.0f && fadeState == ON) {
    if (currentMode == WIRELESS_SOUND) {
      sendWirelessData(11);
    } else {
      playSound(1);
    }
    fadeState = FADING_OFF;
    fadeStartTime = currentTime;
    Serial.println("Starting to fade off");
  }
}

void handleNeoPixels() {
  unsigned long currentTime = millis();

  switch (fadeState) {
    case OFF:
      FastLED.clear();
      FastLED.show();
      break;

    case FADING_ON: {
      unsigned long elapsed = currentTime - fadeStartTime;
      if (elapsed < fadeDuration) {
        int brightness = map(elapsed, 0, fadeDuration, 0, 255);
        for (int i = 0; i < NUM_PIXELS; ++i) {
          leds[i] = CRGB(brightness, brightness, brightness);
        }
        FastLED.show();
      } else {
        for (int i = 0; i < NUM_PIXELS; ++i) {
          leds[i] = CRGB::White;
        }
        FastLED.show();
        fadeState = ON;
        Serial.println("Fade on complete");
      }
      break;
    }

    case ON:
      for (int i = 0; i < NUM_PIXELS; ++i) {
        leds[i] = CRGB::White;
      }
      FastLED.show();
      break;

    case FADING_OFF: {
      unsigned long elapsed = currentTime - fadeStartTime;
      if (elapsed < fadeDuration) {
        int brightness = map(elapsed, 0, fadeDuration, 255, 0);
        for (int i = 0; i < NUM_PIXELS; ++i) {
          leds[i] = CRGB(brightness, brightness, brightness);
        }
        FastLED.show();
      } else {
        FastLED.clear();
        FastLED.show();
        fadeState = OFF;
        Serial.println("Fade off complete");
      }
      break;
    }

    case ALWAYS_ON:
      for (int i = 0; i < NUM_PIXELS; ++i) {
        leds[i] = CRGB::White;
      }
      FastLED.show();
      break;

    case BLAST:
      if (currentTime - blastStartTime < blastDuration) {
        for (int i = 0; i < NUM_PIXELS; ++i) {
          int flicker = random(100, 150);
          if (random(3) == 0) {
            leds[i] = CRGB(flicker, flicker / 2, 0);
          } else {
            leds[i] = CRGB(flicker, flicker, flicker);
          }
        }
        FastLED.show();
      } else {
        FastLED.clear();
        FastLED.show();
        blastTriggered = false;
        fadeState = ON;
        Serial.println("BLAST complete");
      }
      break;
  }
}

void playSound(uint8_t track) {
  if (!dfPlayerReady) {
    return;
  }
  dfPlayer.play(track);
}

void sendWirelessData(uint8_t data) {
  // The XIAO nRF52840 Sense does not provide Wi-Fi or ESP-NOW hardware.
  // For development purposes, we log the payload so that an external
  // radio module (if connected) can mirror this behaviour.
  Serial.print("Wireless data requested: ");
  Serial.println(data);
}

float calculateMovingAverage(float newValue) {
  static float sum = 0.0f;
  static int count = 0;
  static float readings[filterSize] = {0};

  sum -= readings[bufferIndex];
  readings[bufferIndex] = newValue;
  sum += readings[bufferIndex];

  bufferIndex = (bufferIndex + 1) % filterSize;
  if (count < filterSize) {
    ++count;
  }

  return sum / count;
}
