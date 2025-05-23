/*
 * Project Name: Precision Digital Micro Current Meter
 * Project Brief: Precision Digital Micro Current Meter based STM32G071KBU MCU.
 * Author: Jobit Joseph @ https://github.com/jobitjoseph
 * IDE: Arduino IDE 2.3.6
 * Arduino Core: ESP32 Arduino Core V 3.2.0
 * Dependencies : U8g2 Library V 2.35.30 @ https://github.com/olikraus/u8g2
 *
 * Copyright © Jobit Joseph
 * Copyright © Semicon Media Pvt Ltd
 * Copyright © Circuitdigest.com
 * 
 * This code is licensed under the following conditions:
 *
 * 1. Non-Commercial Use:
 * This program is free software: you can redistribute it and/or modify it
 * for personal or educational purposes under the condition that credit is given 
 * to the original author. Attribution is required, and the original author 
 * must be credited in any derivative works or distributions.
 *
 * 2. Commercial Use:
 * For any commercial use of this software, you must obtain a separate license
 * from the original author. Contact the author for permissions or licensing
 * options before using this software for commercial purposes.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING 
 * FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jobit Joseph
 * Date: 23 May 2025
 *
 * For commercial use or licensing requests, please contact [jobitjoseph1@gmail.com].
 */

#include <Wire.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#define LATCH_PIN PA4
#define NUM_SAMPLES 2
#define SAMPLE_DELAY 10
#define REF_VOLTAGE 1249.3f  // float for precision

// Define our custom HardwareSerial
HardwareSerial MySerial(PB6, PB7);  // TX, RX
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
volatile bool buttonPressed = false;
volatile bool buttonReleased = false;
volatile unsigned long lastInterruptTime = 0;
unsigned long releaseTime = 0;
const unsigned long debounceDelay = 50;
const unsigned long shutdownDelay = 1000;

unsigned long lastBatteryUpdate = 0;
float batteryVoltage = 0;
int AmpV = 0;
float AmpVF = 0;
int batteryPercent = 0;

void buttonISR() {
  unsigned long now = millis();
  if (now - lastInterruptTime < debounceDelay) return;

  if (digitalRead(LATCH_PIN) == LOW) {
    buttonPressed = true;
    buttonReleased = false;
  } else {
    if (buttonPressed) {
      buttonReleased = true;
      releaseTime = now;
      delay(500);
      pinMode(LATCH_PIN, OUTPUT);
      digitalWrite(LATCH_PIN, LOW);
      while (1)
        ;
    }
    buttonPressed = false;
  }
  lastInterruptTime = now;
}

void setup() {
  pinMode(LATCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LATCH_PIN), buttonISR, CHANGE);
  MySerial.begin(115200);
  MySerial.println("STM32G071 ADC with External Reference (PA2)");
  analogReadResolution(12);
  u8g2.begin();
}

void loop() {
  unsigned long now = millis();
  if (now - lastBatteryUpdate > 500) {
    float vdda = measureVDDA();
    batteryVoltage = measureInputVoltage(PA3, vdda) * 2.0f;
    float Vref = measureInputVoltage(PA1, vdda);
    float ampVoltage = measureInputVoltage(PA0, vdda);
    //AmpV = ((int)((ampVoltage - Vref) / 2.0f)) - 3;
    //float error_percent = 4.77f;
    //AmpV = (int)(((ampVoltage - Vref) / 2.0f) * (1.0f - error_percent / 100.0f))-3;
    float raw = ((ampVoltage - Vref) / 1.0f);
    float offset = 6.0f;
    float error_percent = 0.0f;
    //AmpV = (int)((raw - offset) * (1.0f - error_percent / 100.0f));
    AmpVF = (raw  * (1.0f - error_percent / 100.0f)- offset);
    AmpV = (int)AmpVF;
    batteryPercent = batteryLevelPercent((int)batteryVoltage);

    MySerial.print("VDDA: "); MySerial.print(vdda);
    MySerial.print(" mV, Vref: "); MySerial.print(Vref);
    MySerial.print(" mV, PA2: "); MySerial.print(measureInputVoltage(PA2, vdda));
    MySerial.print(" mV, Amp: "); MySerial.println(AmpV);

    lastBatteryUpdate = now;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);

    int iconX = 96, iconY = 0, iconW = 28, iconH = 12;
    char batteryPercentStr[6];
    itoa(batteryPercent, batteryPercentStr, 10);
    strcat(batteryPercentStr, "%");
    u8g2.drawStr(iconX - 20, iconY + 10, batteryPercentStr);

    drawBatteryIconWithBars(iconX, iconY, iconW, iconH, batteryPercent);

    char AmpStr[7];
    if (AmpV < 0) {
      sprintf(AmpStr, "-%04d", abs(AmpV));
    } else {
      sprintf(AmpStr, " %04d", AmpV);
    }

    u8g2.setFont(u8g2_font_inb33_mn);
    u8g2.drawStr(-10, 62, AmpStr);
    u8g2.sendBuffer();
  }
}

float measureVDDA() {

  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += oversampledADC(PA1);
    delayMicroseconds(SAMPLE_DELAY);
  }
  float adcAvg = (float)sum / NUM_SAMPLES;
  float vdda = (REF_VOLTAGE * 65535.0f) / adcAvg;
  return vdda;
}

float measureInputVoltage(uint32_t pin, float vdda) {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += oversampledADC(pin);  
    delayMicroseconds(SAMPLE_DELAY);
  }
  float adcAvg = (float)sum / NUM_SAMPLES;
  float voltage = (adcAvg * vdda) / 65535.0f;
  return voltage;
}

uint32_t oversampledADC(uint32_t pin) {
  uint32_t sum = 0;
  const int oversample_count = 256;  // 2^(2 × (16 - 12)) = 256

  for (int i = 0; i < oversample_count; i++) {
    sum += analogRead(pin);  // 12-bit reading
    delayMicroseconds(SAMPLE_DELAY);   // optional: allow settling
  }

  // Average and scale to 16-bit range
  uint32_t avg = sum / oversample_count;
  uint32_t adc16 = avg << 4;  // 12-bit to 16-bit shift (multiply by 16)
  // Ignore 3 LSBs
  const int ignore_lsb = 4;
  adc16 = (adc16 >> ignore_lsb) << ignore_lsb;

  return adc16;
}

int batteryLevelPercent(int v_bat) {
  int percent = (v_bat - 3000) * 100 / (4200 - 3000);
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  return percent;
}

void drawBatteryIconWithBars(int x, int y, int w, int h, int percent) {
  u8g2.drawFrame(x, y, w, h);
  u8g2.drawBox(x + w, y + h / 3, 2, h / 3);

  int numBars = 4;
  int barSpacing = 2;
  int barWidth = (w - 6 - (numBars - 1) * barSpacing) / numBars;
  int barHeight = h - 4;
  int barsToDraw = (percent + 24) / 25;

  for (int i = 0; i < barsToDraw; i++) {
    int bx = x + 2 + i * (barWidth + barSpacing);
    u8g2.drawBox(bx, y + 2, barWidth, barHeight);
  }
}
