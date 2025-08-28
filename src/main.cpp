#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>

// ===== ESC setup =====
Servo esc;
int minThrottle = 1000; // µs
int maxThrottle = 2000; // µs

// ===== OLED setup =====
U8G2_SSD1315_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ===== Example fixed values (to be replaced with sensor readings) =====
float batteryVoltage = 14.8; // V
float batteryCurrent = 3.2;  // A

void setup() {
  // ESC
  esc.attach(2);

  // Serial
  Serial.begin(115200);

  // OLED
  Wire.begin(21, 22);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "ESC Controller Ready");
  display.sendBuffer();
  delay(1000);
}

void loop() {
  // Read potentiometer value
  int potValue = analogRead(13);

  // Map potentiometer to ESC PWM
  int escValue = map(potValue, 0, 4095, minThrottle, maxThrottle);
  esc.writeMicroseconds(escValue);

  // Power percentage
  int powerPercent = map(escValue, minThrottle, maxThrottle, 0, 100);

  // Debug output
  Serial.printf("Pot: %d | ESC: %d | Power: %d%%\n", potValue, escValue, powerPercent);

  // ==== OLED update ====
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  char buf[32];
  
  snprintf(buf, sizeof(buf), "Power: %3d %%", powerPercent);
  display.drawStr(0, 12, buf);

  snprintf(buf, sizeof(buf), "Voltage: %.1f V", batteryVoltage);
  display.drawStr(0, 28, buf);

  snprintf(buf, sizeof(buf), "Current: %.1f A", batteryCurrent);
  display.drawStr(0, 44, buf);

  display.sendBuffer();

  delay(200);
}
