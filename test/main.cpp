#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_INA219.h>

// ====== ESC configuration ======
Servo esc;
const int escPin = 2;
const int potPin = 13;
const int minThrottle = 1000; // µs
const int maxThrottle = 2000; // µs
const float SHUNT_RESISTANCE_OHMS = 0.005347606f; // 5.35 mΩ (1 m, Ø 2 mm, rame)

// ====== OLED configuration ======
U8G2_SSD1315_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ====== INA219 configuration ======
Adafruit_INA219 ina219(0x41);

void scanI2CAddress() {
  Serial.println("Scanning for I2C devices...");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      if (address == 0x40 || address == 0x41) { // Common INA219 addresses
        ina219 = Adafruit_INA219(address);
        break;
      }
    }
  }
  Serial.println("I2C scan complete.");
}
bool sensorAvailable = false;

// -------------------------------------------------------------------
// Read potentiometer, map to ESC range and compute power percentage
// -------------------------------------------------------------------
void readInput(int potPin, int minThrottle, int maxThrottle, int &potValue, int &escValue, int &powerPercent) {
  potValue = analogRead(potPin);
  escValue = map(potValue, 0, 4095, minThrottle, maxThrottle);
  powerPercent = map(escValue, minThrottle, maxThrottle, 0, 100);
}

// -------------------------------------------------------------------
// Write signal to ESC
// -------------------------------------------------------------------
void updateESC(Servo &esc, int escValue) {
  esc.writeMicroseconds(escValue);
}

// -------------------------------------------------------------------
// Read sensors (INA219 or fallback fixed values)
// -------------------------------------------------------------------
void readSensors(Adafruit_INA219 &ina219, bool sensorAvailable,
                 float shuntResistance,
                 float &batteryVoltage, float &batteryCurrent) {
  if (sensorAvailable) {
    // Bus voltage (V) misurata dall'INA219
    float busV = ina219.getBusVoltage_V();
    // Shunt voltage in mV -> converti a V
    float shuntV = ina219.getShuntVoltage_mV() / 1000.0f;
    // Calcola corrente usando la resistenza di shunt che passi
    batteryCurrent = 0.0f;
    if (shuntResistance > 0.0f) {
      batteryCurrent = shuntV / shuntResistance; // A
    }
    // Tensione batteria: bus voltage + caduta shunt (se vuoi includerla)
    batteryVoltage = busV + shuntV;
  } else {
    // Dummy fallback
    batteryVoltage = 14.8f;
    batteryCurrent = 3.2f;
  }
}

// -------------------------------------------------------------------
// Update OLED display
// -------------------------------------------------------------------
void updateDisplay(U8G2 &display, int powerPercent, float batteryVoltage, float batteryCurrent) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  char buf[32];
  snprintf(buf, sizeof(buf), "Power: %3d %%", powerPercent);
  display.drawStr(0, 12, buf);

  snprintf(buf, sizeof(buf), "Voltage: %.2f V", batteryVoltage);
  display.drawStr(0, 28, buf);

  snprintf(buf, sizeof(buf), "Current: %.2f A", batteryCurrent);
  display.drawStr(0, 44, buf);

  // Potenza istantanea
  float powerW = batteryVoltage * batteryCurrent;
  snprintf(buf, sizeof(buf), "Power: %.1f W", powerW);
  display.drawStr(0, 60, buf);

  display.sendBuffer();
}

// -------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // ESC
  esc.attach(escPin);

  // OLED
  Wire.begin(21, 22);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "ESC Controller Init");
  display.sendBuffer();

  // INA219
  if (ina219.begin()) {
    sensorAvailable = true;
    Serial.println("INA219 found.");
  } else {
    sensorAvailable = false;
    Serial.println("INA219 not detected. Using dummy values.");
  }

  delay(1000);
}

// -------------------------------------------------------------------
// Loop
// -------------------------------------------------------------------
void loop() {
  int escValue, potValue, powerPercent;
  float batteryVoltage, batteryCurrent;

  // 1) Input
  readInput(potPin, minThrottle, maxThrottle, potValue, escValue, powerPercent);

  // 2) ESC output
  updateESC(esc, escValue);

  // 3) Sensors
  readSensors(ina219, sensorAvailable, SHUNT_RESISTANCE_OHMS, batteryVoltage, batteryCurrent);


  // 4) Display
  updateDisplay(display, powerPercent, batteryVoltage, batteryCurrent);

  // Debug serial
  //Serial.printf("Pot: %d | ESC: %d | Power: %d%% | Vbat: %.4f V | Ibat: %.4f A\n",
  //              potValue, escValue, powerPercent, batteryVoltage, batteryCurrent);

  Serial.printf("BusV: %.4f V | ShuntV: %.4f mV\n",
                ina219.getBusVoltage_V(), ina219.getShuntVoltage_mV());

  //scanI2CAddress();

  delay(200);


}
