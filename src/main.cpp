#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>

#define INA226_ADDRESS 0x41
const float SHUNT_RESISTANCE_OHMS = 1.44*0.005347606f; // 5.35 mΩ (1 m, Ø 2 mm, rame)
const float MAX_CURRENT = 30.0f; // 30 A

float Current_LSB; // Define it globally
float Power_LSB; // Define it globally
void INA_226_Calibration() {
  // Calibrare il sensore INA226

  // step 1: Current LSB
  Current_LSB = (MAX_CURRENT / 32767.0f); //A/bit

  //step 2: Calibration register
  uint16_t Calibration_value = (uint16_t)(0.00512/(Current_LSB * SHUNT_RESISTANCE_OHMS));

  // step 3: Power LSB
  Power_LSB = 25.0 * Current_LSB;

  // scrittura nel registro 0x05
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(0x05);
  Wire.write(Calibration_value >> 8);
  Wire.write(Calibration_value & 0xFF);
  Wire.endTransmission();

  Serial.println("=== INA226 Calibration ===");
  Serial.print("Shunt = ");
  Serial.print(SHUNT_RESISTANCE_OHMS, 6);
  Serial.println(" Ω");
  Serial.print("Imax = ");
  Serial.print(MAX_CURRENT, 1);
  Serial.println(" A");
  Serial.print("Current_LSB = ");
  Serial.print(Current_LSB, 9);
  Serial.println(" A/bit");
  Serial.print("Power_LSB = ");
  Serial.print(Power_LSB, 9);
  Serial.println(" W/bit");
  Serial.print("Calibration Register = 0x");
  Serial.println(Calibration_value, HEX);
  Serial.println("===========================");
}

uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA226_ADDRESS, 2);

  if (Wire.available() == 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

// ====== ESC configuration ======
Servo esc;
const int escPin = 2;
const int potPin = 13;
const int minThrottle = 1000; // µs
const int maxThrottle = 2000; // µs


// ====== OLED configuration ======
U8G2_SSD1315_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// Bus voltage in volts
float getBusVoltage_V() {
  uint16_t reg = readRegister16(0x02);
  return (reg * 1.25) / 1000.0;  // LSB = 1.25 mV
}

// Shunt voltage in mV
float getShuntVoltage_mV() {
  int16_t reg = (int16_t)readRegister16(0x01);  // signed
  return (reg * 2.5) / 1000.0;  // LSB = 2.5 µV -> in mV
}

float readCurrent() {
  int16_t reg = (int16_t)readRegister16(0x04);  // signed
  return reg * Current_LSB;
}

float readPower() {
  uint16_t reg = readRegister16(0x03);
  return reg * Power_LSB;
}




// -------------------------------------------------------------------
// Read potentiometer, map to ESC range and compute power percentage
// -------------------------------------------------------------------
void readInput(int potPin, int minThrottle, int maxThrottle, int &potValue, int &escValue, int &powerPercent) {
  potValue = analogRead(potPin);
  // Print raw potentiometer value for debugging
  Serial.printf("Raw Potentiometer Value: %d\n", potValue);
  escValue = map(potValue, 0, 4095, minThrottle, maxThrottle);
  powerPercent = map(escValue, minThrottle, maxThrottle, 0, 100);
}


#define POT_FILTER_SIZE 10
int potFilterBuffer[POT_FILTER_SIZE] = {0};
int potFilterIndex = 0;
long potFilterSum = 0;

// Simple moving average filter for potentiometer readings
int filterPotValue(int newValue) {
  potFilterSum -= potFilterBuffer[potFilterIndex];
  potFilterBuffer[potFilterIndex] = newValue;
  potFilterSum += newValue;
  potFilterIndex = (potFilterIndex + 1) % POT_FILTER_SIZE;
  return potFilterSum / POT_FILTER_SIZE;
}

// -------------------------------------------------------------------
// Write signal to ESC
// -------------------------------------------------------------------
void updateESC(Servo &esc, int escValue) {
  esc.writeMicroseconds(escValue);
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

  float powerW = batteryVoltage * batteryCurrent;
  snprintf(buf, sizeof(buf), "Power: %.1f W", powerW);
  display.drawStr(0, 60, buf);

  display.sendBuffer();
}

// Read INA 226 Parameters
void readSensors(float shuntRes, float &busVoltage, float &current, float &power, float &shuntVoltage) {
  busVoltage = getBusVoltage_V();
  current = readCurrent();
  power = readPower();
  shuntVoltage = getShuntVoltage_mV();

}





// -------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  esc.attach(escPin);

  Wire.begin(21, 22);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "ESC Controller Init");
  display.sendBuffer();

  INA_226_Calibration();

  //potentiometer pin setup
  analogReadResolution(12); // 12-bit resolution (0-4095)
  pinMode(potPin, INPUT); // Configure pin as input for ADC
}

// -------------------------------------------------------------------
// Loop
// -------------------------------------------------------------------
void loop() {
  int escValue, potValue, powerPercent;
  int filteredPotValue;
  float busVoltage, batteryCurrent, batteryPower, shuntVoltage;

  readInput(potPin, minThrottle, maxThrottle, potValue, escValue, powerPercent);
  filteredPotValue = filterPotValue(potValue);


  updateESC(esc, escValue);
  readSensors(SHUNT_RESISTANCE_OHMS, busVoltage, batteryCurrent, batteryPower, shuntVoltage);
  updateDisplay(display, powerPercent, busVoltage, batteryCurrent);

  /*
  Serial.printf("BusV: %.3f V | Current: %.3f A | Power: %.3f W | ShuntV: %.3f mV\n",
                busVoltage, batteryCurrent, batteryPower, shuntVoltage);

  // Print potentiometer values for debugging
  Serial.printf("Pot: %4d | Filtered Pot: %4d | ESC: %4d | Power: %3d %%\n",
                potValue, filteredPotValue, escValue, powerPercent);
  */


  delay(200);
}
