#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

const uint8_t INA_ADDR = 0x41;  // tipico indirizzo del tuo modulo
Adafruit_INA219 ina219(INA_ADDR);

// --- funzione per calibrazione custom
void setCalibration_Custom(float Rshunt, float Imax) {
  // Calcola Current_LSB
  float currentLSB = Imax / 32767.0;

  // Calibrazione
  uint16_t calibration = (uint16_t)(0.04096 / (currentLSB * Rshunt));

  // Power LSB
  float powerLSB = currentLSB * 20.0;

  // Scrive il registro di calibrazione
  Wire.beginTransmission(INA_ADDR);
  Wire.write((uint8_t)0x05);               // Calibration register
  Wire.write((calibration >> 8) & 0xFF);
  Wire.write(calibration & 0xFF);
  Wire.endTransmission();

  Serial.printf("Custom INA219 calibrazione:\n");
  Serial.printf("Rshunt=%.6f Ω | Imax=%.1f A\n", Rshunt, Imax);
  Serial.printf("Current_LSB=%.6f A/bit | Cal=0x%04X | Power_LSB=%.6f W/bit\n",
                currentLSB, calibration, powerLSB);
}

// --- funzione per leggere un registro raw
uint16_t readReg(uint8_t reg) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFFFF;
  Wire.requestFrom((int)INA_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA=21, SCL=22 su ESP32

  if (!ina219.begin()) {
    Serial.println("INA219 non trovato!");
    while (1);
  }

  // Calibrazione custom per 60A con Rshunt da 5.35 mΩ
  setCalibration_Custom(0.00535, 60.0);
}

void loop() {
  uint16_t reg_cfg   = readReg(0x00);
  uint16_t reg_shunt = readReg(0x01);
  uint16_t reg_bus   = readReg(0x02);
  uint16_t reg_power = readReg(0x03);
  uint16_t reg_curr  = readReg(0x04);
  uint16_t reg_cal   = readReg(0x05);

  Serial.printf("CFG=0x%04X  SHUNT=0x%04X  BUS=0x%04X  PWR=0x%04X  CUR=0x%04X  CAL=0x%04X\n",
                reg_cfg, reg_shunt, reg_bus, reg_power, reg_curr, reg_cal);

  // interpretazioni corrette
int16_t sh_signed = (int16_t)reg_shunt;
float sh_mV = sh_signed * 0.01f;  // 10 µV LSB -> 0.01 mV

// Bus voltage: bit [15:13] flag, bit [12:3] validi
uint16_t bus_raw = (reg_bus >> 3) & 0x1FFF;
float busV = bus_raw * 0.004f;    // 4 mV per LSB

  Serial.printf("Interpreted -> Bus_reg_shifted=%u -> BusV=%.4f V | Shunt= %d -> %.4f mV\n",
                bus_raw, busV, sh_signed, sh_mV);

  delay(700);
}
