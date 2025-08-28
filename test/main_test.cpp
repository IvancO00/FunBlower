#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#ifndef SDA_PIN
#define SDA_PIN 21
#endif
#ifndef SCL_PIN
#define SCL_PIN 22
#endif

// Usa hardware I2C dell'ESP32 (SDA=21, SCL=22 di default)
U8G2_SSD1315_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);



void setup() {
   Serial.begin(115200);
  Wire.begin(21, 22);

  // Se serve cambiare indirizzo (ma nel tuo caso è 0x3C)
  // u8g2.setI2CAddress(0x3C*2);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, "SSD1315 OK!");
  u8g2.drawStr(0, 30, "Hello Ivan");
  u8g2.drawLine(0, 35, 127, 35);
  u8g2.sendBuffer();
}

void loop() {

}
