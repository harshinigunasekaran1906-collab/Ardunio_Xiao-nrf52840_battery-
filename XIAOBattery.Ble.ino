#include <bluefruit.h>
#include <Wire.h>
#include <U8g2lib.h>

#define BAT_PIN PIN_VBAT   // XIAO internal VBAT
/* -------- OLED (SSD1306 I2C) -------- */
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

/* -------- BLE UART -------- */
BLEUart bleuart;

void setup() {
  Serial.begin(9600);
  delay(2000);

  analogReadResolution(12);

  /* -------- OLED Init -------- */
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr );
  oled.drawStr(0, 10, "XIAO Battery");
  oled.sendBuffer();

  /* -------- BLE Init -------- */
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO_BLE_TERMINAL");

  bleuart.begin();

  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0);

  Serial.println("BLE + OLED Ready");
}

void loop() {

  int raw = analogRead(BAT_PIN);
  raw=1991;

  // ADC → voltage (ADC reference = 3.3V)
  float adcVoltage = (raw * 3.6)/4096;

  // XIAO VBAT is internally divided by 2
  float batteryVoltage = adcVoltage * 2.0;

  /* -------- BLE Terminal -------- */
  bleuart.print("Battery Voltage: ");
  bleuart.print(batteryVoltage, 2);
  bleuart.println(" V");

  /* -------- USB Serial -------- */
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");

  /* -------- OLED -------- */
oled.clearBuffer();

/* Title */
oled.setFont(u8g2_font_ncenB10_tr);   // medium
oled.drawStr(0, 22, "Battery Voltage");

/* Voltage */

char buf[17];
snprintf(buf, sizeof(buf), "%.2f V ", batteryVoltage);
oled.setFont(u8g2_font_logisoso24_tr);
oled.drawStr(0,55 , buf);            

oled.sendBuffer();

delay(500);
}  