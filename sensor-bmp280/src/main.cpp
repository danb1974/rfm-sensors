#include <Arduino.h>
#include <sensor.h>
#include <Adafruit_BMP280.h>

#define PIN_LED 9

Sensor sensor(false);
Adafruit_BMP280 bmp;

void flashLed(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
}

void setup()
{
  // settle
  sensor.sleep(5);

  pinMode(PIN_LED, OUTPUT);
  flashLed(1);
  sensor.sleep(1);

  if (!sensor.init() || !bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
  {
    flashLed(10);
    while (1)
    {
      sensor.sleep(10000);
    }
  }

  bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X4,
      Adafruit_BMP280::STANDBY_MS_1000
  );

  flashLed(3);
}

void loop()
{
  uint8_t msg[8] = {'B', 0, 'T', 0, 0, 'P', 0, 0};

  uint16_t voltage = sensor.readVoltage();
  msg[1] = voltage / 10 - 100;

  uint16_t temperature = round(bmp.readTemperature() * 256);
  msg[3] = temperature >> 8;
  msg[4] = temperature & 0xff;

  uint16_t pressure = round(bmp.readPressure() / 10);
  msg[6] = pressure >> 8;
  msg[7] = pressure & 0xff;

  sensor.powerUp();
  sensor.sendAndWait(msg, sizeof(msg));
  sensor.powerDown();

  sensor.sleep(60);
}
