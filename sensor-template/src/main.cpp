#include <Arduino.h>
#include <sensor.h>

#define PIN_LED 9

Sensor sensor;

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

void onDataReceived(const uint8_t *data, uint8_t size, uint8_t rssi)
{
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_LED, LOW);
}

void setup()
{
    pinMode(PIN_LED, OUTPUT);

    const uint8_t key[16] = {0xd9, 0xc1, 0xbd, 0x60, 0x9c, 0x35, 0x3e, 0x8b, 0xab, 0xbc, 0x8d, 0x35, 0xc7, 0x04, 0x74, 0xef};

    // on first programming of a board with flash, or when you want to change the radio setup
    // example for high power radio and flash present
    // sensor.init(99, 1, key, true, true);

    // if flash present and config already saved
    // sensor.init();

    // no flash present, low power radio
    sensor.init(99, 1, key);

    sensor.onMessage(onDataReceived);
}

void loop()
{
    static uint32_t nextSend;
    if (millis() > nextSend)
    {
        nextSend = millis() + 2000;
        if (sensor.sendAndWait((uint8_t *)"Hello 4", 7))
        {
            flashLed(10);
        }
    }

    sensor.update();
}
