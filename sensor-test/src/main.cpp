#include <Arduino.h>
#include <sensor.h>

#define PIN_LED 9

Sensor sensor(false);

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
    sensor.sleep(3);

    // signal start of setup
    pinMode(PIN_LED, OUTPUT);
    flashLed(1);

    sensor.sleep(1);

    if (!sensor.init()) {
        sensor.powerDown();
        while(1) {
            flashLed(10);
            delay(10000);
        }
    }

    // signal end of setup
    flashLed(3);
}

void loop()
{
    uint8_t msg[7] = {'B', 0, 'L', 0, 0, 'S', 0};
    uint16_t voltage = sensor.readVoltage();
    msg[1] = voltage / 10 - 100;

    flashLed(1);
    sensor.powerUp();
    sensor.sendAndWait(msg, sizeof(msg));
    sensor.powerDown();
    sensor.sleep(5);
}
