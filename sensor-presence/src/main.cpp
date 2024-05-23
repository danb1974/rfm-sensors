#include <Arduino.h>
#include <sensor.h>
#include <Wire.h>

#ifndef NO_LIGHT_SENSOR
#include "MAX44009.h"
MAX44009 max44009;
#endif

#define PIN_LED 9
#define PIN_INPUT 3

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

void motionDetected()
{
    sensor.wake();
}

void setup()
{
    // settle
    sensor.sleep(5);

    // signal start of setup
    pinMode(PIN_LED, OUTPUT);
    flashLed(1);
    sensor.sleep(1);

    pinMode(PIN_INPUT, INPUT);

    digitalWrite(A4, HIGH);
    digitalWrite(A5, HIGH);
    Wire.begin();

    sensor.init();

#ifndef NO_LIGHT_SENSOR
    if (!max44009.initialize())
    {
        // signal max44009 error
        flashLed(10);
        sensor.powerDown();
        while (1)
        {
            sensor.sleep(10000);
        }
    }
#endif

    // signal end of setup
    flashLed(3);
}

bool isMotionActive() {
    return digitalRead(PIN_INPUT);
}

static uint8_t msg[7] = {'B', 0, 'L', 0, 0, 'S', 0};

void updateVoltage() {
    uint16_t voltage = sensor.readVoltage();
    msg[1] = voltage / 10 - 100;
}

void updateMotion() {
    uint8_t state = isMotionActive();
    msg[6] = state;
}

#ifndef NO_LIGHT_SENSOR
void updateLight() {
    uint16_t light = max44009.getMeasurement();
    msg[3] = light >> 8;
    msg[4] = light;
}
#endif

void updateAll() {
    updateVoltage();
    updateMotion();
#ifndef NO_LIGHT_SENSOR
    updateLight();
#endif
}

void loop()
{
    do {
        updateAll();

        sensor.powerUp();
        sensor.sendAndWait(msg, sizeof(msg));
        sensor.powerDown();
        sensor.sleep(30);
    } while (isMotionActive());

    EIFR = 0x03; // clear INT0/INT1
    attachInterrupt(digitalPinToInterrupt(PIN_INPUT), motionDetected, RISING);
    sensor.sleep(1800);
    detachInterrupt(digitalPinToInterrupt(PIN_INPUT));
}
