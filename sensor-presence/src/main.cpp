#include <Arduino.h>
#include <sensor.h>
#include "MAX44009.h"
#include <Wire.h>

#define PIN_LED 9
#define PIN_INPUT 3

MAX44009 max44009;
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

void updateLight() {
    uint16_t light = max44009.getMeasurement();
    msg[3] = light >> 8;
    msg[4] = light;
}

void updateMotion() {
    uint8_t state = isMotionActive();
    msg[6] = state;
}

void updateAll() {
    updateVoltage();
    updateLight();
    updateMotion();
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
