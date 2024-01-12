#include <Arduino.h>

#define PIN_LED 9

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
    pinMode(PIN_LED, OUTPUT);
}

void loop()
{
    flashLed(3);
    delay(1000);
}
