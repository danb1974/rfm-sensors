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
    // settle
    delay(3000);

    // signal start of setup
    pinMode(PIN_LED, OUTPUT);
    flashLed(1);

    Serial.begin(9600);
    const char *str = "Hello\r\n";
    Serial.write(str);
}

void loop()
{
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        Serial.write(&byte, 1);
//        Serial.write("\r\n");
    }

    flashLed(1);
    delay(100);
}
