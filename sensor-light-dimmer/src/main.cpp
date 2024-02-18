#include <Arduino.h>
#include <sensor.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define PIN_LED 9

#define PIN_ZERO 3
#define PIN_TOUCH 4
#define PIN_RB_LED 5
#define PIN_TRIAC 6

Sensor sensor(false);

#define CMD_SET 1
#define CMD_GET 2
#define CMD_SET_MODE 3
#define CMD_SET_LED 4
#define CMD_MIN_LIGHT 5

#define MODE_MAN_DIMMER 0x01  //enable manual dimming
#define MODE_DISABLE_MAN 0x02 //disable manual control

#define RSP_INIT 1
#define RSP_STATE 2

static volatile uint8_t currentBrightness = 0;
static volatile uint8_t currentState = 0;
static volatile uint16_t timerDelay = 0xFFFF;
static volatile uint8_t brightness;
static volatile uint8_t mode = 0;
static volatile uint8_t modeNoDimmerBrightness = 100;

static uint8_t ledBrightness = 255;
static uint8_t minBrightness = 0;
static uint32_t minBrightnessReset = 0;

void setRBLedBrightness(uint8_t brightness);

void flashLed(uint8_t count, uint16_t delayms = 100)
{
	for (uint8_t i = 0; i < count; i++)
	{
		digitalWrite(PIN_LED, HIGH);
		if (delayms > 0)
			delay(delayms);
		digitalWrite(PIN_LED, LOW);
		if (delayms > 0)
			delay(delayms);
	}
}

inline void updateRBLed()
{
    if (brightness)
        PORTD &= ~(1 << PIN_RB_LED);
    else
        PORTD |= 1 << PIN_RB_LED;
}

void sendState()
{
    uint8_t send[2] = {RSP_STATE, brightness};
    sensor.send(send, sizeof(send));
}

void onData(const uint8_t *data, uint8_t length, uint8_t rssi)
{
    if (data[0] == CMD_SET && length == 2)
    {
        if (data[1] > 100)
            brightness = 100;
        else
            brightness = data[1];
        updateRBLed();
    }
    else if (data[0] == CMD_GET && length == 1)
    {
        sendState();
    }
    else if (data[0] == CMD_SET_MODE && length == 3)
    {
        mode = data[1];
        modeNoDimmerBrightness = data[2];
    }
    else if (data[0] == CMD_SET_LED && length == 2)
    {
        setRBLedBrightness(data[1]);
    }
    else if (data[0] == CMD_MIN_LIGHT && length == 3)
    {
        minBrightness = data[1];
        minBrightnessReset = millis() + data[2] * 1000UL;
    }
}

// philips led 8.5W LedLustre
// from 7 to 3
static const uint16_t powerToTicks[100] = {
    13998, 13992, 13982, 13968, 13951, 13929, 13904, 13874, 13841, 13804,
    13764, 13719, 13671, 13619, 13564, 13505, 13443, 13377, 13308, 13236,
    13161, 13082, 13000, 12916, 12828, 12738, 12645, 12550, 12452, 12351,
    12248, 12143, 12036, 11927, 11816, 11703, 11589, 11472, 11355, 11236,
    11116, 10995, 10873, 10750, 10626, 10501, 10376, 10251, 10126, 10000,
     9874,  9749,  9624,  9499,  9374,  9250,  9127,  9005,  8884,  8764,
     8645,  8528,  8411,  8297,  8184,  8073,  7964,  7857,  7752,  7649,
     7548,  7450,  7355,  7262,  7172,  7084,  7000,  6918,  6839,  6764,
     6692,  6623,  6557,  6495,  6436,  6381,  6329,  6281,  6236,  6196,
     6159,  6126,  6096,  6071,  6049,  6032,  6018,  6008,  6002,  6000,
};

// osram led filament 7W
// philips led 10.5W
// from 6.2 to 3.5
// static const uint16_t powerToTicks[100] = {
//     12399, 12395, 12388, 12379, 12367, 12352, 12335, 12315, 12293, 12268,
//     12240, 12210, 12178, 12143, 12106, 12066, 12024, 11980, 11933, 11884,
//     11833, 11780, 11725, 11668, 11609, 11548, 11486, 11421, 11355, 11287,
//     11218, 11147, 11074, 11001, 10926, 10850, 10772, 10694, 10615, 10534,
//     10453, 10371, 10289, 10206, 10122, 10038,  9954,  9870,  9785,  9700,
//      9615,  9530,  9446,  9362,  9278,  9194,  9111,  9029,  8947,  8866,
//      8785,  8706,  8628,  8550,  8474,  8399,  8326,  8253,  8182,  8113,
//      8045,  7979,  7914,  7852,  7791,  7732,  7675,  7620,  7567,  7516,
//      7467,  7420,  7376,  7334,  7294,  7257,  7222,  7190,  7160,  7132,
//      7107,  7085,  7065,  7048,  7033,  7021,  7012,  7005,  7001,  7000,
// };

static volatile bool ledShouldBeOn = false;
static volatile uint32_t ledIsOnSinceUs = 0;
static volatile uint32_t lastCrossUs = 0;

// WARNING zero cross only happens each 20ms since only one of the transitions is detected
// the triac interrupt handler schedules a second triac pulse after 10ms
void zeroCross()
{
    uint32_t nowUs = micros();

    // detect and ignore noise, blink led
    if (nowUs - lastCrossUs < 2000) {
        if (!ledShouldBeOn) {
            ledShouldBeOn = true;
            ledIsOnSinceUs = nowUs;
        }

        return;
    }
    lastCrossUs = nowUs;

    // keep blinks long enough for user to see
    if (ledShouldBeOn && nowUs - ledIsOnSinceUs > 50000) {
        ledShouldBeOn = false;
    }

    uint8_t target = max(brightness, minBrightness);
    if (currentBrightness != target)
    {
        if (currentBrightness < target)
            currentBrightness++;
        else if (currentBrightness > target)
            currentBrightness--;
    }

    if (currentBrightness) {
        timerDelay = powerToTicks[currentBrightness - 1];
        currentState = 1;
    } else {
        timerDelay = 0xFFFF;
        currentState = 0;
    }

    PORTD &= ~(1 << PIN_TRIAC); // off
    TCNT1 = 0;
    OCR1A = timerDelay;
}

void setup()
{
    sensor.sleep(3); //wait for power to stabilize

    pinMode(PIN_LED, OUTPUT);
    
    if (!sensor.init()) {
        sensor.powerDown();
        while (1)
        {
            flashLed(10);
            sensor.sleep(10000);
        }
    }
    
    sensor.onMessage(onData);

    pinMode(PIN_TOUCH, INPUT);
    pinMode(PIN_ZERO, INPUT);
    pinMode(PIN_RB_LED, OUTPUT);
    pinMode(PIN_TRIAC, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ZERO), zeroCross, RISING);

    OCR1A = 0xFFFF;
    TCNT1 = 0;
    TIMSK1 = 1 << OCIE1A; //enable COMPA interrupt
    TIFR1 = 1 << OCF1A;   //clear any flag that may be there
    TCCR1A = 0;
    TCCR1B = 1 << CS11;   //(start 8 prescaler - 2MHz - 20000 "ticks" are 10ms or half sine duration)

    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = (1 << OCIE2A) | (1 << TOIE2); //enable COMPB and OVRFB

    sei();

    uint8_t announce = RSP_INIT;
    sensor.send(&announce, 1);
    wdt_enable(WDTO_2S);

    flashLed(3);
}

ISR(TIMER1_COMPA_vect)
{
    // 2000 = 1ms
    // 2300 is too small
    // 2500 seems to work with 5.5W led but not with classic
    // 3000 does not work with 25W classic, 3200 works
    #define END_OFFSET 3500

    switch (currentState)
    {
    case 1:
    case 3:
        PORTD |= 1 << PIN_TRIAC; // on
        TCNT1 = 0;
        OCR1A = 20000 - END_OFFSET - timerDelay;
        break;

    case 2:
        PORTD &= ~(1 << PIN_TRIAC); // off
        TCNT1 = 0;
        OCR1A = timerDelay + END_OFFSET;
        break;

    default:
        PORTD &= ~(1 << PIN_TRIAC); // off
        TCNT1 = 0;
        OCR1A = 0xFFFF;
    }

    if (currentState > 0) currentState++;
    if (currentState > 3) currentState = 0;
}

ISR(TIMER2_COMPA_vect)
{
    DDRD &= ~(1 << PIN_RB_LED);
    PORTD &= ~(1 << PIN_RB_LED);
}

ISR(TIMER2_OVF_vect)
{
    DDRD |= 1 << PIN_RB_LED;
    updateRBLed();
}

void setRBLedBrightness(uint8_t newLedBrightness)
{
    if (newLedBrightness == ledBrightness)
        return;

    ledBrightness = newLedBrightness;

    OCR2A = newLedBrightness;
    if (newLedBrightness == 255 || newLedBrightness == 0)
    {
        TCCR2B = 0;
        if (brightness && newLedBrightness)
            DDRD |= 1 << PIN_RB_LED;
        else
            DDRD &= ~(1 << PIN_RB_LED);
        updateRBLed();
    }
    else
    {
        TCCR2B = 1 << CS22;
    }
}

bool handleTouchEvents()
{
    uint32_t now = millis();
    uint8_t touchState = digitalRead(PIN_TOUCH);

    static uint32_t debounce;
    static uint8_t lastDebounceState = LOW;
    if (touchState != lastDebounceState)
    {
        debounce = now;
        lastDebounceState = touchState;
    }

    if (now - debounce < 10)
    {
        return false;
    }

    static bool levelChanged;
    static uint8_t lastBrightness = 100;
    static uint32_t nextManualChange;
    static bool increaseLevel = true;
    static uint8_t lastTouchState = LOW;
    uint8_t initialBrightness = brightness;
    if (touchState != lastTouchState)
    {
        lastTouchState = touchState;
        if (mode & MODE_DISABLE_MAN)
        {
            return false;
        }

        if (touchState)
        {
            if (mode & MODE_MAN_DIMMER)
            {
                nextManualChange = now + 1000;
                levelChanged = false;
                if (!brightness)
                    increaseLevel = true;
            }
            else
            {
                if (brightness < modeNoDimmerBrightness)
                {
                    brightness = modeNoDimmerBrightness;
                }
                else
                {
                    brightness = 0;
                }
                updateRBLed();
            }
        }
        else
        {
            if (mode & MODE_MAN_DIMMER)
            {
                if (!levelChanged)
                {
                    if (brightness)
                    {
                        lastBrightness = brightness;
                        brightness = 0;
                        increaseLevel = true;
                    }
                    else
                    {
                        brightness = lastBrightness;
                        increaseLevel = false;
                    }
                }
                else
                {
                    increaseLevel = !increaseLevel;
                }
            }
            updateRBLed();
        }
    }
    else if (touchState)
    {
        if (mode & MODE_MAN_DIMMER)
        {
            if (now >= nextManualChange)
            {
                levelChanged = true;
                nextManualChange = now + 15;
                if (increaseLevel)
                {
                    if (brightness < 100)
                        brightness++;
                }
                else
                {
                    if (brightness > 1)
                        brightness--;
                }
            }
        }
    }
    return initialBrightness != brightness;
}

static bool ledIsOn = false;

void loop()
{
    wdt_reset();
    auto now = millis();

    if (minBrightness && now >= minBrightnessReset)
    {
        minBrightness = 0;
        minBrightnessReset = 0;
    }

    if (handleTouchEvents())
    {
        sendState();
    }

    if (ledShouldBeOn != ledIsOn) {
        digitalWrite(PIN_LED, ledShouldBeOn ? HIGH : LOW);
        ledIsOn = ledShouldBeOn;
    }

    sensor.update();
}
