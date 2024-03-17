#include <Arduino.h>
#include <sensor.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define BLINK_ON_ZERO_CROSS_ERRORS

// early zero cross threshold
#define IGNORE_ZERO_BEFORE_USEC 19500
// out of sync zero cross threshold
#define IGNORE_ZERO_PRECISION_USEC 1000
// how many full cycles to go without being synced to zero cross
#define UNSYNCED_CYCLES 10

// 2000 = 1ms
#define HALF_PERIOD_TICKS 20000
// turn off triac before end of period
// the zero cross interrupt is delayed by probably about 0.5ms but zero cross depends on bulb current curve
// 20M:1.2M resistive divider from 324V sine peak considering input transition at 2.5V
#define TRIAC_OFF_BEFORE_TICKS 4000

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
#define CMD_SET_CURVE 6

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

static const uint8_t powerCurveCount = 2;
static uint8_t powerCurveIndex = 0;
static const uint16_t powerToTicks[powerCurveCount][100] = {
    // Philips LedLustre, sinus + gamma curve
    // from 7 to 3.5 algo 1
    {
        13998, 13993, 13984, 13972, 13957, 13938, 13916, 13890, 13861, 13829,
        13793, 13754, 13712, 13667, 13619, 13567, 13513, 13455, 13395, 13332,
        13266, 13197, 13125, 13051, 12975, 12896, 12815, 12731, 12645, 12557,
        12467, 12375, 12282, 12186, 12089, 11990, 11890, 11788, 11686, 11582,
        11476, 11370, 11264, 11156, 11048, 10939, 10829, 10720, 10610, 10500,
        10390, 10280, 10171, 10061,  9952,  9844,  9736,  9630,  9524,  9418,
        9314,  9212,  9110,  9010,  8911,  8814,  8718,  8625,  8533,  8443,
        8355,  8269,  8185,  8104,  8025,  7949,  7875,  7803,  7734,  7668,
        7605,  7545,  7487,  7433,  7381,  7333,  7288,  7246,  7207,  7171,
        7139,  7110,  7084,  7062,  7043,  7028,  7016,  7007,  7002,  7000,
    },
    // Generic modern led, linear curve
    // from 6.2 to 3.5 algo 3
    {
        12346, 12292, 12238, 12184, 12130, 12076, 12022, 11968, 11914, 11860,
        11806, 11752, 11698, 11644, 11590, 11536, 11482, 11428, 11374, 11320,
        11266, 11212, 11158, 11104, 11050, 10996, 10942, 10888, 10834, 10780,
        10726, 10672, 10618, 10564, 10510, 10456, 10402, 10348, 10294, 10240,
        10186, 10132, 10078, 10024,  9970,  9916,  9862,  9808,  9754,  9700,
        9646,  9592,  9538,  9484,  9430,  9376,  9322,  9268,  9214,  9160,
        9106,  9052,  8998,  8944,  8890,  8836,  8782,  8728,  8674,  8620,
        8566,  8512,  8458,  8404,  8350,  8296,  8242,  8188,  8134,  8080,
        8026,  7972,  7918,  7864,  7810,  7756,  7702,  7648,  7594,  7540,
        7486,  7432,  7378,  7324,  7270,  7216,  7162,  7108,  7054,  7000,
    }
};

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
    else if (data[0] == CMD_SET_CURVE && length == 2)
    {
        if (data[1] >= 1 && data[1] <= powerCurveCount) {
            powerCurveIndex = data[1] - 1;
        }
    }
}

static volatile bool triacIsOn = false;

void triacOn() {
    PORTD |= 1 << PIN_TRIAC; // on
    triacIsOn = true;
}

void triacOff() {
    PORTD &= ~(1 << PIN_TRIAC); // off
    triacIsOn = false;
}

#ifdef BLINK_ON_ZERO_CROSS_ERRORS
static volatile bool ledShouldBeOn = false;
static volatile uint32_t ledIsOnSinceUs = 0;
#endif

#define ZERO_CROSS_PULSES 10
static volatile uint8_t zeroCrossPulseIdx = 0;
static volatile uint16_t zeroCrossPulses[ZERO_CROSS_PULSES];

void initZeroCrossPulses() {
    for (uint8_t i = 0; i < ZERO_CROSS_PULSES; i++) {
        zeroCrossPulses[i] = 0;
    }
}

#define PULSE_USEC_DIVIDER 256

bool validPulse(unsigned long pulseLong) {
    uint16_t pulse = (pulseLong / PULSE_USEC_DIVIDER) & 0xffff;

    // no zero values please
    if (pulse == 0) {
        pulse = 1;
    }

    uint8_t hits = 0;
    for (uint8_t i = 0; i < ZERO_CROSS_PULSES; i++) {
        uint16_t zeroCrossPulse = zeroCrossPulses[i];

        // compute first to keep loop timing as constant as possible
        uint16_t offset = abs(pulse - zeroCrossPulse) % (20000 / PULSE_USEC_DIVIDER);
        if (offset >= 10000 / PULSE_USEC_DIVIDER) {
            offset = 20000 / PULSE_USEC_DIVIDER - offset;
        }

        if (zeroCrossPulse == 0) {
            // not enough data, handle as if it were
            hits++;
            continue;
        } else if (pulse == zeroCrossPulse) {
            // this is us, ignore
            continue;
        } else if (offset < IGNORE_ZERO_PRECISION_USEC / PULSE_USEC_DIVIDER) {
            // this one looks legit
            hits++;
        }
    }

    return hits > ZERO_CROSS_PULSES / 2;
}

void storePulse(unsigned long pulseLong) {
    uint16_t pulse = (pulseLong / PULSE_USEC_DIVIDER) & 0xffff;

    // no zero values please
    if (pulse == 0) {
        pulse = 1;
    }

    zeroCrossPulses[zeroCrossPulseIdx++] = pulse;
    if (zeroCrossPulseIdx >= ZERO_CROSS_PULSES) {
        zeroCrossPulseIdx = 0;
    }
}

static volatile uint32_t lastCrossUs = 0;

// WARNING zero cross only happens each 20ms since only one of the transitions is detected
// the triac interrupt handler schedules a second triac pulse after 10ms
void zeroCross()
{
    uint32_t nowUs = micros();

    storePulse(nowUs);

    if (nowUs - lastCrossUs < IGNORE_ZERO_BEFORE_USEC || !validPulse(nowUs)) {
        #ifdef BLINK_ON_ZERO_CROSS_ERRORS
        if (!ledShouldBeOn) {
            ledShouldBeOn = true;
            ledIsOnSinceUs = nowUs;
        }
        #endif

        return;
    }

    lastCrossUs = nowUs;

    // keep blinks long enough for user to see
    #ifdef BLINK_ON_ZERO_CROSS_ERRORS
    if (ledShouldBeOn && nowUs - ledIsOnSinceUs > 20000) {
        ledShouldBeOn = false;
    }
    #endif

    uint8_t target = max(brightness, minBrightness);
    if (currentBrightness != target)
    {
        if (currentBrightness < target)
            currentBrightness++;
        else if (currentBrightness > target)
            currentBrightness--;
    }

    if (currentBrightness) {
        timerDelay = powerToTicks[powerCurveIndex][currentBrightness - 1];
        currentState = 1;
    } else {
        timerDelay = 0xFFFF;
        currentState = 0;
    }

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

    initZeroCrossPulses();
    
    sensor.onMessage(onData);

    pinMode(PIN_TOUCH, INPUT);
    pinMode(PIN_ZERO, INPUT);
    pinMode(PIN_RB_LED, OUTPUT);
    pinMode(PIN_TRIAC, OUTPUT);

    OCR1A = 0xFFFF;
    TCNT1 = 0;
    TIMSK1 = 1 << OCIE1A; //enable COMPA interrupt
    TIFR1 = 1 << OCF1A;   //clear any flag that may be there
    TCCR1A = 0;
    TCCR1B = 1 << CS11;   //(start 8 prescaler - 2MHz - 20000 "ticks" are 10ms or half sine duration)

    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = (1 << OCIE2A) | (1 << TOIE2); //enable COMPB and OVRFB

    attachInterrupt(digitalPinToInterrupt(PIN_ZERO), zeroCross, RISING);

    uint8_t announce = RSP_INIT;
    sensor.send(&announce, 1);

    wdt_enable(WDTO_2S);

    flashLed(3);
}

ISR(TIMER1_COMPA_vect)
{
    if (currentState == 0) {
        triacOff();
        
        TCNT1 = 0;
        OCR1A = 0xFFFF;

        return;
    }

    if ((currentState & 0x01) == 1) {
        triacOn();

        TCNT1 = 0;
        OCR1A = HALF_PERIOD_TICKS - TRIAC_OFF_BEFORE_TICKS - timerDelay;
    } else {
        triacOff();

        TCNT1 = 0;
        OCR1A = timerDelay + TRIAC_OFF_BEFORE_TICKS;
    }

    currentState++;
    if (currentState > 3 + 4 * UNSYNCED_CYCLES) {
        currentState = 0;
    }
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

#ifdef BLINK_ON_ZERO_CROSS_ERRORS
static bool ledIsOn = false;
#endif

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

    #ifdef BLINK_ON_ZERO_CROSS_ERRORS
    if (ledShouldBeOn != ledIsOn) {
        digitalWrite(PIN_LED, ledShouldBeOn ? HIGH : LOW);
        ledIsOn = ledShouldBeOn;
    }
    #endif

    sensor.update();
}
