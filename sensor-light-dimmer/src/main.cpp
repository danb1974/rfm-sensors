#include <Arduino.h>
#include <sensor.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

//#define BLINK_ON_ZERO_CROSS_ERRORS

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
// (when using) length of short pulses to turn triac on (classic bulbs only)
#define SHORT_TRIAC_PULSE 200

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
static volatile uint8_t triacShortPulseMode = 0;

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

static const uint8_t powerCurveCount = 6;
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
    },

    // Philips LedLustre, sinus + gamma curve
    // from 7 to 3.8 algo 1
    {
        13998, 13994, 13986, 13975, 13961, 13943, 13923, 13899, 13873, 13843,
        13811, 13775, 13737, 13695, 13651, 13604, 13554, 13502, 13447, 13389,
        13328, 13266, 13200, 13133, 13063, 12991, 12916, 12840, 12761, 12681,
        12599, 12515, 12429, 12342, 12253, 12162, 12071, 11978, 11884, 11789,
        11693, 11596, 11498, 11400, 11301, 11201, 11101, 11001, 10901, 10800,
        10699, 10599, 10499, 10399, 10299, 10200, 10102, 10004,  9907,  9811,
         9716,  9622,  9529,  9438,  9347,  9258,  9171,  9085,  9001,  8919,
         8839,  8760,  8684,  8609,  8537,  8467,  8400,  8334,  8272,  8211,
         8153,  8098,  8046,  7996,  7949,  7905,  7863,  7825,  7789,  7757,
         7727,  7701,  7677,  7657,  7639,  7625,  7614,  7606,  7602,  7600,
    },

    // Generic modern led, linear curve
    // from 6.2 to 3.8 algo 3
    {
        12352, 12304, 12256, 12208, 12160, 12112, 12064, 12016, 11968, 11920,
        11872, 11824, 11776, 11728, 11680, 11632, 11584, 11536, 11488, 11440,
        11392, 11344, 11296, 11248, 11200, 11152, 11104, 11056, 11008, 10960,
        10912, 10864, 10816, 10768, 10720, 10672, 10624, 10576, 10528, 10480,
        10432, 10384, 10336, 10288, 10240, 10192, 10144, 10096, 10048, 10000,
         9952,  9904,  9856,  9808,  9760,  9712,  9664,  9616,  9568,  9520,
         9472,  9424,  9376,  9328,  9280,  9232,  9184,  9136,  9088,  9040,
         8992,  8944,  8896,  8848,  8800,  8752,  8704,  8656,  8608,  8560,
         8512,  8464,  8416,  8368,  8320,  8272,  8224,  8176,  8128,  8080,
         8032,  7984,  7936,  7888,  7840,  7792,  7744,  7696,  7648,  7600,
    },

    // Philips LedLustre, sinus + gamma curve
    // from 7 to 3 algo 1
    {
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
    },

    // Generic modern led, linear curve
    // from 6.2 to 3 algo 3
    {
        12336, 12272, 12208, 12144, 12080, 12016, 11952, 11888, 11824, 11760,
        11696, 11632, 11568, 11504, 11440, 11376, 11312, 11248, 11184, 11120,
        11056, 10992, 10928, 10864, 10800, 10736, 10672, 10608, 10544, 10480,
        10416, 10352, 10288, 10224, 10160, 10096, 10032,  9968,  9904,  9840,
         9776,  9712,  9648,  9584,  9520,  9456,  9392,  9328,  9264,  9200,
         9136,  9072,  9008,  8944,  8880,  8816,  8752,  8688,  8624,  8560,
         8496,  8432,  8368,  8304,  8240,  8176,  8112,  8048,  7984,  7920,
         7856,  7792,  7728,  7664,  7600,  7536,  7472,  7408,  7344,  7280,
         7216,  7152,  7088,  7024,  6960,  6896,  6832,  6768,  6704,  6640,
         6576,  6512,  6448,  6384,  6320,  6256,  6192,  6128,  6064,  6000,
    }
};

void onData(const uint8_t *data, uint8_t length, uint8_t rssi)
{
    for (uint8_t offset = 0; offset < length; offset++) {
        switch (data[offset])
        {
            case CMD_SET:
                brightness = data[offset + 1];
                if (brightness > 100) {
                    brightness = 100;
                }
                offset += 1;
                break;

            case CMD_GET:
                sendState();
                break;

            case CMD_SET_MODE:
                mode = data[offset + 1];
                modeNoDimmerBrightness = data[offset + 2];
                if (modeNoDimmerBrightness > 100) {
                    modeNoDimmerBrightness = 100;
                }
                offset += 2;
                break;

            case CMD_SET_LED:
                setRBLedBrightness(data[offset + 1]);
                offset += 1;
                break;

            case CMD_MIN_LIGHT:
                minBrightness = data[offset + 1];
                if (minBrightness > 100) {
                    minBrightness = 100;
                }
                minBrightnessReset = millis() + data[offset + 2] * 1000UL;
                offset += 2;
                break;

            case CMD_SET_CURVE:
                powerCurveIndex = data[offset + 1] - 1;

                triacShortPulseMode = 0;
                if (powerCurveIndex > 128) {
                    powerCurveIndex -= 128;
                    triacShortPulseMode = 1;
                }

                if (powerCurveIndex > powerCurveCount - 1) {
                    powerCurveIndex = 0;
                }
                offset += 1;
                break;
                
            default:
                // command not supported, skip rest of buffer
                offset = length;
        }
    }
}

inline void triacOn() {
    PORTD |= 1 << PIN_TRIAC; // on
}

inline void triacOff() {
    PORTD &= ~(1 << PIN_TRIAC); // off
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

bool validPulse(uint16_t pulse) {
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

void storePulse(uint16_t pulse) {
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
static volatile uint8_t crossSequence = 0;

// WARNING zero cross only happens each 20ms since only one of the transitions is detected
// the triac interrupt handler schedules a second triac pulse after 10ms
void zeroCross()
{
    uint32_t nowUs = micros();
    uint16_t pulse = (nowUs / PULSE_USEC_DIVIDER) & 0xffff;

    storePulse(pulse);

    if (nowUs - lastCrossUs < IGNORE_ZERO_BEFORE_USEC || !validPulse(pulse)) {
        #ifdef BLINK_ON_ZERO_CROSS_ERRORS
        if (!ledShouldBeOn) {
            ledShouldBeOn = true;
            ledIsOnSinceUs = nowUs;
        }
        #endif

        return;
    }

    lastCrossUs = nowUs;
    crossSequence++;

    // keep blinks long enough for user to see
    #ifdef BLINK_ON_ZERO_CROSS_ERRORS
    if (ledShouldBeOn && nowUs - ledIsOnSinceUs > 20000) {
        ledShouldBeOn = false;
    }
    #endif

    uint8_t target = max(brightness, minBrightness);
    if (currentBrightness != target/* && crossSequence & 0x01*/)
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

    // compensate computing delay (from start of interrupt 'till now)
    // 2 pulses = 1us
    uint32_t delayPulses = (micros() - nowUs) * 2;
    TCNT1 = 0;
    OCR1A = timerDelay - delayPulses;
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
        if (triacShortPulseMode) {
            OCR1A = SHORT_TRIAC_PULSE;
        } else {
            OCR1A = HALF_PERIOD_TICKS - TRIAC_OFF_BEFORE_TICKS - timerDelay;
        }
    } else {
        triacOff();

        TCNT1 = 0;
        if (triacShortPulseMode) {
            OCR1A = HALF_PERIOD_TICKS - SHORT_TRIAC_PULSE;
        } else {
            OCR1A = timerDelay + TRIAC_OFF_BEFORE_TICKS;
        }
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
