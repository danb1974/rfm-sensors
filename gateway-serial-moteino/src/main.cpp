#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>

#define PIN_LED 9

#define SERIAL_RATE 9600

#define FRAME_HEADER_1 0xDE
#define FRAME_HEADER_2 0x5B

#define FRAME_CONFIGURE 0x90
#define FRAME_CONFIGURED 0x91
#define FRAME_SENDPACKET 0x92
#define FRAME_PACKETSENT 0x93
#define FRAME_RECEIVEPACKET 0x94
#define FRAME_INIT 0x95
#define FRAME_HB 0x96

#define FRAME_ERR_INVALID_SIZE 0x71
#define FRAME_ERR_BUSY 0x72
#define FRAME_ERR_ADDR 0x73
#define FRAME_ERR_MEM 0x74
#define FRAME_ERR_TIMEOUT 0x75

#define CHKSUM_INIT 0x1021

#define SEND_RETRIES 5
#define RETRY_INTERVAL 200

typedef struct
{
	uint32_t nextSendNonce;
	uint32_t oldReceiveNonce, nextReceiveNonce;
} SensorState;

typedef enum
{
	Data = 0x01,
	Ack = 0x02,
	Nack = 0x03,
} MsgType;

#define MIN_ADDR 4
#define MAX_ADDR 63
#define SENSORS (MAX_ADDR - MIN_ADDR + 1)
static SensorState sensors[SENSORS];

#define RADIO_QUEUE_SIZE 5
static RfmPacket radioQueue[RADIO_QUEUE_SIZE];
static volatile uint8_t radioTail = 0, radioCount = 0;
static uint8_t radioHead = 0;

uint32_t lastSendTime;
uint8_t sendBuffer[RF69_MAX_DATA_LEN], sendSize, sendRetries;
uint8_t sendTo;

#define TX_PACKET_SIZE 60
typedef struct
{
	uint8_t data[TX_PACKET_SIZE];
	uint8_t size;
	uint8_t sent;
} TxSerial;

#define RX_PACKET_SIZE 60
typedef struct
{
	uint8_t data[RX_PACKET_SIZE];
	uint8_t size;
} RxSerial;

#define SERIAL_RX_QUEUE_SIZE 5
RxSerial serialRxQueue[SERIAL_RX_QUEUE_SIZE];
volatile uint8_t serialRxTail = 0, serialRxCount = 0;
uint8_t serialRxHead = 0;

typedef enum
{
	Idle,
	Hdr,
	Size,
	Payload,
	Chk1,
	Chk2
} RxStatus;

void spi_Transfer(uint8_t *data, uint8_t len)
{
	digitalWrite(SS, LOW);
	while (len--)
	{
		*data = SPI.transfer(*data);
		data++;
	}
	digitalWrite(SS, HIGH);
}

RFM69 radio(spi_Transfer, millis);

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

static void updateChecksum(uint16_t *checksum, uint8_t data)
{
	bool roll = *checksum & 0x8000 ? true : false;
	*checksum <<= 1;
	if (roll)
		*checksum |= 1;
	*checksum ^= data;
}

static uint16_t getChecksum(uint8_t *data, uint8_t length)
{
	uint16_t checksum = CHKSUM_INIT;
	while (length--)
	{
		updateChecksum(&checksum, *data++);
	}
	return checksum;
}

void handleSerialData()
{
	static RxStatus status = RxStatus::Idle;
	static uint8_t offset;
	static uint16_t chksum, rxChksum;
	static uint32_t last;

	if (!Serial.available())
	{
		return;
	}

	uint8_t data = Serial.read();

	uint32_t now = millis();
	if (now - last > 300)
	{
		status = RxStatus::Idle;
	}
	last = now;

	switch (status)
	{
	case RxStatus::Idle:
		if (data == FRAME_HEADER_1)
			status = RxStatus::Hdr;
		break;
	case RxStatus::Hdr:
		status = data == FRAME_HEADER_2 ? RxStatus::Size : RxStatus::Idle;
		break;
	case RxStatus::Size:
		if (serialRxCount == SERIAL_RX_QUEUE_SIZE || data > RX_PACKET_SIZE)
		{
			status = RxStatus::Idle;
			break;
		}
		serialRxQueue[serialRxTail].size = data;
		status = RxStatus::Payload;
		chksum = CHKSUM_INIT;
		offset = 0;
		updateChecksum(&chksum, data);
		break;
	case RxStatus::Payload:
		updateChecksum(&chksum, data);
		serialRxQueue[serialRxTail].data[offset++] = data;
		if (offset == serialRxQueue[serialRxTail].size)
			status = RxStatus::Chk1;
		break;
	case RxStatus::Chk1:
		rxChksum = data << 8;
		status = RxStatus::Chk2;
		break;
	case RxStatus::Chk2:
		rxChksum |= data;
		if (chksum == rxChksum)
		{
			noInterrupts();
			if (++serialRxTail == SERIAL_RX_QUEUE_SIZE)
				serialRxTail = 0;
			serialRxCount++;
			interrupts();
		}
		status = RxStatus::Idle;
		break;
	}
}

void serialSendFrame(uint8_t head, uint8_t from, const uint8_t *data, uint8_t size)
{
	uint8_t packet[5 + size + 2];
	packet[0] = FRAME_HEADER_1;
	packet[1] = FRAME_HEADER_2;
	packet[2] = size + 2;
	packet[3] = head;
	packet[4] = from;
	if (size)
	{
		memcpy(&packet[5], data, size);
	}

	uint16_t checksum = getChecksum(&packet[2], size + 3);
	packet[5 + size] = checksum >> 8;
	packet[6 + size] = checksum;

	Serial.write(packet, sizeof(packet));
}

//-----------------------------------------------------------------------------

uint32_t readNonce(const uint8_t *data)
{
	uint32_t nonce = *data++;
	nonce |= (uint32_t)*data++ << 8;
	nonce |= (uint32_t)*data++ << 16;
	nonce |= (uint32_t)*data++ << 24;
	return nonce;
}

uint16_t readUint16_t(const uint8_t *data)
{
	uint16_t d = *data++;
	d |= (uint16_t)*data++ << 8;
	return d;
}

void writeNonce(uint8_t *data, uint32_t nonce)
{
	*data++ = nonce;
	*data++ = nonce >> 8;
	*data++ = nonce >> 16;
	*data++ = nonce >> 24;
}

uint32_t createNonce()
{
	uint32_t nonce = rand();
	nonce |= (uint32_t)rand() << 16;
	return nonce;
}

//-----------------------------------------------------------------------------

bool inited = false;

void radioInterrupt()
{
	if (!inited)
		return;

	RfmPacket packet;
	while (radio.receive(packet))
	{
		if (packet.from < MIN_ADDR || packet.from > MAX_ADDR || radioCount == RADIO_QUEUE_SIZE)
			continue;

		radioQueue[radioTail] = packet;

		radioCount++;
		if (++radioTail == RADIO_QUEUE_SIZE)
			radioTail = 0;
	}
}

void sendRadioDone()
{
	sendRetries = 0;
	sendSize = 0;
}

void sendRadioNow()
{
	SensorState &sensor = sensors[sendTo - MIN_ADDR];
	writeNonce(&sendBuffer[1], sensor.nextSendNonce);
	noInterrupts();
	radio.send(sendTo, sendBuffer, sendSize);
	interrupts();
	lastSendTime = millis();
}

uint8_t sendRadio(uint8_t to, const uint8_t *data, uint8_t size)
{
	if (sendRetries)
		return FRAME_ERR_BUSY;

	sendTo = to;
	sendSize = size + 5;
	sendBuffer[0] = MsgType::Data;
	sendRetries = SEND_RETRIES;
	memcpy(&sendBuffer[5], data, size);
	sendRadioNow();
	return 0;
}

void sendResponse(SensorState &sensor, uint8_t to, uint8_t rssi, uint32_t nonce, bool ack)
{
	uint8_t data[10] = {ack ? MsgType::Ack : MsgType::Nack};
	writeNonce(&data[1], nonce);
	writeNonce(&data[5], sensor.nextReceiveNonce);
	data[9] = rssi;
	noInterrupts();
	radio.send(to, data, sizeof(data));
	interrupts();
}

void onSerialPacketReceived(const uint8_t *data, uint8_t size)
{
	size--;
	switch (*data++)
	{
	case FRAME_HB:
		serialSendFrame(FRAME_HB, 0, NULL, 0);
		break;
	case FRAME_SENDPACKET:
	{
		size--;
		uint8_t to = *data++;
		if (to < MIN_ADDR || to > MAX_ADDR)
		{
			serialSendFrame(FRAME_ERR_ADDR, to, NULL, 0);
			return;
		}
		if (size > RF69_MAX_DATA_LEN - 5)
		{
			serialSendFrame(FRAME_ERR_INVALID_SIZE, to, NULL, 0);
			return;
		}
		uint8_t err = sendRadio(to, data, size);
		if (err != 0)
		{
			serialSendFrame(err, to, NULL, 0);
		}
	}
	break;
	case FRAME_CONFIGURE:
	{
		while (size)
		{
			size--;
			switch (*data++)
			{
			case 'K': // encryption key
				radio.encrypt(data);
				data += 16;
				size -= 16;
				break;
			case 'F': // frequency
				radio.setFrequency(readNonce(data));
				data += 4;
				size -= 4;
				break;
			case 'N':
				radio.setAddress(*data++);
				size--;
				break;
			case 'P':
				radio.setPowerLevel(*data++);
				size--;
				break;
			case 'R':
				srand(readUint16_t(data));
				data += 2;
				size -= 2;
				break;
			default:
				size = 0;
				break;
			}
		}
		serialSendFrame(FRAME_CONFIGURED, 0, NULL, 0);
	}
	break;
	}
}

void onRadioPacketReceived(RfmPacket &packet)
{
	if (packet.from < MIN_ADDR || packet.from > MAX_ADDR)
		return;

	SensorState &sensor = sensors[packet.from - MIN_ADDR];
	auto data = packet.data;
	auto size = packet.size;

	size--;
	switch (*data++)
	{
	case MsgType::Data:
	{
		uint32_t nonce = readNonce(data);
		if (nonce == sensor.oldReceiveNonce)
		{
			// already received this data
			sendResponse(sensor, packet.from, packet.rssi, nonce, true);
			break;
		}

		if (nonce != sensor.nextReceiveNonce)
		{
			// this is not what we're expecting
			sendResponse(sensor, packet.from, packet.rssi, nonce, false);
			break;
		}

		data += 4; // skip nonce
		size -= 4;

		sensor.oldReceiveNonce = sensor.nextReceiveNonce;
		do
		{
			sensor.nextReceiveNonce = createNonce();
		} while (sensor.nextReceiveNonce == sensor.oldReceiveNonce);
		sendResponse(sensor, packet.from, packet.rssi, nonce, true);
		serialSendFrame(FRAME_RECEIVEPACKET, packet.from, data, size);
	}
	break;
	case MsgType::Ack:
	{
		uint32_t ackNonce = readNonce(data);
		if (size != 9 || ackNonce != sensor.nextSendNonce)
			break;

		sensor.nextSendNonce = readNonce(&data[4]);
		sendRadioDone();
		serialSendFrame(FRAME_PACKETSENT, packet.from, NULL, 0);
	}
	break;
	case MsgType::Nack:
	{
		uint32_t nackNonce = readNonce(data);
		if (size != 9 || nackNonce != sensor.nextSendNonce)
			break;

		sensor.nextSendNonce = readNonce(&data[4]);
		sendRadioNow();
	}
	break;
	}
}

//-----------------------------------------------------------------------------

void setup()
{
	delay(5000);
	flashLed(1);

	Serial.begin(SERIAL_RATE);

	for (uint8_t i = 0; i < SENSORS; i++)
	{
		sensors[i].oldReceiveNonce = createNonce();
		sensors[i].nextReceiveNonce = createNonce();
		sensors[i].nextSendNonce = createNonce();
	}

	inited = radio.initialize(RF69_433MHZ, 1, 1, true);
	if (!inited)
	{
		while (1)
		{
			flashLed(1);
			delay(1);
		}
	}

	attachInterrupt(0, radioInterrupt, RISING);
	serialSendFrame(FRAME_INIT, 0, NULL, 0);

	flashLed(3);
}

//-----------------------------------------------------------------------------

void loop()
{
	handleSerialData();

	while (radioCount != 0)
	{
		RfmPacket &rx = radioQueue[radioHead];
		onRadioPacketReceived(rx);
		noInterrupts();
		radioCount--;
		if (++radioHead == RADIO_QUEUE_SIZE)
			radioHead = 0;
		interrupts();
	}

	while (serialRxCount != 0)
	{
		RxSerial &rx = serialRxQueue[serialRxHead];
		onSerialPacketReceived(rx.data, rx.size);
		noInterrupts();
		serialRxCount--;
		if (++serialRxHead == SERIAL_RX_QUEUE_SIZE)
			serialRxHead = 0;
		interrupts();
	}

	if (sendRetries && millis() - lastSendTime >= RETRY_INTERVAL)
	{
		sendRetries--;
		if (sendRetries == 0)
		{
			sendRadioDone();
			serialSendFrame(FRAME_ERR_TIMEOUT, sendTo, NULL, 0);
		}
		else
		{
			sendRadioNow();
		}
	}
}