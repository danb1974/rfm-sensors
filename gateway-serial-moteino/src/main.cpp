#include <Arduino.h>

#define SERIAL_RATE 9600

#define FRAME_HEADER_1  0xDE
#define FRAME_HEADER_2  0x5B
#define CHKSUM_INIT     0x1021

#define TX_PACKET_SIZE	60
typedef struct {
	uint8_t data[TX_PACKET_SIZE];
	uint8_t size;
	uint8_t sent;
} TxSerial;

#define RX_PACKET_SIZE	60
typedef struct {
	uint8_t data[RX_PACKET_SIZE];
	uint8_t size;
} RxSerial;

#define SERIAL_TX_QUEUE_SIZE	5
static TxSerial serialTxQueue[SERIAL_TX_QUEUE_SIZE];
static volatile uint8_t serialTxHead = 0, serialTxCount = 0;
static uint8_t serialTxTail = 0;

#define SERIAL_RX_QUEUE_SIZE	5
RxSerial serialRxQueue[SERIAL_RX_QUEUE_SIZE];
volatile uint8_t serialRxTail = 0, serialRxCount = 0;
uint8_t serialRxHead = 0;

typedef enum {
	Idle, Hdr, Size, Data, Chk1, Chk2
} RxStatus;

//-----------------------------------------------------------------------------

static void updateChecksum(uint16_t *checksum, uint8_t data) {
	bool roll = *checksum & 0x8000 ? true : false;
	*checksum <<= 1;
	if (roll)
		*checksum |= 1;
	*checksum ^= data;
}

static uint16_t getChecksum(uint8_t *data, uint8_t length) {
	uint16_t checksum = CHKSUM_INIT;
	while (length--) {
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

  if (!Serial.available()) {
    return;
  }

  uint8_t data = Serial.read();
	
	uint32_t now = millis();
	if (now - last > 300) {
		status = RxStatus::Idle;
	}
	last = now;
	
	switch (status) {
	case RxStatus::Idle:
		if (data == FRAME_HEADER_1)
			status = RxStatus::Hdr;
		break;
	case RxStatus::Hdr:
		status = data == FRAME_HEADER_2 ? RxStatus::Size : RxStatus::Idle;
		break;
	case RxStatus::Size:
		if (serialRxCount == SERIAL_RX_QUEUE_SIZE || data > RX_PACKET_SIZE) {
			status = RxStatus::Idle;
			break;
		}
		serialRxQueue[serialRxTail].size = data;
		status = RxStatus::Data;
		chksum = CHKSUM_INIT;
		offset = 0;
		updateChecksum(&chksum, data);
		break;
	case RxStatus::Data:
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
		if (chksum == rxChksum) {
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

//-----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_RATE);
}

//-----------------------------------------------------------------------------

void loop() {
  
}
