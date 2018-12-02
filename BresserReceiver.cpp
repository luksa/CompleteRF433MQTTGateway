//===================================================================
//
// short low = 0
// long low = 1
// Timings: 3875,515,956,503,1944,505,957,503,961,497,1951,497,966,491,970,505,958,504,1942,489,976,503,959,499,1944,491,971,507,969,491,968,486,973,497,1948,503,1928,519,1928,508,1923,530,1917,516,946,512,1936,516,1912,539,1910,522,1910,541,1906,526,1904,547,931,530,932,527,1903,549,1898,535,927,548,928,533,1900,551,1894,538,3852,
// Timings: 3835,569,886,571,1877,555,907,569,907,555,1876,571,891,569,906,553,911,550,1880,563,915,549,911,550,1897,552,909,551,911,565,895,563,913,547,1884,563,1885,551,1881,568,1881,549,1896,553,909,550,1882,569,1878,554,1894,552,1879,572,1877,557,1876,571,906,556,903,556,1878,572,1874,555,907,569,891,568,1880,553,1895,554,3834,
// Timings: 3832,571,899,560,1873,575,901,555,906,554,1894,554,907,554,907,568,890,571,1878,555,906,569,909,550,1881,567,894,566,911,547,913,547,914,564,1881,552,1881,567,2121,
// Timings: 3835,569,900,558,1874,575,887,572,905,552,1878,573,906,553,908,551,909,552,1895,554,905,554,908,567,1879,555,906,566,896,565,910,550,911,551,1898,550,1880,565,1883,550,1882,570,1877,552,909,567,1879,552,1881,569,1877,557,1894,555,1875,572,1874,557,905,554,907,571,1875,558,1875,571,907,553,908,551,1895,554,1876,573,3816,
// Timings: 3831,571,900,559,1873,575,903,555,904,558,1891,552,908,555,906,554,907,567,1879,553,909,568,907,553,1881,567,892,567,910,549,911,550,910,564,1883,551,1879,
//
//
//                    xxxx xxxx  xxcc tttt  tttt tttt  xxxx hhhh  hhhh 0000
// CH1: 26.2°C, 52%:  0011 1110  1000 0001  0000 0110  1111 0011  0100 0000,
// x - unknown
// c - channel
// t - temperature
// h - humidity

#include "BresserReceiver.h"

BresserReceiver::BresserReceiver() {
	BresserReceiver::reset();
}

void BresserReceiver::handleInterrupt() {
    static word last;
    // determine the pulse length in microseconds, for either polarity
    word pulse = micros() - last;
    last += pulse;

	BresserReceiver::handlePulse(pulse);
}

void BresserReceiver::handlePulse(word pulse) {
	if (BresserReceiver::nextPulse(pulse)) {
		const byte* data = BresserReceiver::data;
		memcpy(lastReceivedPacket, data, BresserReceiver_NUM_BYTES);
		packetReceived = true;
		BresserReceiver::reset();
	}
}

bool BresserReceiver::nextPulse(word width) {
	if (state == DONE) {
		return true;
	}

	switch (state) {
		case UNKNOWN:
			if (widthMatches(width, BresserReceiver_PREAMBLE)) {
				state = OK;
				#ifdef BresserReceiver_DEBUG_TIMINGS
					idx = 0;
					timings[idx++] = width;
				#endif
			}
			break;
		case OK:
			#ifdef BresserReceiver_DEBUG_TIMINGS
				timings[idx++] = width;
			#endif
			if (widthMatches(width, BresserReceiver_HIGH_PULSE)) {
				state = T0;
			} else {
				reset();
			}
			break;
		case T0:  // previous pulse was short high
			#ifdef BresserReceiver_DEBUG_TIMINGS
				timings[idx++] = width;
			#endif
			if (widthMatches(width, BresserReceiver_SHORT_LOW_PULSE)) {
				BresserReceiver::gotBit(0);
				state = OK;
			} else if (widthMatches(width, BresserReceiver_LONG_LOW_PULSE)) {
				BresserReceiver::gotBit(1);
				state = OK;
			} else {
				reset();
			}
			break;
		default:
			reset();
	}

	if (BresserReceiver::receivedBits >= BresserReceiver_MAX_PULSES) {
		reset();
	}

	if (receivedBits == BresserReceiver_NUM_BITS) {
		#ifdef BresserReceiver_DEBUG_TIMINGS
			dumpTimings();
		#endif
		done();
	}
	return state == DONE;
}

inline unsigned int BresserReceiver::widthMatches(word width, word expectedWidth) {
	return expectedWidth - BresserReceiver_TOLERANCE <= width && width <= expectedWidth + BresserReceiver_TOLERANCE;
}


void BresserReceiver::reset () {
	receivedBits = 0;
	state = UNKNOWN;
	#ifdef BresserReceiver_DEBUG_TIMINGS
		dumpTimings();
	#endif
}

// add one bit to the packet data buffer
void BresserReceiver::gotBit(byte bt) {
	int i = BresserReceiver::receivedBits++ / 8;
	data[i] = (data[i]<<1) | (bt ? 1 : 0);
}

void BresserReceiver::done() {
	while (BresserReceiver::receivedBits % 8 != 0) {
		BresserReceiver::gotBit(0); // padding
	}
	BresserReceiver::state = DONE;
}

#ifdef BresserReceiver_DEBUG_TIMINGS
void dumpTimings() {
	if (idx > 10) {
		Serial.print("Timings: ");
		for (int i=0; i<idx; i++) {
			Serial.print(timings[i], DEC);
			Serial.print(",");
		}
		Serial.println();
		idx = 0;
	}
}
#endif



bool BresserReceiver::takePacket(byte* out) {
	noInterrupts();
	bool received = packetReceived;
	if (received) {
		memcpy(out, lastReceivedPacket, BresserReceiver_NUM_BYTES);
	}
	packetReceived = false;
	interrupts();
	return received;
}

// iiii iiii  iicc tttt  tttt tttt  xxxx hhhh  hhhh 0000
// 0101 1111  1000 0000  1111 1011  1111 0011  0010 0000   
int BresserReceiver::deviceId(const byte* data) {
	return data[0]<<2 | ((data[1]>>6) & 0x03);
}

// xxxx xxxx  xxcc tttt  tttt tttt  xxxx hhhh  hhhh 0000
// 0101 1111  1000 0000  1111 1011  1111 0011  0010 0000   (25.1°C)
float BresserReceiver::temperature(const byte* data) {
	bool negative = data[1] & 0x8;
	int t = (((int)data[1] & 0x0F) << 8) | data[2];

	if (negative) {
		t = (t^0xFFF) & 0x7F;
		t += 1;
	}
	return (negative ? -1 : 1) * (float)t / 10.0;
}

// xxxx xxxx  xxcc tttt  tttt tttt  xxxx hhhh  hhhh 0000
byte BresserReceiver::humidity(const byte* data) {
	return (data[3] & 0x0F) << 4 | (data[4] >> 4);
}

// xxxx xxxx  bbcc tttt  tttt tttt  xxxx hhhh  hhhh 0000
// 0101 1100  1001 0001  0000 0010  1111 0011  0001 0000
byte BresserReceiver::battery(const byte* data) {
	return (data[1] & 0x80) ? 90 : 10;
}

// xxxx xxxx  xxcc tttt  tttt tttt  xxxx hhhh  hhhh 0000
// 0101 1100  1001 0001  0000 0010  1111 0011  0001 0000
byte BresserReceiver::channel(const byte* data) {
	return ((data[1]>>4) & 0x03) + 1;
}

bool BresserReceiver::samePacket(byte* packet1, byte* packet2) {
	for (int i=0; i<BresserReceiver_NUM_BYTES; i++) {
		if (packet1[i] != packet2[i]) {
			return false;
		}
	}
	return true;
}
