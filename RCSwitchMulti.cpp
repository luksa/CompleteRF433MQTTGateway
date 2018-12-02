#include "RCSwitchMulti.h"

volatile unsigned long RCSwitchMulti::nReceivedValue = 0;
volatile unsigned int RCSwitchMulti::nReceivedBitlength = 0;
volatile unsigned int RCSwitchMulti::nReceivedDelay = 0;
volatile unsigned int RCSwitchMulti::nReceivedProtocol = 0;
unsigned int RCSwitchMulti::timings[RCSwitchMulti_MAX_CHANGES];
int RCSwitchMulti::nReceiveTolerance = 60;

RCSwitchMulti::RCSwitchMulti() {
  this->nReceiverInterrupt = -1;
  this->nReceivedValue = 0;
  this->setReceiveTolerance(60);
}

void RCSwitchMulti::setReceiveTolerance(int nPercent) {
  this->nReceiveTolerance = nPercent;
}

void RCSwitchMulti::enableReceive(int pin) {
  this->nReceiverInterrupt = digitalPinToInterrupt(pin);
  this->enableReceive();
}

void RCSwitchMulti::enableReceive() {
  if (this->nReceiverInterrupt != -1) {
    RCSwitchMulti::nReceivedValue = 0;
    RCSwitchMulti::nReceivedBitlength = 0;
//    attachInterrupt(this->nReceiverInterrupt, RCSwitchMulti_handleInterrupt, CHANGE);
  }
}

void RCSwitchMulti::disableReceive() {
	detachInterrupt(this->nReceiverInterrupt);
	this->nReceiverInterrupt = -1;
}

bool RCSwitchMulti::available() {
  return RCSwitchMulti::nReceivedValue != 0;
}

void RCSwitchMulti::resetAvailable() {
  RCSwitchMulti::nReceivedValue = 0;
}

unsigned long RCSwitchMulti::getReceivedValue() {
return RCSwitchMulti::nReceivedValue;
}

unsigned int RCSwitchMulti::getReceivedBitlength() {
  return RCSwitchMulti::nReceivedBitlength;
}

unsigned int RCSwitchMulti::getReceivedDelay() {
  return RCSwitchMulti::nReceivedDelay;
}

unsigned int RCSwitchMulti::getReceivedProtocol() {
  return RCSwitchMulti::nReceivedProtocol;
}

unsigned int* RCSwitchMulti::getReceivedRawdata() {
    return RCSwitchMulti::timings;
}

/* helper function for the receiveProtocol method */
static inline unsigned int diff(int A, int B) {
  return abs(A - B);
}

//void RCSwitchMulti_handleInterrupt() {
//	static unsigned long lastTime = 0;
//	const long time = micros();
//	const unsigned int duration = time - lastTime;	// time since previous interrupt
//	RCSwitchMulti::handlePulse(duration);
//	lastTime = time;
//}

void RCSwitchMulti::handlePulse(unsigned int duration) {

	static unsigned int changeCount = 0;
	static int repeatCount = 0;
	static unsigned int previousTimings[RCSwitchMulti_MAX_CHANGES];

	static const bool CHECK_CONSECUTIVE_PACKETS_FOR_EQUALITY = false;


	if (duration > 2900) {

//		char buffer[100];
//	    sprintf(buffer, "long duration: %u; previous duration: %u", duration, RCSwitchMulti::timings[0]);
//		Serial.println(buffer);
		// A long stretch without signal level change occurred. This could
		// be the gap between two transmission.
		if (diff(duration, RCSwitchMulti::timings[0]) < 100) {
			// This long signal is close in length to the long signal which
			// started the previously recorded timings; this suggests that
			// it may indeed by a a gap between two transmissions (we assume
			// here that a sender will send the signal multiple times,
			// with roughly the same gap between them).

			repeatCount++;
			if (repeatCount == 1) {

				if (CHECK_CONSECUTIVE_PACKETS_FOR_EQUALITY) {
					for (int i=0; i<changeCount; i++) {
						previousTimings[i] = RCSwitchMulti::timings[i];
					}
				}
			} else if (repeatCount == 2) {
				bool consecutivePacketsMatch = true;
				if (CHECK_CONSECUTIVE_PACKETS_FOR_EQUALITY) {
					for (int i=0; i<changeCount; i++) {

						unsigned long percent = (unsigned long)previousTimings[i]  * 100 / RCSwitchMulti::timings[i];
						unsigned int percentDifference = abs(100 - (signed int)percent);

						if (percentDifference > 25) {
							consecutivePacketsMatch = false;
							break;
						}
					}
				}

				if (changeCount > 6 && consecutivePacketsMatch) {	// no device sends less than 6 changes
					if (RCSwitchMulti_DEBUG_PACKETS) {
						for (int i=0; i<changeCount; i++) {
							char buffer[20];
							Serial.print(itoa(RCSwitchMulti::timings[i], buffer, 10));
							Serial.print(",");
						}
						Serial.println();
					}

					receiveIntertechno(changeCount) || receiveNatsen(changeCount);
				}
				repeatCount = 0;
			}
		}
		changeCount = 0;
	}

	// detect overflow
	if (changeCount >= RCSwitchMulti_MAX_CHANGES) {
//		Serial.println("over MAX_CHANGES");
		changeCount = 0;
		repeatCount = 0;
	}

	RCSwitchMulti::timings[changeCount] = duration;
	changeCount++;
}


static inline bool timingWithin(unsigned int duration, long expectedDuration, long tolerance) {
  return (duration > expectedDuration - tolerance)
  	  && (duration < expectedDuration + tolerance);
}


/**
 * 1,10 preamble
 * bits
 * pulse high + very long pulse low
 */
bool RCSwitchMulti::receiveIntertechno(unsigned int changeCount){
// intertechno app sends: 0,0,6,0,256,67,0,   1,10, 1,5,1,1, 1,5,1,1, 1,1,1,5, 1,1,1,5, 1,5,1,1, 1,5,1,1, 1,5,1,1,1,5,1,1,1,5,1,1,1,5,1,1,1,1,1,5,1,5,1,1,1,1,1,5,1,1,1,5,1,5,1,1,1,5,1,1,1,1,1,5,1,5,1,1,1,1,1,5,1,5,1,1,1,5,1,1,1,5,1,1,1,5,1,1,1,1,1,5,1,5,1,1,1,5,1,1,1,1,1,5,1,5,1,1,1,5,1,1,1,1,1,5,1,1,1,5,1,1,1,5, 1,39  ,0
// Timings (intertechno app):
// 9928, 344,2484, 348,1204,352,176, 348,1200,352,188, 336,192,336,1204, 344,192,332,1224, 332,1224,332,196, 332,1216,340,192, 332,1216,336,200,332,1220,336,200,324,1224,328,200,328,1212,344,196,332,196,332,1212,324,1228,332,192,332,200,328,1224,328,200,324,1212,328,1212,328,196,332,1212,328,196,332,200,324,1216,328,1220,336,196,332,196,332,1212,340,1220,328,200,336,1216,336,192,336,1224,336,192,328,1220,336,196,328,200,324,1228,332,1216,336,192,336,1220,336,192,328,204,328,1220,332,1208,348,196,332,1220,332,200,328,200,328,1224,328,196,332,1224,332,196,332,1212,328,


// remote sends: 0,0,6,0,350,67,0,   1,7, 1,5,1,1, 1,5,1,1, 1,1,1,5, 1,1,1,5, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,1,1,5, 1,5,1,1, 1,1,1,5, 1,1,1,5, 1,5,1,1, 1,5,1,1, 1,1,1,5, 1,5,1,1, 1,1,1,5, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,5,1,1, 1,1,1,5, 1,5,1,1,1,5,1,1,1,1,1,5,1,5,1,1,1,5,1,1,1,1,1,5,1,1,1,5,1,1,1,5, 1,39  ,0
// Timings (new intertechno remote):
// 10336, 328,2648, 332,216,328,1256, 332,1240,332,220, 336,216,328,1260, 324,216,328,1256, 328,1244,332,228, 328,216,328,1256, 328,220,324,1256, 332,1248,328,240,324,1248,328,224,328,1248,324,232,320,1256,320,240,320,224,316,1264,320,1260,316,240,312,228,320,1268,316,232,312,1272,308,236,312,1284,312,1264,312,244,308,1264,308,248,312,236,304,1284,304,1268,308,244,308,1268,308,248,304,1268,304,256,296,252,292,1292,292,1280,296,272,288,1296,284,268,280,268,276,1304,284,268,276,1308,276,268,276,1312,272,272,272,1316,268,276,268,1316,268,276,268,1316,268,1308,268,288,252,
// 10332, 328,2652, 332,216,332,1248, 332,1248,332,220, 332,216,328,1256, 328,216,328,1260, 328,1248,328,224, 328,220,324,1256, 328,224,324,1256, 328,1252,324,244,324,1248,328,228,324,1252,324,232,320,1256,324,236,316,224,316,1268,320,1252,324,236,316,228,316,1272,312,232,312,1280,308,240,304,1284,316,1272,300,244,312,1268,308,244,312,236,308,1280,300,1272,304,256,304,1272,304,252,300,1276,300,256,292,252,292,1296,292,1284,288,276,288,1288,292,268,284,260,280,1304,284,264,276,1308,276,272,272,1312,272,280,264,1320,264,280,264,1316,272,280,260,1328,264,1308,264,284,260,
// 10332, 1H, 12L,  1H,1L,1H,6L, 1H,6L,1H,1L

// 710P 2 off:         11 001111 110100 110101 111011 0 0 1000
// 710P 2 on:          11 001111 110100 110101 111011 0 0 1000
// 710Q 2 off:         11 010000 110100 110101 111011 0 0 1000
// old remote 4 off:   00 011000 001110 110110 110110 0 0 0010
// old remote all off: 00 011000 001110 110110 110110 1 0 0000

	unsigned long code = 0;

	unsigned long highPulseDuration = RCSwitchMulti::timings[1];
	unsigned long highPulseDurationTolerance = highPulseDuration * RCSwitchMulti::nReceiveTolerance * 0.01;
	unsigned long lowPulseDuration = RCSwitchMulti::timings[2] / 12;
	unsigned long lowPulseDurationTolerance = lowPulseDuration * RCSwitchMulti::nReceiveTolerance * 0.01;

	// timings[0] = very long pulse low
	// timings[1] = preamble's 1 unit long high pulse
	// timings[2] = preamble's 12 units long low pulse
	// timings[3..] = data bits

	for (int i = 3; i+3<changeCount ; i+=4) {

		if (timingWithin(RCSwitchMulti::timings[i], 1*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+1], 1*lowPulseDuration, lowPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+2], 1*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+3], 6*lowPulseDuration, lowPulseDurationTolerance) ) {
			// bit = 0
			code = code << 1;
		} else if (timingWithin(RCSwitchMulti::timings[i], 1*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+1], 6*lowPulseDuration, lowPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+2], 1*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+3], 1*lowPulseDuration, lowPulseDurationTolerance) ) {

			// bit = 1
			code = code << 1;
			code+=1;
		} else {
			// Failed
//			char buffer[100];
//			sprintf(buffer, "Failed at %u", i);
//			Serial.println(buffer);
			code = 0;
			break;
		}
	}
	if (changeCount > 6) {    // ignore < 6bit values as there are no devices sending 6bit values => noise
//	    Serial.println("Changing nReceivedValue");
//	    Serial.println("Storing new nReceivedValue");
		RCSwitchMulti::nReceivedValue = code;
		RCSwitchMulti::nReceivedBitlength = changeCount / 4;
		RCSwitchMulti::nReceivedDelay = highPulseDuration;
		RCSwitchMulti::nReceivedProtocol = PROTOCOL_INTERTECHNO;
	}

	if (code == 0){
		return false;
	} else if (code != 0){
		return true;
	}
}


/**
 * 1,10 preamble
 * bits
 * pulse high + very long pulse low
 */
bool RCSwitchMulti::receiveNatsen(unsigned int changeCount){
// top right button:     3052,264,752,764,248,756,260,756,260,248,764,748,256,756,268,240,772,248,756,748,276,752,248,256,756,260,756,760,256,752,256,256,756,256,756,260,752,260,756,252,756,264,748,264,752,764,252,256,744,256,
// 2nd top right button: 3044,272,744,772,248,764,248,760,252,256,756,764,244,760,268,244,756,260,752,764,252,760,252,256,752,256,768,748,256,760,256,252,756,256,756,260,740,272,752,256,748,772,248,260,748,268,740,272,736,264,
//                       11,1,3,3,1,3,1,3,1,1,3,3,1,3,1,1,3,1,3,3,1,3,1,1,3,1,3,3,1,3,1,1,3,1,3,1,3,1,3,1,3,1,3,1,3,3,1,1,3,1


	unsigned long code = 0;

	unsigned long highPulseDuration = RCSwitchMulti::timings[0] / 11;
	unsigned long highPulseDurationTolerance = highPulseDuration * RCSwitchMulti::nReceiveTolerance * 0.01;
	unsigned long lowPulseDuration = RCSwitchMulti::timings[0] / 11;
	unsigned long lowPulseDurationTolerance = lowPulseDuration * RCSwitchMulti::nReceiveTolerance * 0.01;

	// timings[0] = very long pulse low
	// timings[1] = preamble's 1 unit long high pulse
	// timings[2] = preamble's 12 units long low pulse
	// timings[3..] = data bits

	for (int i = 1; i+1<changeCount ; i=i+2) {

		if  (timingWithin(RCSwitchMulti::timings[i], 1*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+1], 3*lowPulseDuration, lowPulseDurationTolerance) ) {

			// bit = 0
			code = code << 1;
		} else if (timingWithin(RCSwitchMulti::timings[i], 3*highPulseDuration, highPulseDurationTolerance)
			&& timingWithin(RCSwitchMulti::timings[i+1], 1*lowPulseDuration, lowPulseDurationTolerance) ) {

			// bit = 1
			code+=1;
			code = code << 1;
		} else {
			// Failed
//			char buffer[100];
//			sprintf(buffer, "Failed at %u", i);
//			Serial.println(buffer);
			i = changeCount;
			code = 0;
		}
	}
	code = code >> 1;
	if (changeCount > 6) {    // ignore < 4bit values as there are no devices sending 4bit values => noise
//	    Serial.println("Changing nReceivedValue");
//	    Serial.println("Storing new nReceivedValue");
		RCSwitchMulti::nReceivedValue = code;
		RCSwitchMulti::nReceivedBitlength = changeCount / 2;
		RCSwitchMulti::nReceivedDelay = highPulseDuration;
		RCSwitchMulti::nReceivedProtocol = PROTOCOL_NATSEN;
	}

	if (code == 0){
		return false;
	} else if (code != 0){
		return true;
	}
}
