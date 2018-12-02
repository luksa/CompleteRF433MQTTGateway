#ifndef _BresserReceiver_h
#define _BresserReceiver_h

// enable DEBUG with:
//#define BresserReceiver_DEBUG_TIMINGS true

#include "Arduino.h"

#define BresserReceiver_PREAMBLE 3850 // (us) preamble is low
#define BresserReceiver_HIGH_PULSE 550
#define BresserReceiver_SHORT_LOW_PULSE 900
#define BresserReceiver_LONG_LOW_PULSE 1900

#define BresserReceiver_TOLERANCE 100

#define BresserReceiver_MAX_PULSES 80
#define BresserReceiver_NUM_BITS 36
#define BresserReceiver_NUM_BYTES 5


class BresserReceiver {
	public:
		BresserReceiver();

		void handleInterrupt();
		void handlePulse(word pulse);
		bool takePacket(byte* out);

		bool samePacket(byte* packet1, byte* packet2);
		int deviceId(const byte* data);
		byte channel(const byte* data);
		float temperature(const byte* data);
		byte humidity(const byte* data);
		byte battery(const byte* data);

	private:
		enum { UNKNOWN, OK, T0, DONE };

		byte receivedBits, state, data[BresserReceiver_NUM_BYTES];
		bool packetReceived;
		byte lastReceivedPacket[BresserReceiver_NUM_BYTES];

		bool nextPulse(word width);
		virtual void gotBit(byte bt);
		void reset();
		void done();

		static inline unsigned int widthMatches(word width, word expectedWidth);

		#ifdef BresserReceiver_DEBUG_TIMINGS
			byte idx;
			int timings[BresserReceiver_MAX_PULSES];
			virtual void dumpTimings();
		#endif
};

#endif
