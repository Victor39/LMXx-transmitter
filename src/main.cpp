#include <stdint.h>
#include <stdlib.h>
#include "Device.h"
#include "Message.h"

void delayNop (int n) {
	for (int j = 0; j < n; ++j) {
		asm(" nop");
	}
}

int main (void) {

	lmx2571::Device & transmitter = lmx2571::Device::getInstance();
	transmitter.startUp();


//	transmitter.setTxMode();
	transmitter.setRxMode();

	while (1) {

		asm(" nop");

//		for (int i = 0; i < 6240; ++i) {
//			transmitter.setTxFrequencyBy(i);
//			for (int j = 0; j < 100000; ++j) {
//				asm(" nop");
//			}
//		}

//		uint16_t index = rand()%6240;
//		transmitter.setTxFreqBy(index);
//		delayNop(500000);

//		transmitter.setTxMode();
//		for (int index = 0; index < 6240; ++index) {
//			transmitter.setTxFreqBy(rand()%6240);
//			transmitter.setTxFreqBy(index);
//			delayNop(200000);
//		}

//		transmitter.setTxMode();
//		transmitter.setTxFreqBy(rand()%6240);

//		for (int index = 0; index < 6240; ++index) {
			transmitter.setRxFreqBy(rand()%6240);
			delayNop(500000);
//		}

//		transmitter.setTxFrequencyBy_2(1282);
//		delayNop(500000);
//		transmitter.setTxFrequencyBy_2(1300);
//		delayNop(500000);

	}

//	return (0);
}

// Modulation
// Samples
//	unsigned short sample = 0;
//	const float dev = 2e3;
//	const float Fpd = 80e6;
//	const float Prescaler = 4;
//	const float CHDIV1 = 4;
//	const float CHDIV2 = 64;
//	const float DEN = 100000;
//
//#define SINE_SIZE (24)
//	float sin_24k_fs_1k_fc[SINE_SIZE] = {0.258819045102521, 0.500000000000000, 0.707106781186548, 0.866025403784439,
//			0.965925826289068, 1, 0.965925826289068, 0.866025403784439, 0.707106781186548, 0.500000000000000, 0.258819045102521,
//			1.22464679914735e-16, -0.258819045102520, -0.500000000000000, -0.707106781186547, -0.866025403784439,
//			-0.965925826289068, -1, -0.965925826289068, -0.866025403784439, -0.707106781186548, -0.500000000000000,
//			-0.258819045102522, -2.44929359829471e-16};
//
//		for (int i = 0; i < SINE_SIZE; ++i) {
//			float Fdev = dev * sin_24k_fs_1k_fc[i];
//			sample = round((Fdev * DEN) / Fpd * (CHDIV1 * CHDIV2) / Prescaler);
//			transmitter.writeRegister(33, sample);
//		}
