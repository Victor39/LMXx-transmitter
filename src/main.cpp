#include <stdint.h>
#include "Device.h"
#include "Message.h"

int main (void) {

	Lmx2571::Device & transmitter = Lmx2571::Device::getInstance();



	while (1) {

		transmitter.setRegister(53, 0x2803);
		uint16_t data = transmitter.getRegister(53);
		asm(" nop");
	}

	return (0);
}

