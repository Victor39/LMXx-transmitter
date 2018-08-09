#include <stdint.h>
#include "Device.h"
#include "Message.h"

void delay (const unsigned int cnt) {

	for (int i = 0; i < cnt; ++i) {
		asm(" nop");
	}

}
int main (void) {


	lmx2571::Device & transmitter = lmx2571::Device::getInstance();


////	uint16_t a = 0x0D35;
////	uint16_t b = 0x0D32;
////	transmitter.setBits(&a, 0, b, 2, 3);
//
	// Περες
//	transmitter.writeRegister(0, 0x2000);
//	uint16_t registr0_1 = transmitter.readRegister(0);

	transmitter.write_RESET(1);
	uint16_t registr0_2 = transmitter.readRegister(0);

	// Enable FSK SPI FAST mode
//	transmitter.writeRegister(34, 0x0003);
//	uint16_t registr34_1 = transmitter.readRegister(34);
//
//	transmitter.writeRegister(8, 0x0410);
//	uint16_t registr8_1 = transmitter.readRegister(8);

	transmitter.write_FSK_MODE_SEL0(1);
	transmitter.write_FSK_MODE_SEL1(1);
	uint16_t registr34_2 = transmitter.readRegister(34);

	transmitter.write_FSK_EN_F1(1);
	uint16_t registr8_2 = transmitter.readRegister(8);

	// Carrier
//	transmitter.writeRegister(6, 0x9B84);
//	uint16_t registr6_1 = transmitter.readRegister(6);
	transmitter.write_CHDIV1_F1(0);
	transmitter.write_CHDIV2_F1(6);
	uint16_t registr6_2 = transmitter.readRegister(6);

	transmitter.write_PLL_N_PRE_F1(1);
	transmitter.write_PLL_NUM_F1(1388608);
	transmitter.write_PLL_DEN_F1(0);
	transmitter.write_PLL_N_F1(16);

	transmitter.write_PLL_R_PRE_F1(1);
	transmitter.write_MULT_F1(4);
	transmitter.write_PLL_R_F1(1);


	// Set MUXout as lock detect
//	transmitter.writeRegister(39, 0x11FB);
	transmitter.write_SDO_LD_SEL(1);
	transmitter.write_LD_EN(1);

	// Enable
//	transmitter.writeRegister(0, 0x0003);
//	uint16_t registr0_2 = transmitter.readRegister(0);
	transmitter.write_RESET(0);
	transmitter.write_FCAL_EN(1);


//	//	transmitter.write_PLL_N_PRE_F1(1);
//	//	transmitter.write_PLL_DEN_F1(8388608);
//	//
//	//	transmitter.write_PLL_R_PRE_F1(1);
//	//	transmitter.write_MULT_F1(4);
//	//	transmitter.write_PLL_R_F1(1);
//
//		// Set MUXout as lock detect
//		transmitter.write_LD_EN(1);
//		transmitter.write_SDO_LD_SEL(1);
//




	// Samples
	unsigned short sample = 0;
	while (1) {

		transmitter.writeRegister(33, sample);
		delay(10);
		sample += 1500;

		asm(" nop");
	}

//	return (0);
}

