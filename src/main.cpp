#include <stdint.h>
#include "Device.h"
#include "Message.h"



int main (void) {

	lmx2571::Device & transmitter = lmx2571::Device::getInstance();

	// Περες
	transmitter.write_RESET(1);
	uint16_t registr0 = transmitter.readRegister(0);

	// Enable FSK SPI FAST mode
	transmitter.write_FSK_MODE_SEL0(1);
	transmitter.write_FSK_MODE_SEL1(1);
	uint16_t registr34 = transmitter.readRegister(34);

	transmitter.write_FSK_EN_F1(1);
	uint16_t registr8 = transmitter.readRegister(8);

	// Out frequency
	transmitter.write_CHDIV1_F1(0);
	transmitter.write_CHDIV2_F1(6);
	uint16_t registr6 = transmitter.readRegister(6);

	transmitter.write_PLL_N_PRE_F1(1);
	transmitter.write_PLL_NUM_F1(20000);
	transmitter.write_PLL_DEN_F1(100000);
	transmitter.write_FRAC_ORDER_F1(1);
	transmitter.write_PLL_N_F1(15);

	transmitter.write_PLL_R_PRE_F1(1);
	transmitter.write_MULT_F1(4);
	transmitter.write_PLL_R_F1(1);

	// Set MUXout as lock detect
//	transmitter.write_SDO_LD_SEL(1);
//	transmitter.write_LD_EN(1);

	// Enable
//	transmitter.writeRegister(0, 0x0003);
//	uint16_t registr0_2 = transmitter.readRegister(0);
	transmitter.write_RESET(0);
	transmitter.write_F1F2_MODE(1);
	transmitter.write_F1F2_SEL(0);
	transmitter.write_FCAL_EN(1);
	uint16_t registers2[61];

	for (int i = 0; i < 61; ++i) {
		registers2[i] = transmitter.readRegister(i);
	}

#define SINE_SIZE (24)
	float sin_24k_fs_1k_fc[SINE_SIZE] = {0.258819045102521, 0.500000000000000, 0.707106781186548, 0.866025403784439,
			0.965925826289068, 1, 0.965925826289068, 0.866025403784439, 0.707106781186548, 0.500000000000000, 0.258819045102521,
			1.22464679914735e-16, -0.258819045102520, -0.500000000000000, -0.707106781186547, -0.866025403784439,
			-0.965925826289068, -1, -0.965925826289068, -0.866025403784439, -0.707106781186548, -0.500000000000000,
			-0.258819045102522, -2.44929359829471e-16};

	// Samples
	unsigned short sample = 0;
	const float dev = 2e3;
	const float Fpd = 80e6;
	const float Prescaler = 4;
	const float CHDIV1 = 4;
	const float CHDIV2 = 64;
	const float DEN = 100000;

	while (1) {

		for (int i = 0; i < SINE_SIZE; ++i) {
			float Fdev = dev * sin_24k_fs_1k_fc[i];
			sample = round((Fdev * DEN) / Fpd * (CHDIV1 * CHDIV2) / Prescaler);
			transmitter.writeRegister(33, sample);
		}

		asm(" nop");
	}

//	return (0);
}


//	PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON,
//	PSC_MDCTL_NEXT_ENABLE);
//
//	GPIOBank0Pin1PinMuxSetup();
////	GPIOBank0Pin3PinMuxSetup();
////	GPIOBank0Pin5PinMuxSetup();
//
//	GPIODirModeSet(SOC_GPIO_0_REGS, 2, GPIO_DIR_OUTPUT);
////	GPIODirModeSet(SOC_GPIO_0_REGS, 4, GPIO_DIR_OUTPUT);
////	GPIODirModeSet(SOC_GPIO_0_REGS, 6, GPIO_DIR_OUTPUT);
//
//	while (1) {
//		GPIOPinWrite(SOC_GPIO_0_REGS, 2, GPIO_PIN_HIGH);
//		GPIOPinWrite(SOC_GPIO_0_REGS, 2, GPIO_PIN_LOW);
//	}

