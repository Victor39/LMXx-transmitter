#include <stdint.h>
#include <math.h>
#include <string.h>
#include <src/Device.h>
#include "soc_OMAPL138.h"
#include "hw_psc_OMAPL138.h"
#include "lcdkOMAPL138.h"
#include "spi.h"
#include "psc.h"
#include "interrupt.h"
#include "gpio.h"

namespace lmx2571 {

// TX parameters
#define TX_MIN_FREQ	(30000000UL)
#define TX_MAX_FREQ	(108000000UL)
#define TX_FREQ_SPACING	(12500UL)

#define TX_MIN_FREQ_INDEX (0)
#define TX_MAX_FREQ_INDEX ((TX_MAX_FREQ - TX_MIN_FREQ) / TX_FREQ_SPACING)

#define TX_PDD_FREQ	(50000000)
#define TX_PRE_N_DIVIDER	(4)

// RX parameters
#define RX_MIN_FREQ	(285000000UL)
#define RX_MAX_FREQ	(363000000UL)
#define RX_FREQ_SPACING	(12500UL)

#define RX_MIN_FREQ_INDEX (0)
#define RX_MAX_FREQ_INDEX ((RX_MAX_FREQ - RX_MIN_FREQ) / RX_FREQ_SPACING)

#define RX_PDD_FREQ	(80000000)
#define RX_PRE_N_DIVIDER	(2)

//
#define MIN_VCO_FREQ	(4300000000)
#define MAX_VCO_FREQ	(5376000000)
#define PLL_DEN	(10000000UL)
#define SPI_CS						4
//
	const uint16_t DEFV_REGISTER[REGISTER_ARR_SIZE] = {
	// 0 ... 9
			0x0003, 0x0000, 0x0000, 0x0000, 0x0028, 0x0101, 0x8584, 0x10A4, 0x0010, 0x0000,
			// 10 ... 19
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			// 20 ... 29
			0x0028, 0x0101, 0x8584, 0x10A4, 0x0010, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			// 30 ... 39
			0x0000, 0x0000, 0x0000, 0x0000, 0x1000, 0x0647, 0x0000, 0x0000, 0x0000, 0x11F0,
			// 40 ... 49
			0x101C, 0x0810, 0x0210, 0x0000, 0x0000, 0x0000, 0x001A, 0x0000, 0x0000, 0x0000,
			// 50 ... 59
			0x0000, 0x0000, 0x0000, 0x2802, 0x0000, 0x0000, 0x0000, 0x0000, 0x0C00, 0x0000,
			// 60
			0x4000};

	void Device::spiTransfer (const uint8_t* dataTx, uint8_t* dataRx, const uint16_t len) {

		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1 << SPI_CS));

		for (uint16_t i = 0; i < len; i++) {
			while (!SPIIntStatus(SOC_SPI_0_REGS, SPI_TRANSMIT_INT))
				;
			SPITransmitData1(SOC_SPI_0_REGS, dataTx[i]);

			while (!SPIIntStatus(SOC_SPI_0_REGS, SPI_RECV_INT))
				;
			dataRx[i] = (uint8_t) SPIDataReceive(SOC_SPI_0_REGS);
		}

		SPIDat1Config(SOC_SPI_0_REGS, (SPI_DATA_FORMAT0), (1 << SPI_CS));
	}

	void Device::startUp () {

		/// Reset
		write_RESET_R0(1);

		/// Enable FSK SPI FAST mode
		write_FSK_MODE_SEL0_R34(1);
		write_FSK_MODE_SEL1_R34(1);
		write_FSK_EN_F1_R8(1);

		/// Set MUXout as lock detect
		write_SDO_LD_SEL_R39(1);
		write_LD_EN(1);

		/// Set charge pump
		write_CP_IUP_R40(8); // 1250 uA
		write_CP_GAIN_R40(3); // 2.5x
		write_CP_IDN_R41(8); // 1250 uA

		/// Set parameters loop filter
		write_LF_R3_F1_R6(6); // 533 Ohm
		write_LF_R4_F1_R7(7); // 457 Ohm
		write_LF_R3_F2_R22(6); // 533 Ohm
		write_LF_R4_F2_R23(7); // 457 Ohm

		/// Set ...
		write_MULT_WAIT_R35(400);
		write_PFD_DELAY_F1_R6(4);
		write_FRAC_ORDER_F1_R4(3);
		write_PFD_DELAY_F2_R22(4);
		write_FRAC_ORDER_F2_R20(3);

		// Set tx/rx constant parts of N-divider
		write_PLL_DEN_F1_R1R3(PLL_DEN);
		write_PLL_DEN_F2_R17R19(PLL_DEN);
		write_PLL_N_PRE_F1_R4(1 /*TX_PRE_N_DIVIDER*/);
		write_PLL_N_PRE_F2_R20(0 /*RX_PRE_N_DIVIDER*/);

		// Set tx/rx R-divider
		write_MULT_F1_R6(5);
		write_PLL_R_PRE_F1_R5(1);
		write_PLL_R_F1_R5(2);

		write_MULT_F2_R22(4);
		write_PLL_R_PRE_F2_R21(1);
		write_PLL_R_F2_R21(1);

		// Set RF output pins
		write_OUTBUF_TX_EN_F1_R7(1);
		write_OUTBUF_RX_EN_F1_R7(0);
		write_OUTBUF_TX_EN_F2_R23(0);
		write_OUTBUF_RX_EN_F2_R23(1);

		// Enable
		write_RESET_R0(0);
		write_FCAL_EN_R0(1);

	}

	void Device::setTxMode () {

		write_F1F2_SEL_R0(0);
		// todo
		setTxFreqBy(0);
	}

	bool Device::setTxFreqBy (const uint32_t freqIndex) {

		const uint32_t startFreq = TX_MIN_FREQ;
		const uint32_t freqStep = TX_FREQ_SPACING;

		return setTxFreq(startFreq + freqIndex * freqStep);
	}

	void Device::setRxMode () {

		write_F1F2_SEL_R0(1);
		// todo
		setRxFreqBy(0);
	}

	bool Device::setRxFreqBy (const uint32_t freqIndex) {

		const uint32_t startFreq = RX_MIN_FREQ;
		const uint32_t freqStep = RX_FREQ_SPACING;

		return setRxFreq(startFreq + freqIndex * freqStep);
	}

	bool Device::calcSettingFor (const uint32_t freq, const uint64_t pddFreq, const uint16_t preNDividerValue,
			FreqSetting & setting) {

		const uint64_t minVcoFreq = MIN_VCO_FREQ;
		const uint64_t maxVcoFreq = MAX_VCO_FREQ;

		// Set out ch divider
		const uint16_t minChdiv = ceilf((minVcoFreq * 1.0f) / freq);
		const uint16_t maxChdiv = floorf((maxVcoFreq * 1.0f) / freq);

		const uint16_t numberOfChdiv1Values = 4;
		const uint16_t possibleChdiv1Values[numberOfChdiv1Values] = {4, 5, 6, 7};
		const uint16_t numberOfChdiv2Values = 7;
		const uint16_t possibleChdiv2Values[numberOfChdiv2Values] = {1, 2, 4, 8, 16, 32, 64};

		uint16_t chDiv1Value = 0;
		uint16_t chDiv2Value = 0;
		uint16_t chdiv = 0;
		for (uint16_t currChdiv = maxChdiv; currChdiv >= minChdiv; --currChdiv) {
			for (uint16_t chdiv1Index = 0; chdiv1Index < numberOfChdiv1Values; ++chdiv1Index) {
				for (uint16_t chdiv2Index = 0; chdiv2Index < numberOfChdiv2Values; ++chdiv2Index) {
					if (currChdiv == (possibleChdiv1Values[chdiv1Index] * possibleChdiv2Values[chdiv2Index])) {
						chDiv1Value = possibleChdiv1Values[chdiv1Index];
						chDiv2Value = possibleChdiv2Values[chdiv2Index];
						chdiv = chDiv1Value * chDiv2Value;
						break;
					}
				}
				if (chdiv != 0)
					break;
			}
			if (chdiv != 0)
				break;
		}

		if (chdiv == 0)
			return false;

		// Set N-divider
		const uint64_t vcoFreq = freq * (uint64_t) chdiv;

		double NFactor = vcoFreq * 1.0 / (preNDividerValue * pddFreq * 1UL);

		const uint32_t pllNValue = floor(NFactor);
		const uint32_t pllNDenValue = PLL_DEN;
		const uint32_t pllNNumValue = (NFactor - pllNValue) * pllNDenValue;

		// Write setting
		setting.preNDivider = (preNDividerValue == 2) ? 0 : 1; // 0 = Divide by 2; 1 = Divide by 4;
		setting.pllN = pllNValue;
		setting.pllNNum = pllNNumValue;
		setting.pllNDen = pllNDenValue;
		setting.chDiv1 = (chDiv1Value == 4) ? 0 : (chDiv1Value == 5) ? 1 : (chDiv1Value == 6) ? 2 : 3;
		setting.chDiv2 = (chDiv2Value == 1) ? 0 : (chDiv2Value == 2) ? 1 : (chDiv2Value == 4) ? 2 : (chDiv2Value == 8) ? 3 :
							(chDiv2Value == 16) ? 4 : (chDiv2Value == 32) ? 5 : (chDiv2Value == 64) ? 6 : 0;
		return true;
	}

	bool Device::setTxFreq (const uint32_t freq) {

		// Set pdd frequency
		uint64_t pddFreq = 0;
		if ((TX_MIN_FREQ <= freq) && (freq <= TX_MAX_FREQ)) {
			pddFreq = TX_PDD_FREQ;
		}
		else {
			return false;
		}
		FreqSetting setting;
		bool settingsSuccessful = calcSettingFor(freq, pddFreq, TX_PRE_N_DIVIDER, setting);

		if (!settingsSuccessful) {
			return false;
		}

		// Set registers
		set_PLL_N_F1_R4(setting.pllN);
		set_PLL_NUM_F1_R1R2(setting.pllNNum);

		set_CHDIV1_F1_R6(setting.chDiv1);
		set_CHDIV2_F1_R6(setting.chDiv2);

		// Write registers
		writeRegister(6);
		writeRegister(4);
		writeRegister(2);
		writeRegister(1);

		write_FCAL_EN_R0(1);

		return true;
	}

	bool Device::setRxFreq (const uint32_t freq) {

		// Set pdd frequency
		uint64_t pddFreq = 0;
		if ((RX_MIN_FREQ <= freq) && (freq <= RX_MAX_FREQ)) {
			pddFreq = RX_PDD_FREQ;
		}
		else {
			return false;
		}
		FreqSetting setting;
		bool settingsSuccessful = calcSettingFor(freq, pddFreq, RX_PRE_N_DIVIDER, setting);

		if (!settingsSuccessful) {
			return false;
		}

		// Set registers
		set_PLL_N_F2_R20(setting.pllN);
		set_PLL_NUM_F2_R17R18(setting.pllNNum);

		set_CHDIV1_F2_R22(setting.chDiv1);
		set_CHDIV2_F2_R22(setting.chDiv2);

		// Write registers
		writeRegister(22);
		writeRegister(20);
		writeRegister(18);
		writeRegister(17);

		write_FCAL_EN_R0(1);

		return true;
	}

	Device::Device () {

		initSpi();
		memcpy(m_registers, DEFV_REGISTER, REGISTER_ARR_SIZE * (sizeof(m_registers[0])));
	}

	void Device::setBits (uint16_t * reg, uint16_t wstart, uint16_t value, uint16_t rstart, uint16_t num) {

		for (uint16_t i = 0; i < num; ++i) {
			*reg ^= (-(unsigned long) (value >> (rstart + i) & 1UL) ^ *reg) & (1UL << (wstart + i));
		}
	}

	void Device::writeRegister (const uint8_t adrr, const uint16_t data) {

		m_txMsg.rw() = 0;
		m_txMsg.address() = adrr;
		m_txMsg.data() = data;
		m_txMsg.write(m_txBuffer);

		spiTransfer(m_txBuffer, m_rxBuffer, 3);
	}

	uint16_t Device::readRegister (const uint8_t adrr) {
		m_txMsg.rw() = 1;
		m_txMsg.address() = adrr;
		m_txMsg.write(m_txBuffer);

		spiTransfer(m_txBuffer, m_rxBuffer, m_txMsg.size());

		m_rxMsg.read(m_rxBuffer);

		return m_rxMsg.data();
	}

	void Device::initSpi () {

		// SPI
		/* Waking up the SPI0 instance. */
		PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_SPI0, PSC_POWERDOMAIN_ALWAYS_ON,
		PSC_MDCTL_NEXT_ENABLE);

		/* Performing the Pin Multiplexing for SPI0. */
		SPIPinMuxSetup(0);

		/* Using the Chip Select(CS) 4 pin of SPI0 to communicate with the LMX2571. */
		SPI0CSPinMuxSetup(SPI_CS);

//		/* Enable use of SPI0 interrupts. */
//		// Initialize the DSP interrupt controller
//		IntDSPINTCInit();
//
//		// Register the ISR in the vector table
//		IntRegister(C674X_MASK_INT4, SPIIsr);
//
//		// Map system interrupt to the DSP maskable interrupt
//		IntEventMap(C674X_MASK_INT4, SYS_INT_SPI0_INT);
//
//		// Enable the DSP maskable interrupt
//		IntEnable(C674X_MASK_INT4);
//
//		// Enable DSP interrupts globally
//		IntGlobalEnable();

		/* Configuring and enabling the SPI0 instance. */
		SPIReset(SOC_SPI_0_REGS);

		SPIOutOfReset(SOC_SPI_0_REGS);

		SPIModeConfigure(SOC_SPI_0_REGS, SPI_MASTER_MODE);

		SPIClkConfigure(SOC_SPI_0_REGS, 150000000, 10000000, SPI_DATA_FORMAT0);

		/* value to configure SMIO,SOMI,CLK and CS pin as functional pin */
		unsigned int controlRegIdx = 0;
		unsigned int controlReg0 = 0x00000E00;
		controlReg0 |= (1 << SPI_CS);
		SPIPinControl(SOC_SPI_0_REGS, controlRegIdx, 0, &controlReg0);

		SPIDefaultCSSet(SOC_SPI_0_REGS, (1 << SPI_CS));

		/* Configures SPI Data Format Register */
		/* Configures the polarity and phase of SPI clock */
		SPIConfigClkFormat(SOC_SPI_0_REGS, (SPI_CLK_POL_HIGH | SPI_CLK_INPHASE), SPI_DATA_FORMAT0);

		/* Configures SPI to transmit MSB bit First during data transfer */
		SPIShiftMsbFirst(SOC_SPI_0_REGS, SPI_DATA_FORMAT0);

		/* Sets the Charcter length */
		const uint32_t charcterLength = 0x8;
		SPICharLengthSet(SOC_SPI_0_REGS, charcterLength, SPI_DATA_FORMAT0);

		/* Selects the SPI Data format register to used and Sets CSHOLD
		 * to assert CS pin(line)
		 */
		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1 << SPI_CS));

		/* map interrupts to interrupt line INT1 */
		SPIIntLevelSet(SOC_SPI_0_REGS, SPI_RECV_INTLVL | SPI_TRANSMIT_INTLVL);

		/* Enable SPI communication */
		SPIEnable(SOC_SPI_0_REGS);

	}

} /* namespace Lmx2571 */
