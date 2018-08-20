#include <stdint.h>
#include <math.h>
#include <src/Device.h>
#include "soc_OMAPL138.h"
#include "hw_psc_OMAPL138.h"
#include "lcdkOMAPL138.h"
#include "spi.h"
#include "psc.h"
#include "interrupt.h"
#include "gpio.h"

namespace lmx2571 {

#define MIN_TX_FREQ	(30000000UL)
#define MAX_TX_FREQ	(108000000UL)
#define TX_FREQ_SPACING	(12500UL)

#define MIN_TX_FREQ_INDEX (0)
#define MAX_TX_FREQ_INDEX ((MAX_TX_FREQ - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 30,0 - 33,6 MHz
#define LOW_TX_FREQ_RANGE1 (30000000UL)
#define HIGH_TX_FREQ_RANGE1 (33600000UL)
#define LOW_TX_FREQ_INDEX_RANGE1 ((LOW_TX_FREQ_RANGE1 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE1 ((HIGH_TX_FREQ_RANGE1 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 33,6125 - 40 MHz
#define LOW_TX_FREQ_RANGE2 (33612500UL)
#define HIGH_TX_FREQ_RANGE2 (40000000UL)
#define LOW_TX_FREQ_INDEX_RANGE2 ((LOW_TX_FREQ_RANGE2 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE2 ((HIGH_TX_FREQ_RANGE2 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range  40,0125…46 MHz
#define LOW_TX_FREQ_RANGE3 (40012500UL)
#define HIGH_TX_FREQ_RANGE3 (46000000UL)
#define LOW_TX_FREQ_INDEX_RANGE3 ((LOW_TX_FREQ_RANGE3 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE3 ((HIGH_TX_FREQ_RANGE3 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 46,0125…54 MHz
#define LOW_TX_FREQ_RANGE4 (46012500UL)
#define HIGH_TX_FREQ_RANGE4 (54000000UL)
#define LOW_TX_FREQ_INDEX_RANGE4 ((LOW_TX_FREQ_RANGE4 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE4 ((HIGH_TX_FREQ_RANGE4 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 54,0125…67,2 MHz
#define LOW_TX_FREQ_RANGE5 (54012500UL)
#define HIGH_TX_FREQ_RANGE5 (67200000UL)
#define LOW_TX_FREQ_INDEX_RANGE5 ((LOW_TX_FREQ_RANGE5 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE5 ((HIGH_TX_FREQ_RANGE5 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 62,2125…78,0 MHz
#define LOW_TX_FREQ_RANGE6 (67212500UL)
#define HIGH_TX_FREQ_RANGE6 (78000000UL)
#define LOW_TX_FREQ_INDEX_RANGE6 ((LOW_TX_FREQ_RANGE6 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE6 ((HIGH_TX_FREQ_RANGE6 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 78,0125…92 MHz
#define LOW_TX_FREQ_RANGE7 (78012500UL)
#define HIGH_TX_FREQ_RANGE7 (92000000UL)
#define LOW_TX_FREQ_INDEX_RANGE7 ((LOW_TX_FREQ_RANGE7 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE7 ((HIGH_TX_FREQ_RANGE7 - MIN_TX_FREQ) / TX_FREQ_SPACING)

// Range 92,0125…108 MHz
#define LOW_TX_FREQ_RANGE8 (92012500UL)
#define HIGH_TX_FREQ_RANGE8 (108000000UL)
#define LOW_TX_FREQ_INDEX_RANGE8 ((LOW_TX_FREQ_RANGE8 - MIN_TX_FREQ) / TX_FREQ_SPACING)
#define HIGH_TX_FREQ_INDEX_RANGE8 ((HIGH_TX_FREQ_RANGE8 - MIN_TX_FREQ) / TX_FREQ_SPACING)

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

	/******************************************************************************
	 **                      INTERNAL MACRO DEFINITIONS
	 *******************************************************************************/
#define CHAR_LENGTH             0x8
#define CS						4

	/******************************************************************************
	 **                      INTERNAL FUNCTION PROTOTYPES
	 *******************************************************************************/
	void SPIIsr (void);
	static void SpiTransfer (unsigned char cs);

	/******************************************************************************
	 **                      INTERNAL VARIABLE DEFINITIONS
	 *******************************************************************************/
	volatile unsigned int flag = 1;
	unsigned int tx_len;
	unsigned int rx_len;
	unsigned char vrf_data[260];
	unsigned char tx_data[260];
	volatile unsigned char rx_data[260];
	unsigned char *p_tx;
	volatile unsigned char *p_rx;
	volatile unsigned char StatusResponseMessage[10];

	/*
	 ** Data transmission and receiption SPIIsr
	 **
	 */
	void SPIIsr (void) {
		unsigned int intCode = 0;

#ifdef _TMS320C6X
		IntEventClear(SYS_INT_SPI0_INT);
#else
		IntSystemStatusClear(56);
#endif

		intCode = SPIInterruptVectorGet(SOC_SPI_0_REGS);

		while (intCode) {
			if (intCode == SPI_TX_BUF_EMPTY) {
				tx_len--;
				SPITransmitData1(SOC_SPI_0_REGS, *p_tx);
				p_tx++;
				if (!tx_len) {
					SPIIntDisable(SOC_SPI_0_REGS, SPI_TRANSMIT_INT);
				}
			}

			if (intCode == SPI_RECV_FULL) {
				rx_len--;
				*p_rx = (char) SPIDataReceive(SOC_SPI_0_REGS);
				if (*p_rx != 0) {
					asm(" nop");
				}

				p_rx++;
				if (!rx_len) {
					flag = 0;
					SPIIntDisable(SOC_SPI_0_REGS, SPI_RECV_INT);
				}
			}

			intCode = SPIInterruptVectorGet(SOC_SPI_0_REGS);
		}
	}

	/*
	 ** Enables SPI Transmit and Receive interrupt.
	 ** Deasserts Chip Select line.
	 */
	static void SpiTransfer (unsigned char cs) {
		p_tx = &tx_data[0];
		p_rx = &rx_data[0];
		SPIIntEnable(SOC_SPI_0_REGS, (SPI_RECV_INT | SPI_TRANSMIT_INT));
		while (flag)
			;
		flag = 1;
		/* Deasserts the CS pin(line) */
		SPIDat1Config(SOC_SPI_0_REGS, SPI_DATA_FORMAT0, cs);
	}

	static void sendCommand (uint16_t length) {

		tx_len = length;
		rx_len = length;

		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 1 << CS);
		SpiTransfer(0);
	}

	void Device::setTxMode () {
		write_PLL_N_PRE_F1_R4(1); // 0 = Divide by 2; 1 = Divide by 4
		setTxFrequencyBy(0);
	}

	bool Device::setTxFrequencyBy (const uint32_t freqIndex) {

		bool result = true;

		// R-divider
		uint16_t Rmult; // 0 = Reserved; 1 = Bypass; 2 = 2x ... 13 = 13x; 14-31 = Reserved (must be greater than Pre-divider value)
		uint8_t RpreDivider; // 0 ... 255
		uint8_t RpostDivider; // 0 ... 255 ???

		// N-divider
		uint16_t Ninteger; // 0 ... 1023
		uint32_t Nden; // 0 ... (2^24)-1
		uint32_t Nnum; // 0 ... (2^24)-1

		// Out-divider
		uint8_t chDiv1; // 0 = Divide by 4; 1 = Divide by 5; 2 = Divide by 6; 3 = Divide by 7
		uint8_t chDiv2; // 0 = Divide by 1; 1 = Divide by 2; 2 = Divide by 4; . 6 = Divide by 64

		if ((LOW_TX_FREQ_INDEX_RANGE1 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE1)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 5;
			RpostDivider = 5;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE1);
			Ninteger = 240 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 1; // 5
			chDiv2 = 5; // 32
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE2 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE2)) {
			// R-divider
			RpreDivider = 5;
			Rmult = 6;
			RpostDivider = 6;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE2) + 9;
			Ninteger = 268 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 0; // 4
			chDiv2 = 5; // 32
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE3 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE3)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 7;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE3) + 1;
			Ninteger = 320 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 3; // 7
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE4 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE4)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 6;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE4) + 1;
			Ninteger = 368 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 2; // 6
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE5 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE5)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 5;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE5) + 1;
			Ninteger = 432 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 1; // 5
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE6 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE6)) {
			// R-divider
			RpreDivider = 1;
			Rmult = 1;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE6) + 7;
			Ninteger = 537 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 0; // 4
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE7 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE7)) {
			// R-divider
			RpreDivider = 2;
			Rmult = 7;
			RpostDivider = 40;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE7) + 1;
			Ninteger = 624 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 3; // 7
			chDiv2 = 3; // 8
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE8 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE8)) {
			// R-divider
			RpreDivider = 1;
			Rmult = 3;
			RpostDivider = 40;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE8) + 1;
			Ninteger = 736 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 2; // 6
			chDiv2 = 3; // 8
		}
		else {
			result = false;
		}

		if (result) {
			set_PLL_R_PRE_F1_R5(RpreDivider);
			set_MULT_F1_R6(Rmult);
			set_PLL_R_F1_R5(RpostDivider);

			set_PLL_N_F1_R4(Ninteger);
			set_PLL_DEN_F1_R1R3(Nden);
			set_PLL_NUM_F1_R1R2(Nnum);

			set_CHDIV1_F1_R6(chDiv1);
			set_CHDIV2_F1_R6(chDiv2);

			set_FCAL_EN_R0(1);

			writeRegister(6);
			writeRegister(5);
			writeRegister(4);
			writeRegister(3);
			writeRegister(2);
			writeRegister(0);
		}
		return result;
	}


	bool Device::setReduceTxFrequencyBy (const uint32_t freqIndex) {

		bool result = true;

		// R-divider
		uint16_t Rmult; // 0 = Reserved; 1 = Bypass; 2 = 2x ... 13 = 13x; 14-31 = Reserved (must be greater than Pre-divider value)
		uint8_t RpreDivider; // 0 ... 255
		uint8_t RpostDivider; // 0 ... 255 ???

		// N-divider
		uint16_t Ninteger; // 0 ... 1023
		uint32_t Nden; // 0 ... (2^24)-1
		uint32_t Nnum; // 0 ... (2^24)-1

		// Out-divider
		uint8_t chDiv1; // 0 = Divide by 4; 1 = Divide by 5; 2 = Divide by 6; 3 = Divide by 7
		uint8_t chDiv2; // 0 = Divide by 1; 1 = Divide by 2; 2 = Divide by 4; . 6 = Divide by 64

		if ((LOW_TX_FREQ_INDEX_RANGE1 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE1)) {
			// R-divider
//			RpreDivider = 4;
//			Rmult = 5;
//			RpostDivider = 5;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE1);
			Ninteger = 240 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
//			// Out-divider
//			chDiv1 = 1; // 5
//			chDiv2 = 5; // 32
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE2 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE2)) {
			// R-divider
			RpreDivider = 5;
			Rmult = 6;
			RpostDivider = 6;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE2) + 9;
			Ninteger = 268 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 0; // 4
			chDiv2 = 5; // 32
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE3 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE3)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 7;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE3) + 1;
			Ninteger = 320 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 3; // 7
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE4 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE4)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 6;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE4) + 1;
			Ninteger = 368 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 2; // 6
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE5 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE5)) {
			// R-divider
			RpreDivider = 4;
			Rmult = 5;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE5) + 1;
			Ninteger = 432 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 1; // 5
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE6 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE6)) {
			// R-divider
			RpreDivider = 1;
			Rmult = 1;
			RpostDivider = 10;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE6) + 7;
			Ninteger = 537 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 0; // 4
			chDiv2 = 4; // 16
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE7 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE7)) {
			// R-divider
			RpreDivider = 2;
			Rmult = 7;
			RpostDivider = 40;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE7) + 1;
			Ninteger = 624 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
			chDiv1 = 3; // 7
			chDiv2 = 3; // 8
		}
		else if ((LOW_TX_FREQ_INDEX_RANGE8 <= freqIndex) && (freqIndex <= HIGH_TX_FREQ_INDEX_RANGE8)) {
			// R-divider
//			RpreDivider = 1;
//			Rmult = 3;
//			RpostDivider = 40;
			// N-divider
			uint32_t extDivValue = (freqIndex - LOW_TX_FREQ_INDEX_RANGE8) + 1;
			Ninteger = 736 + extDivValue / 10;
			Nden = 10;
			Nnum = extDivValue % 10;
			// Out-divider
//			chDiv1 = 2; // 6
//			chDiv2 = 3; // 8
		}
		else {
			result = false;
		}

		if (result) {
//			set_PLL_R_PRE_F1_R5(RpreDivider);
//			set_MULT_F1_R6(Rmult);
//			set_PLL_R_F1_R5(RpostDivider);

			set_PLL_N_F1_R4(Ninteger);
//			set_PLL_DEN_F1_R1R3(Nden);
			set_PLL_NUM_F1_R1R2(Nnum);

//			set_CHDIV1_F1_R6(chDiv1);
//			set_CHDIV2_F1_R6(chDiv2);

			set_FCAL_EN_R0(1);

//			writeRegister(6);
//			writeRegister(5);
			writeRegister(4);
//			writeRegister(3);
			writeRegister(2);
			writeRegister(0);
		}
		return result;
	}
	Device::Device () {

		// SPI

		/* Waking up the SPI0 instance. */
		PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_SPI0, PSC_POWERDOMAIN_ALWAYS_ON,
		PSC_MDCTL_NEXT_ENABLE);

		/* Performing the Pin Multiplexing for SPI0. */
		SPIPinMuxSetup(0);

		/* Using the Chip Select(CS) 4 pin of SPI0 to communicate with the LMX2571. */
		SPI0CSPinMuxSetup(CS);

		/* Enable use of SPI0 interrupts. */
		// Initialize the DSP interrupt controller
		IntDSPINTCInit();

		// Register the ISR in the vector table
		IntRegister(C674X_MASK_INT4, SPIIsr);

		// Map system interrupt to the DSP maskable interrupt
		IntEventMap(C674X_MASK_INT4, SYS_INT_SPI0_INT);

		// Enable the DSP maskable interrupt
		IntEnable(C674X_MASK_INT4);

		// Enable DSP interrupts globally
		IntGlobalEnable();

		/* Configuring and enabling the SPI0 instance. */
		SPIReset(SOC_SPI_0_REGS);

		SPIOutOfReset(SOC_SPI_0_REGS);

		SPIModeConfigure(SOC_SPI_0_REGS, SPI_MASTER_MODE);

		SPIClkConfigure(SOC_SPI_0_REGS, 150000000, 1000000, SPI_DATA_FORMAT0);

		/* value to configure SMIO,SOMI,CLK and CS pin as functional pin */
		unsigned int controlRegIdx = 0;
		unsigned int controlReg0 = 0x00000E00;
		controlReg0 |= (1 << CS);
		SPIPinControl(SOC_SPI_0_REGS, controlRegIdx, 0, &controlReg0);

		SPIDefaultCSSet(SOC_SPI_0_REGS, (1 << CS));

		/* Configures SPI Data Format Register */
		/* Configures the polarity and phase of SPI clock */
		SPIConfigClkFormat(SOC_SPI_0_REGS, (SPI_CLK_POL_HIGH | SPI_CLK_INPHASE), SPI_DATA_FORMAT0);

		/* Configures SPI to transmit MSB bit First during data transfer */
		SPIShiftMsbFirst(SOC_SPI_0_REGS, SPI_DATA_FORMAT0);

		/* Sets the Charcter length */
		SPICharLengthSet(SOC_SPI_0_REGS, CHAR_LENGTH, SPI_DATA_FORMAT0);

		/* Selects the SPI Data format register to used and Sets CSHOLD
		 * to assert CS pin(line)
		 */
		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1 << CS));

		/* map interrupts to interrupt line INT1 */
		SPIIntLevelSet(SOC_SPI_0_REGS, SPI_RECV_INTLVL | SPI_TRANSMIT_INTLVL);

		/* Enable SPI communication */
		SPIEnable(SOC_SPI_0_REGS);

		memcpy(m_registers, DEFV_REGISTER, REGISTER_ARR_SIZE * (sizeof(m_registers[0])));
	}

	void Device::setBits (uint16_t * reg, uint16_t wstart, uint16_t value, uint16_t rstart, uint16_t num) {

		for (uint16_t i = 0; i < num; ++i) {
			*reg ^= (-(unsigned long) (value >> (rstart + i) & 1UL) ^ *reg) & (1UL << (wstart + i));
		}
	}

	void Device::writeRegister (const uint8_t adrr, const uint16_t data) {

		m_txMsg().rw = 0;
		m_txMsg().address = adrr;
		m_txMsg().data = data;
		m_txMsg.pack(tx_data);
		sendCommand(m_txMsg.sizeBytes());
	}

	uint16_t Device::readRegister (const uint8_t adrr) {
		m_txMsg().rw = 1;
		m_txMsg().address = adrr;

		m_txMsg.pack(tx_data);

		sendCommand(m_rxMsg.sizeBytes());
		m_rxMsg.extract((uint8_t*) rx_data);

		return m_rxMsg().data.value<uint16_t>();
	}

} /* namespace Lmx2571 */
