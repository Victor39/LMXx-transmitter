#include <stdint.h>
#include <src/Device.h>
#include "soc_OMAPL138.h"
#include "hw_psc_OMAPL138.h"
#include "lcdkOMAPL138.h"
#include "spi.h"
#include "psc.h"
#include "interrupt.h"
#include "gpio.h"

namespace lmx2571 {

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
	static void SPIConfigDataFmtReg (unsigned int dataFormat);
	static void SetUpInt (void);
	static void SetUpSPI (unsigned char cs, unsigned char dcs);
	void SPIIsr (void);
	static void SpiTransfer (unsigned char cs);
	static void enableLatchMICROWIRE ();

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
	 ** Configures ARM interrupt controller to generate SPI interrupt
	 **
	 */
	static void SetUpInt (void) {
// Setup the ARM or DSP interrupt controller

#ifdef _TMS320C6X
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
#else
		/* Initialize the ARM Interrupt Controller.*/
		IntAINTCInit();

		/* Register the ISR in the Interrupt Vector Table.*/
		IntRegister(SYS_INT_SPINT1, SPIIsr);

		/* Set the channnel number 2 of AINTC for system interrupt 56.
		 * Channel 2 is mapped to IRQ interrupt of ARM9.
		 */
		IntChannelSet(SYS_INT_SPINT1, 2);

		/* Enable the System Interrupts for AINTC.*/
		IntSystemEnable(SYS_INT_SPINT1);

		/* Enable IRQ in CPSR.*/
		IntMasterIRQEnable();

		/* Enable the interrupts in GER of AINTC.*/
		IntGlobalEnable();

		/* Enable the interrupts in HIER of AINTC.*/
		IntIRQEnable();
#endif
	}

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
					enableLatchMICROWIRE();
				}
			}

			intCode = SPIInterruptVectorGet(SOC_SPI_0_REGS);
		}
	}

	/*
	 ** Configures SPI Controller
	 **
	 */
	static void SetUpSPI (unsigned char cs, unsigned char dcs) {
		SPIReset(SOC_SPI_0_REGS);

		SPIOutOfReset(SOC_SPI_0_REGS);

		SPIModeConfigure(SOC_SPI_0_REGS, SPI_MASTER_MODE);

		SPIClkConfigure(SOC_SPI_0_REGS, 150000000, 1000000, SPI_DATA_FORMAT0);

		/* value to configure SMIO,SOMI,CLK and CS pin as functional pin */
		unsigned int controlRegIdx = 0;
		unsigned int controlReg0 = 0x00000E00;
		controlReg0 |= cs;
		SPIPinControl(SOC_SPI_0_REGS, controlRegIdx, 0, &controlReg0);

		SPIDefaultCSSet(SOC_SPI_0_REGS, dcs);

		/* Configures SPI Data Format Register */
		SPIConfigDataFmtReg(SPI_DATA_FORMAT0);

		/* Selects the SPI Data format register to used and Sets CSHOLD
		 * to assert CS pin(line)
		 */
		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), cs);

		/* map interrupts to interrupt line INT1 */
		SPIIntLevelSet(SOC_SPI_0_REGS, SPI_RECV_INTLVL | SPI_TRANSMIT_INTLVL);

		/* Enable SPI communication */
		SPIEnable(SOC_SPI_0_REGS);
	}

	/*
	 ** Configures Data Format register of SPI
	 **
	 */
	static void SPIConfigDataFmtReg (unsigned int dataFormat) {
		/* Configures the polarity and phase of SPI clock */
		SPIConfigClkFormat(SOC_SPI_0_REGS, (SPI_CLK_POL_HIGH | SPI_CLK_INPHASE), dataFormat);
//	SPIConfigClkFormat(SOC_SPI_0_REGS, (SPI_CLK_POL_LOW | SPI_CLK_OUTOFPHASE), dataFormat);
//	SPIConfigClkFormat(SOC_SPI_0_REGS, (SPI_CLK_POL_LOW | SPI_CLK_INPHASE), dataFormat);

		/* Configures SPI to transmit MSB bit First during data transfer */
		SPIShiftMsbFirst(SOC_SPI_0_REGS, dataFormat);

		/* Sets the Charcter length */
		SPICharLengthSet(SOC_SPI_0_REGS, CHAR_LENGTH, dataFormat);
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

	static void enableLatchMICROWIRE () {

		const unsigned int timeout = 2;
		GPIOPinWrite(SOC_GPIO_0_REGS, 133, GPIO_PIN_HIGH);
		int i;
		for (i = 0; i < timeout; ++i) {
			asm(" nop");
		}
		GPIOPinWrite(SOC_GPIO_0_REGS, 133, GPIO_PIN_LOW);
	}

////
///
	static void sendCommand (uint16_t length) {

		tx_len = length;
		rx_len = length;

		SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 1 << CS);
		SpiTransfer(0);
	}

	void Device::setTxMode () {
	}

	void Device::setRxMode () {
	}

	bool Device::isTxMode () {
	}

	void Device::isRxMode () {
	}

	void Device::setTxFrequency (const uint32_t constUnsignedInt) {

		// TX R-divider
		txRmult = 4;
		txPostDivider = 1;
		txPreDivider = 1;

		// TX N-divider
		txPreScaler = 1; // 0 = Divide by 2; 1 = Divide by 4
		txNinteger = 15; // 0 ... 1023
		txNden = 8388608; // 0 ... (2^24)-1
		txNnum = 0; // 0 ... (2^24)-1

		//
		txChDiv1 = 0; // 0 = Divide by 4; 1 = Divide by 5; 2 = Divide by 6; 3 = Divide by 7
		txChDiv2 = 6; // 0 = Divide by 1; 1 = Divide by 2; 2 = Divide by 4; . 6 = Divide by 64
	}

	Device::Device () {
		/* The Local PSC number for GPIO is 3. GPIO belongs to PSC1 module.*/
		PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON,
		PSC_MDCTL_NEXT_ENABLE);

		/* Pin Multiplexing of pin 4 of GPIO Bank 8.*/
		GPIOBank8Pin4PinMuxSetup();

		/* Sets the pin 133 (GP8[4]) as output.*/
		GPIODirModeSet(SOC_GPIO_0_REGS, 133, GPIO_DIR_OUTPUT);

		/* Waking up the SPI0 instance. */
		PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_SPI0, PSC_POWERDOMAIN_ALWAYS_ON,
		PSC_MDCTL_NEXT_ENABLE);

		/* Performing the Pin Multiplexing for SPI0. */
		SPIPinMuxSetup(0);

		/*
		 ** Using the Chip Select(CS) 4 pin of SPI0 to communicate with the LMX2571.
		 */
		SPI0CSPinMuxSetup(CS);

		/* Enable use of SPI0 interrupts. */
		SetUpInt();

		/* Configuring and enabling the SPI0 instance. */
		SetUpSPI((1 << CS), 0/*(1 << CS)*/);

		//
		memcpy(m_registers, DEFV_REGISTER, REGISTER_ARR_SIZE*(sizeof(m_registers[0])));

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
