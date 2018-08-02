#include "soc_OMAPL138.h"
#include "hw_psc_OMAPL138.h"
#include "lcdkOMAPL138.h"
#include "spi.h"
#include "psc.h"
#include "interrupt.h"
#include "gpio.h"

/******************************************************************************
 **                      INTERNAL MACRO DEFINITIONS
 *******************************************************************************/
#define CHAR_LENGTH             0x8

/******************************************************************************
 **                      INTERNAL FUNCTION PROTOTYPES
 *******************************************************************************/
static void SPIConfigDataFmtReg (unsigned int dataFormat);
static void SetUpInt (void);
static void SetUpSPI (unsigned char cs, unsigned char dcs);
void SPIIsr (void);
static void SpiTransfer (unsigned char cs);
static void GetStatusCommand (unsigned char cs);
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

int main (void) {

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
	unsigned char CS = 4;
	SPI0CSPinMuxSetup(CS);

	/* Enable use of SPI0 interrupts. */
	SetUpInt();

	/* Configuring and enabling the SPI0 instance. */
	SetUpSPI((1 << CS), 0/*(1 << CS)*/);
//
//    /* Issue a Reset to the Fingerprint Sensor */
//    ResetCommand();

	volatile int stop = 0;
	while (!stop) {
		GetStatusCommand((1 << CS));
		asm(" nop");
	}

	return (0);
}

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
			p_rx++;
			if (!rx_len) {
				flag = 0;
				enableLatchMICROWIRE();
				SPIIntDisable(SOC_SPI_0_REGS, SPI_RECV_INT);
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

	const unsigned int timeout = 10;
	GPIOPinWrite(SOC_GPIO_0_REGS, 133, GPIO_PIN_HIGH);
	int i;
	for (i = 0; i < timeout; ++i) {
		asm(" nop");
	}
	GPIOPinWrite(SOC_GPIO_0_REGS, 133, GPIO_PIN_LOW);
}

////
///
static void GetStatusCommand (unsigned char cs) {
//	unsigned int i;

// Issue Get Status command and read Status Message Response
	tx_data[0] = 0x87;
	tx_data[1] = 0xA5;
	tx_data[2] = 0x87;

	tx_len = 3;
	rx_len = 3;

	SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), cs);
	SpiTransfer(0);

//    for(i=0;i<16;i++)
//    {
//    	StatusResponseMessage[i] = rx_data[i+7];
//    }
}

//static void ResetCommand(void)
//{
//	// Issue Reset Command
//	tx_data[0] = 0x80;
//	tx_data[1] = 0x14;
//	tx_data[2] = 0xBA;
//	tx_data[3] = 0x07;
//	tx_data[4] = 0x00;
//	tx_data[5] = 0x00;
//	tx_data[6] = 0x00;
//
//	tx_len = 7;
//	rx_len = 7;
//
//	SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x4);
//    SpiTransfer();
//}
