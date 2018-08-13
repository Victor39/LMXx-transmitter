#pragma once
#include <stdint.h>
#include <src/Message.h>
#include <bitset>

namespace lmx2571 {

#define REGISTER_ARR_SIZE (61)

	class Device {
	public:
		static Device& getInstance () {
			static Device instance;
			return instance;
		}

		void setTxMode ();
		void setRxMode ();
		bool isTxMode ();
		void isRxMode ();

		void setTxFrequency (const uint32_t);

		void writeRegister (const uint8_t adrr, const uint16_t data);
		uint16_t readRegister (const uint8_t adrr);

	private:
		static const uint32_t m_Foscin = 20e6;

		// TX R-divider
		uint16_t txRmult; // 0 = Reserved; 1 = Bypass; 2 = 2x ... 13 = 13x; 14-31 = Reserved (must be greater than Pre-divider value)
		uint8_t txPostDivider; // 0 ... 255 ???
		uint8_t txPreDivider; // 0 ... 255 ???

		// RX R-divider
		uint16_t rxRmult; // 0 = Reserved; 1 = Bypass; 2 = 2x ... 13 = 13x; 14-31 = Reserved (must be greater than Pre-divider value)
		uint8_t rxPostDivider; // 0 ... 255 ???
		uint8_t rxPreDivider; // 0 ... 255 ???

		// TX N-divider
		uint8_t txPreScaler; // 0 = Divide by 2; 1 = Divide by 4
		uint16_t txNinteger; // 0 ... 1023
		uint32_t txNden; // 0 ... (2^24)-1
		uint32_t txNnum; // 0 ... (2^24)-1

		// RX N-divider
		uint8_t rxPreScaler; // 0 = Divide by 2; 1 = Divide by 4
		uint16_t rxNinteger; // 0 ... 1023
		uint32_t rxNden; // 0 ... (2^24)-1
		uint32_t rxNnum; // 0 ... (2^24)-1

		//
		uint8_t txChDiv1; // 0 = Divide by 4; 1 = Divide by 5; 2 = Divide by 6; 3 = Divide by 7
		uint8_t txChDiv2; // 0 = Divide by 1; 1 = Divide by 2; 2 = Divide by 4; . 6 = Divide by 64

		// Redisters
		uint16_t m_registers[REGISTER_ARR_SIZE];

		SmartBfArray<Message> m_txMsg;
		SmartBfArray<Message> m_rxMsg;

		Device ();
		Device (Device const&);
		void operator= (Device const&);

		void setBits (uint16_t * reg, uint16_t wstart, uint16_t value, uint16_t rstart, uint16_t num) {

			for (uint16_t i = 0; i < num; ++i) {
				*reg ^= (-(unsigned long) (value >> (rstart + i) & 1UL) ^ *reg) & (1UL << (wstart + i));
			}
		}

	public:
		// Register 0
		void write_FCAL_EN (const uint8_t value) {
			setBits(&m_registers[0], 0, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_F1F2_SEL (const uint8_t value) {
			setBits(&m_registers[0], 6, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_F1F2_MODE (const uint8_t value) {
			setBits(&m_registers[0], 7, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_F1F2_CTRL (const uint8_t value) {
			setBits(&m_registers[0], 8, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_F1F2_INIT (const uint8_t value) {
			setBits(&m_registers[0], 9, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_RXTX_POL (const uint8_t value) {
			setBits(&m_registers[0], 10, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_RXTX_CTRL (const uint8_t value) {
			setBits(&m_registers[0], 11, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_POWERDOWN (const uint8_t value) {
			setBits(&m_registers[0], 12, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}
		void write_RESET (const uint8_t value) {
			setBits(&m_registers[0], 13, value, 0, 1);
			writeRegister(0, m_registers[0]);
		}

		// Register 1 ... 3
		void write_PLL_DEN_F1 (const uint32_t value) {
			setBits(&m_registers[3], 0, value, 0, 16);
			writeRegister(3, m_registers[3]);
			setBits(&m_registers[1], 8, value >> 16, 0, 8);
			writeRegister(1, m_registers[1]);
		}
		void write_PLL_NUM_F1 (const uint32_t value) {
			setBits(&m_registers[2], 0, value, 0, 16);
			writeRegister(2, m_registers[2]);
			setBits(&m_registers[1], 0, value >> 16, 0, 8);
			writeRegister(1, m_registers[1]);
		}

		// Register 4
		void write_PLL_N_F1 (const uint16_t value) {
			setBits(&m_registers[4], 0, value, 0, 12);
			writeRegister(4, m_registers[4]);
		}
		void write_FRAC_ORDER_F1 (const uint8_t value) {
			setBits(&m_registers[4], 12, value, 0, 3);
			writeRegister(4, m_registers[4]);
		}
		void write_PLL_N_PRE_F1 (const uint8_t value) {
			setBits(&m_registers[4], 15, value, 0, 1);
			writeRegister(4, m_registers[4]);
		}

		// Register 5
		void write_PLL_R_PRE_F1 (const uint8_t value) {
			setBits(&m_registers[5], 0, value, 0, 8);
			writeRegister(5, m_registers[5]);
		}
		void write_PLL_R_F1 (const uint8_t value) {
			setBits(&m_registers[26], 8, value, 0, 8);
			writeRegister(5, m_registers[5]);
		}

		// Register 6
		void write_MULT_F1 (const uint8_t value) {
			setBits(&m_registers[6], 0, value, 0, 4);
			writeRegister(6, m_registers[6]);
		}
		void write_PFD_DELAY_F1 (const uint8_t value) {
			setBits(&m_registers[6], 5, value, 0, 3);
			writeRegister(6, m_registers[6]);
		}
		void write_CHDIV1_F1 (const uint8_t value) {
			setBits(&m_registers[6], 8, value, 0, 2);
			writeRegister(6, m_registers[6]);
		}
		void write_CHDIV2_F1 (const uint8_t value) {
			setBits(&m_registers[6], 10, value, 0, 3);
			writeRegister(6, m_registers[6]);
		}
		void write_LF_R3_F1 (const uint8_t value) {
			setBits(&m_registers[6], 13, value, 0, 3);
			writeRegister(6, m_registers[6]);
		}

		// Register 8
		void write_OUTBUF_TX_PWR_F1 (const uint8_t value) {
			setBits(&m_registers[8], 0, value, 0, 5);
			writeRegister(8, m_registers[8]);
		}
		void write_EXTVCO_SEL_F1 (const uint8_t value) {
			setBits(&m_registers[8], 5, value, 0, 1);
			writeRegister(8, m_registers[8]);
		}
		void write_EXTVCO_CHDIV_F1 (const uint8_t value) {
			setBits(&m_registers[8], 6, value, 0, 4);
			writeRegister(8, m_registers[8]);
		}
		void write_FSK_EN_F1 (const uint8_t value) {
			setBits(&m_registers[8], 10, value, 0, 1);
			writeRegister(8, m_registers[8]);
		}

		// Register 17 ... 19
		void write_PLL_DEN_F2 (const uint32_t value) {
			setBits(&m_registers[19], 0, value, 0, 16);
			writeRegister(19, m_registers[19]);
			setBits(&m_registers[17], 8, value >> 16, 0, 8);
			writeRegister(17, m_registers[17]);
		}
		void write_PLL_NUM_F2 (const uint32_t value) {
			setBits(&m_registers[18], 0, value, 0, 16);
			writeRegister(18, m_registers[18]);
			setBits(&m_registers[17], 0, value >> 16, 0, 8);
			writeRegister(17, m_registers[17]);
		}

		// Register 20
		void write_PLL_N_F2 (const uint16_t value) {
			setBits(&m_registers[20], 0, value, 0, 12);
			writeRegister(20, m_registers[20]);
		}
		void write_FRAC_ORDER_F2 (const uint8_t value) {
			setBits(&m_registers[20], 12, value, 0, 3);
			writeRegister(20, m_registers[20]);
		}
		void write_PLL_N_PRE_F2 (const uint8_t value) {
			setBits(&m_registers[20], 15, value, 0, 1);
			writeRegister(20, m_registers[20]);
		}

		// Register 21
		void write_PLL_R_PRE_F2 (const uint8_t value) {
			setBits(&m_registers[21], 0, value, 0, 8);
			writeRegister(21, m_registers[21]);
		}
		void write_PLL_R_F2 (const uint8_t value) {
			setBits(&m_registers[21], 8, value, 0, 8);
			writeRegister(21, m_registers[21]);
		}

		// Register 22
		void write_MULT_F2 (const uint8_t value) {
			setBits(&m_registers[22], 0, value, 0, 4);
			writeRegister(22, m_registers[22]);
		}
		void write_PFD_DELAY_F2 (const uint8_t value) {
			setBits(&m_registers[22], 5, value, 0, 3);
			writeRegister(22, m_registers[22]);
		}
		void write_CHDIV1_F2 (const uint8_t value) {
			setBits(&m_registers[22], 8, value, 0, 2);
			writeRegister(22, m_registers[22]);
		}
		void write_CHDIV2_F2 (const uint8_t value) {
			setBits(&m_registers[22], 10, value, 0, 3);
			writeRegister(22, m_registers[22]);
		}
		void write_LF_R3_F2 (const uint8_t value) {
			setBits(&m_registers[22], 13, value, 0, 3);
			writeRegister(22, m_registers[22]);
		}

		// Register 24
		void write_OUTBUF_TX_PWR_F2 (const uint8_t value) {
			setBits(&m_registers[24], 0, value, 0, 5);
			writeRegister(24, m_registers[24]);
		}
		void write_EXTVCO_SEL_F2 (const uint8_t value) {
			setBits(&m_registers[24], 5, value, 0, 1);
			writeRegister(24, m_registers[24]);
		}
		void write_EXTVCO_CHDIV_F2 (const uint8_t value) {
			setBits(&m_registers[24], 6, value, 0, 4);
			writeRegister(24, m_registers[24]);
		}
		void write_FSK_EN_F2 (const uint8_t value) {
			setBits(&m_registers[24], 10, value, 0, 1);
			writeRegister(24, m_registers[24]);
		}

		// Register 33
		void write_FSK_DEV_SPI_FAST (const uint16_t value) {
			setBits(&m_registers[33], 0, value, 0, 16);
			writeRegister(33, m_registers[33]);
		}

		// Register 34
		void write_FSK_MODE_SEL1 (const uint8_t value) {
			setBits(&m_registers[34], 0, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_FSK_MODE_SEL0 (const uint8_t value) {
			setBits(&m_registers[34], 1, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_FSK_DEV_SEL (const uint8_t value) {
			setBits(&m_registers[34], 2, value, 0, 3);
			writeRegister(34, m_registers[34]);
		}
		void write_FSK_LEVEL (const uint8_t value) {
			setBits(&m_registers[34], 5, value, 0, 2);
			writeRegister(34, m_registers[34]);
		}
		void write_FSK_I2S_CLK_POL (const uint8_t value) {
			setBits(&m_registers[34], 7, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_FSK_I2S_FS_POL (const uint8_t value) {
			setBits(&m_registers[34], 8, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_XTAL_EN (const uint8_t value) {
			setBits(&m_registers[34], 10, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_XTAL_PWRCTRL (const uint8_t value) {
			setBits(&m_registers[34], 11, value, 0, 3);
			writeRegister(34, m_registers[34]);
		}
		void write_IPBUF_SE_DIFF_SEL (const uint8_t value) {
			setBits(&m_registers[34], 14, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}
		void write_IPBUFDIFF_TERM (const uint8_t value) {
			setBits(&m_registers[34], 15, value, 0, 1);
			writeRegister(34, m_registers[34]);
		}

		// Register 39
		void write_LD_EN (const uint8_t value) {
			setBits(&m_registers[39], 0, value, 0, 1);
			writeRegister(39, m_registers[39]);
		}
		void write_SDO_LD_SEL (const uint8_t value) {
			setBits(&m_registers[39], 3, value, 0, 1);
			writeRegister(39, m_registers[39]);
		}

	};

} /* namespace Lmx2571 */
