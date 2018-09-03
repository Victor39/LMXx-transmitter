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

		void startUp();
		void setTxMode_1 ();
		bool setTxFrequencyBy_1 (const uint32_t freqIndex);

		void setTxMode_2 ();
		bool setTxFrequencyBy_2 (const uint32_t freqIndex);
		bool setTxFrequency_2(const uint32_t freq);

		void writeRegister (const uint8_t adrr, const uint16_t data);
		void writeRegister (const uint8_t adrr) {
			writeRegister(adrr, m_registers[adrr]);
		}
		uint16_t readRegister (const uint8_t adrr);

	private:

		// Redisters
		uint16_t m_registers[REGISTER_ARR_SIZE];

		SmartBfArray<Message> m_txMsg;
		SmartBfArray<Message> m_rxMsg;

		Device ();
		Device (Device const&);
		void operator= (Device const&);
		void setBits (uint16_t * reg, uint16_t wstart, uint16_t value, uint16_t rstart, uint16_t num);

	public:
		// Register 0
		void set_FCAL_EN_R0 (const uint8_t value) {
			setBits(&m_registers[0], 0, value, 0, 1);
		}
		void write_FCAL_EN_R0 (const uint8_t value) {
			set_FCAL_EN_R0(value);
			writeRegister(0);
		}

		void set_F1F2_SEL_R0 (const uint8_t value) {
			setBits(&m_registers[0], 6, value, 0, 1);
		}
		void write_F1F2_SEL_R0 (const uint8_t value) {
			set_F1F2_SEL_R0(value);
			writeRegister(0);
		}

		void set_F1F2_MODE_R0 (const uint8_t value) {
			setBits(&m_registers[0], 7, value, 0, 1);
		}
		void write_F1F2_MODE_R0 (const uint8_t value) {
			set_F1F2_MODE_R0(value);
			writeRegister(0);
		}

		void set_F1F2_CTRL_R0 (const uint8_t value) {
			setBits(&m_registers[0], 8, value, 0, 1);
		}
		void write_F1F2_CTRL_R0 (const uint8_t value) {
			set_F1F2_CTRL_R0(value);
			writeRegister(0);
		}

		void set_F1F2_INIT_R0 (const uint8_t value) {
			setBits(&m_registers[0], 9, value, 0, 1);
		}
		void write_F1F2_INIT_R0 (const uint8_t value) {
			set_F1F2_INIT_R0(value);
			writeRegister(0);
		}

		void set_RXTX_POL_R0 (const uint8_t value) {
			setBits(&m_registers[0], 10, value, 0, 1);
		}
		void write_RXTX_POL (const uint8_t value) {
			set_RXTX_POL_R0(value);
			writeRegister(0);
		}

		void set_RXTX_CTRL_R0 (const uint8_t value) {
			setBits(&m_registers[0], 11, value, 0, 1);
		}
		void write_RXTX_CTRL_R0 (const uint8_t value) {
			set_RXTX_CTRL_R0(value);
			writeRegister(0);
		}

		void set_POWERDOWN_R0 (const uint8_t value) {
			setBits(&m_registers[0], 12, value, 0, 1);
		}
		void write_POWERDOWN_R0 (const uint8_t value) {
			set_POWERDOWN_R0(value);
			writeRegister(0);
		}

		void set_RESET_R0 (const uint8_t value) {
			setBits(&m_registers[0], 13, value, 0, 1);
		}
		void write_RESET_R0 (const uint8_t value) {
			set_RESET_R0(value);
			writeRegister(0);
		}

		// Register 1 ... 3
		void set_PLL_DEN_F1_R1R3 (const uint32_t value) {
			setBits(&m_registers[3], 0, value, 0, 16);
			setBits(&m_registers[1], 8, value >> 16, 0, 8);
		}
		void write_PLL_DEN_F1_R1R3 (const uint32_t value) {
			set_PLL_DEN_F1_R1R3(value);
			writeRegister(3);
			writeRegister(1);
		}

		void set_PLL_NUM_F1_R1R2 (const uint32_t value) {
			setBits(&m_registers[2], 0, value, 0, 16);
			setBits(&m_registers[1], 0, value >> 16, 0, 8);
		}
		void write_PLL_NUM_F1_R1R2 (const uint32_t value) {
			set_PLL_NUM_F1_R1R2(value);
			writeRegister(2);
			writeRegister(1);
		}

		// Register 4
		void set_PLL_N_F1_R4 (const uint16_t value) {
			setBits(&m_registers[4], 0, value, 0, 12);
		}
		void write_PLL_N_F1_R4 (const uint16_t value) {
			set_PLL_N_F1_R4(value);
			writeRegister(4);
		}

		void set_FRAC_ORDER_F1_R4 (const uint8_t value) {
			setBits(&m_registers[4], 12, value, 0, 3);
		}
		void write_FRAC_ORDER_F1_R4 (const uint8_t value) {
			set_FRAC_ORDER_F1_R4(value);
			writeRegister(4);
		}

		void set_PLL_N_PRE_F1_R4 (const uint8_t value) {
			// 0 = Divide by 2; 1 = Divide by 4
			setBits(&m_registers[4], 15, value, 0, 1);
		}
		void write_PLL_N_PRE_F1_R4 (const uint8_t value) {
			// 0 = Divide by 2; 1 = Divide by 4
			set_PLL_N_PRE_F1_R4(value);
			writeRegister(4);
		}

		// Register 5
		void set_PLL_R_PRE_F1_R5 (const uint8_t value) {
			setBits(&m_registers[5], 0, value, 0, 8);
		}
		void write_PLL_R_PRE_F1_R5 (const uint8_t value) {
			set_PLL_R_PRE_F1_R5(value);
			writeRegister(5);
		}

		void set_PLL_R_F1_R5 (const uint8_t value) {
			setBits(&m_registers[5], 8, value, 0, 8);
		}
		void write_PLL_R_F1_R5 (const uint8_t value) {
			set_PLL_R_F1_R5(value);
			writeRegister(5);
		}

		// Register 6
		void set_MULT_F1_R6 (const uint8_t value) {
			setBits(&m_registers[6], 0, value, 0, 5);
		}
		void write_MULT_F1_R6 (const uint8_t value) {
			set_MULT_F1_R6(value);
			writeRegister(6);
		}

		void set_PFD_DELAY_F1_R6 (const uint8_t value) {
			setBits(&m_registers[6], 5, value, 0, 3);
		}
		void write_PFD_DELAY_F1_R6 (const uint8_t value) {
			set_PFD_DELAY_F1_R6(value);
			writeRegister(6);
		}

		void set_CHDIV1_F1_R6 (const uint8_t value) {
			setBits(&m_registers[6], 8, value, 0, 2);
		}
		void write_CHDIV1_F1_R6 (const uint8_t value) {
			set_CHDIV1_F1_R6(value);
			writeRegister(6);
		}

		void set_CHDIV2_F1_R6 (const uint8_t value) {
			setBits(&m_registers[6], 10, value, 0, 3);
		}
		void write_CHDIV2_F1_R6 (const uint8_t value) {
			set_CHDIV2_F1_R6(value);
			writeRegister(6);
		}

		void set_LF_R3_F1_R6 (const uint8_t value) {
			setBits(&m_registers[6], 13, value, 0, 3);
		}
		void write_LF_R3_F1_R6 (const uint8_t value) {
			set_LF_R3_F1_R6(value);
			writeRegister(6);
		}

		// Register 7
		void set_LF_R4_F1_R7 (const uint8_t value) {
			setBits(&m_registers[7], 0, value, 0, 3);
		}
		void write_LF_R4_F1_R7 (const uint8_t value) {
			set_LF_R4_F1_R7(value);
			writeRegister(7);
		}

		void set_OUTBUF_RX_EN_F1_R7 (const uint8_t value) {
			setBits(&m_registers[7], 6, value, 0, 1);
		}
		void write_OUTBUF_RX_EN_F1_R7 (const uint8_t value) {
			set_OUTBUF_RX_EN_F1_R7(value);
			writeRegister(7);
		}

		void set_OUTBUF_TX_EN_F1_R7 (const uint8_t value) {
			setBits(&m_registers[7], 7, value, 0, 1);
		}
		void write_OUTBUF_TX_EN_F1_R7 (const uint8_t value) {
			set_OUTBUF_TX_EN_F1_R7(value);
			writeRegister(7);
		}

		void set_OUTBUF_RX_PWR_F1_R7 (const uint8_t value) {
			setBits(&m_registers[7], 8, value, 0, 5);
		}
		void write_OUTBUF_RX_PWR_F1_R7 (const uint8_t value) {
			set_OUTBUF_RX_PWR_F1_R7(value);
			writeRegister(7);
		}

		// Register 8
		void set_OUTBUF_TX_PWR_F1_R8 (const uint8_t value) {
			setBits(&m_registers[8], 0, value, 0, 5);
		}
		void write_OUTBUF_TX_PWR_F1_R8 (const uint8_t value) {
			set_OUTBUF_TX_PWR_F1_R8(value);
			writeRegister(8);
		}

		void set_EXTVCO_SEL_F1_R8 (const uint8_t value) {
			setBits(&m_registers[8], 5, value, 0, 1);
		}
		void write_EXTVCO_SEL_F1_R8 (const uint8_t value) {
			set_EXTVCO_SEL_F1_R8(value);
			writeRegister(8);
		}

		void set_EXTVCO_CHDIV_F1_R8 (const uint8_t value) {
			setBits(&m_registers[8], 6, value, 0, 4);
		}
		void write_EXTVCO_CHDIV_F1_R8 (const uint8_t value) {
			set_EXTVCO_CHDIV_F1_R8(value);
			writeRegister(8);
		}

		void set_FSK_EN_F1_R8 (const uint8_t value) {
			setBits(&m_registers[8], 10, value, 0, 1);
		}
		void write_FSK_EN_F1_R8 (const uint8_t value) {
			set_FSK_EN_F1_R8(value);
			writeRegister(8);
		}

		// Register 17 ... 19
		void set_PLL_DEN_F2_R17R19 (const uint32_t value) {
			setBits(&m_registers[19], 0, value, 0, 16);
			setBits(&m_registers[17], 8, value >> 16, 0, 8);
		}
		void write_PLL_DEN_F2_R17R19 (const uint32_t value) {
			set_PLL_DEN_F2_R17R19(value);
			writeRegister(19);
			writeRegister(17);
		}

		void set_PLL_NUM_F2_R17R18 (const uint32_t value) {
			setBits(&m_registers[18], 0, value, 0, 16);
			setBits(&m_registers[17], 0, value >> 16, 0, 8);

		}
		void write_PLL_NUM_F2_R17R18 (const uint32_t value) {
			set_PLL_NUM_F2_R17R18(value);
			writeRegister(18);
			writeRegister(17);
		}

		// Register 20
		void set_PLL_N_F2_R20 (const uint16_t value) {
			setBits(&m_registers[20], 0, value, 0, 12);
		}
		void write_PLL_N_F2_R20 (const uint16_t value) {
			set_PLL_N_F2_R20(value);
			writeRegister(20);
		}

		void set_FRAC_ORDER_F2_R20 (const uint8_t value) {
			setBits(&m_registers[20], 12, value, 0, 3);
		}
		void write_FRAC_ORDER_F2_R20 (const uint8_t value) {
			set_FRAC_ORDER_F2_R20(value);
			writeRegister(20);
		}

		void set_PLL_N_PRE_F2_R20 (const uint8_t value) {
			setBits(&m_registers[20], 15, value, 0, 1);
		}
		void write_PLL_N_PRE_F2_R20 (const uint8_t value) {
			set_PLL_N_PRE_F2_R20(value);
			writeRegister(20);
		}

		// Register 21
		void set_PLL_R_PRE_F2_R21 (const uint8_t value) {
			setBits(&m_registers[21], 0, value, 0, 8);
		}
		void write_PLL_R_PRE_F2_R21 (const uint8_t value) {
			set_PLL_R_PRE_F2_R21(value);
			writeRegister(21);
		}
		void set_PLL_R_F2_R21 (const uint8_t value) {
			setBits(&m_registers[21], 8, value, 0, 8);
		}
		void write_PLL_R_F2_R21 (const uint8_t value) {
			set_PLL_R_F2_R21(value);
			writeRegister(21);
		}

		// Register 22
		void set_MULT_F2_R22 (const uint8_t value) {
			setBits(&m_registers[22], 0, value, 0, 4);
		}
		void write_MULT_F2_R22 (const uint8_t value) {
			set_MULT_F2_R22(value);
			writeRegister(22);
		}

		void set_PFD_DELAY_F2_R22 (const uint8_t value) {
			setBits(&m_registers[22], 5, value, 0, 3);
		}
		void write_PFD_DELAY_F2_R22 (const uint8_t value) {
			set_PFD_DELAY_F2_R22(value);
			writeRegister(22);
		}

		void set_CHDIV1_F2_R22 (const uint8_t value) {
			setBits(&m_registers[22], 8, value, 0, 2);
		}
		void write_CHDIV1_F2_R22 (const uint8_t value) {
			set_CHDIV1_F2_R22(value);
			writeRegister(22);
		}

		void set_CHDIV2_F2_R22 (const uint8_t value) {
			setBits(&m_registers[22], 10, value, 0, 3);
		}
		void write_CHDIV2_F2_R22 (const uint8_t value) {
			set_CHDIV2_F2_R22(value);
			writeRegister(22);
		}

		void set_LF_R3_F2_R22 (const uint8_t value) {
			setBits(&m_registers[22], 13, value, 0, 3);
		}
		void write_LF_R3_F2_R22 (const uint8_t value) {
			set_LF_R3_F2_R22(value);
			writeRegister(22);
		}

		// Register 23
		void set_LF_R4_F2_R23 (const uint8_t value) {
			setBits(&m_registers[23], 0, value, 0, 3);
		}
		void write_LF_R4_F2_R23 (const uint8_t value) {
			set_LF_R4_F2_R23(value);
			writeRegister(23);
		}

		void set_OUTBUF_RX_EN_F2_R23 (const uint8_t value) {
			setBits(&m_registers[23], 6, value, 0, 1);
		}
		void write_OUTBUF_RX_EN_F2_R23 (const uint8_t value) {
			set_OUTBUF_RX_EN_F2_R23(value);
			writeRegister(23);
		}

		void set_OUTBUF_TX_EN_F2_R23 (const uint8_t value) {
			setBits(&m_registers[23], 7, value, 0, 1);
		}
		void write_OUTBUF_TX_EN_F2_R23 (const uint8_t value) {
			set_OUTBUF_TX_EN_F2_R23(value);
			writeRegister(23);
		}

		void set_OUTBUF_RX_PWR_F2_R23 (const uint8_t value) {
			setBits(&m_registers[23], 8, value, 0, 5);
		}
		void write_OUTBUF_RX_PWR_F2_R23 (const uint8_t value) {
			set_OUTBUF_RX_PWR_F2_R23(value);
			writeRegister(23);
		}


		// Register 24
		void set_OUTBUF_TX_PWR_F2_R24 (const uint8_t value) {
			setBits(&m_registers[24], 0, value, 0, 5);
		}
		void write_OUTBUF_TX_PWR_F2_R24 (const uint8_t value) {
			set_OUTBUF_TX_PWR_F2_R24(value);
			writeRegister(24);
		}

		void set_EXTVCO_SEL_F2_R24 (const uint8_t value) {
			setBits(&m_registers[24], 5, value, 0, 1);
		}
		void write_EXTVCO_SEL_F2_R24 (const uint8_t value) {
			set_EXTVCO_SEL_F2_R24(value);
			writeRegister(24);
		}

		void set_EXTVCO_CHDIV_F2_R24 (const uint8_t value) {
			setBits(&m_registers[24], 6, value, 0, 4);
		}
		void write_EXTVCO_CHDIV_F2_R24 (const uint8_t value) {
			set_EXTVCO_CHDIV_F2_R24(value);
			writeRegister(24);
		}

		void set_FSK_EN_F2_R24 (const uint8_t value) {
			setBits(&m_registers[24], 10, value, 0, 1);
		}
		void write_FSK_EN_F2 (const uint8_t value) {
			set_FSK_EN_F2_R24(value);
			writeRegister(24);
		}

		// Register 33
		void set_FSK_DEV_SPI_FAST_R33 (const uint16_t value) {
			setBits(&m_registers[33], 0, value, 0, 16);
		}
		void write_FSK_DEV_SPI_FAST_R33 (const uint16_t value) {
			set_FSK_DEV_SPI_FAST_R33(value);
			writeRegister(33);
		}

		// Register 34
		void set_FSK_MODE_SEL1_R34 (const uint8_t value) {
			setBits(&m_registers[34], 0, value, 0, 1);
		}
		void write_FSK_MODE_SEL1_R34 (const uint8_t value) {
			set_FSK_MODE_SEL1_R34(value);
			writeRegister(34);
		}

		void set_FSK_MODE_SEL0_R34 (const uint8_t value) {
			setBits(&m_registers[34], 1, value, 0, 1);
		}
		void write_FSK_MODE_SEL0_R34 (const uint8_t value) {
			set_FSK_MODE_SEL0_R34(value);
			writeRegister(34);
		}

		void set_FSK_DEV_SEL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 2, value, 0, 3);
		}
		void write_FSK_DEV_SEL_R34 (const uint8_t value) {
			set_FSK_DEV_SEL_R34(value);
			writeRegister(34);
		}

		void set_FSK_LEVEL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 5, value, 0, 2);
		}
		void write_FSK_LEVEL_R34 (const uint8_t value) {
			set_FSK_LEVEL_R34(value);
			writeRegister(34);
		}

		void set_FSK_I2S_CLK_POL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 7, value, 0, 1);
		}
		void write_FSK_I2S_CLK_POL_R34 (const uint8_t value) {
			set_FSK_I2S_CLK_POL_R34(value);
			writeRegister(34);
		}

		void set_FSK_I2S_FS_POL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 8, value, 0, 1);
		}
		void write_FSK_I2S_FS_POL_R34 (const uint8_t value) {
			set_FSK_I2S_FS_POL_R34(value);
			writeRegister(34);
		}

		void set_XTAL_EN_R34 (const uint8_t value) {
			setBits(&m_registers[34], 10, value, 0, 1);
		}
		void write_XTAL_EN_R34 (const uint8_t value) {
			set_XTAL_EN_R34(value);
			writeRegister(34);
		}

		void set_XTAL_PWRCTRL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 11, value, 0, 3);
		}
		void write_XTAL_PWRCTRL_R34 (const uint8_t value) {
			set_XTAL_PWRCTRL_R34(value);
			writeRegister(34);
		}

		void set_IPBUF_SE_DIFF_SEL_R34 (const uint8_t value) {
			setBits(&m_registers[34], 14, value, 0, 1);
		}
		void write_IPBUF_SE_DIFF_SEL (const uint8_t value) {
			set_IPBUF_SE_DIFF_SEL_R34(value);
			writeRegister(34);
		}

		void set_IPBUFDIFF_TERM_R34 (const uint8_t value) {
			setBits(&m_registers[34], 15, value, 0, 1);
		}
		void write_IPBUFDIFF_TERM_R34 (const uint8_t value) {
			set_IPBUFDIFF_TERM_R34(value);
			writeRegister(34);
		}

		// Register 35
		void set_OUTBUF_RX_TYPE_R35 (const uint8_t value) {
			setBits(&m_registers[35], 0, value, 0, 1);
		}
		void write_OUTBUF_RX_TYPE_R35 (const uint8_t value) {
			set_OUTBUF_RX_TYPE_R35(value);
			writeRegister(35);
		}

		void set_OUTBUF_TX_TYPE_R35 (const uint8_t value) {
			setBits(&m_registers[35], 1, value, 0, 1);
		}
		void write_OUTBUF_TX_TYPE_R35 (const uint8_t value) {
			set_OUTBUF_TX_TYPE_R35(value);
			writeRegister(35);
		}

		void set_OUTBUF_AUTOMUTE_R35 (const uint8_t value) {
			setBits(&m_registers[35], 2, value, 0, 1);
		}
		void write_OUTBUF_AUTOMUTE_R35 (const uint8_t value) {
			set_OUTBUF_AUTOMUTE_R35(value);
			writeRegister(35);
		}

		void set_MULT_WAIT_R35 (const uint16_t value) {
			setBits(&m_registers[35], 3, value, 0, 11);
		}
		void write_MULT_WAIT_R35 (const uint16_t value) {
			set_MULT_WAIT_R35(value);
			writeRegister(35);
		}

		// Register 39
		void set_LD_EN_R39 (const uint8_t value) {
			setBits(&m_registers[39], 0, value, 0, 1);
		}
		void write_LD_EN (const uint8_t value) {
			set_LD_EN_R39(value);
			writeRegister(39);
		}

		void set_SDO_LD_SEL_R39 (const uint8_t value) {
			setBits(&m_registers[39], 3, value, 0, 1);
		}
		void write_SDO_LD_SEL_R39 (const uint8_t value) {
			set_SDO_LD_SEL_R39(value);
			writeRegister(39);
		}

		// Register 40
		void set_CP_GAIN_R40 (const uint8_t value) {
			setBits(&m_registers[40], 6, value, 0, 2);
		}
		void write_CP_GAIN_R40 (const uint8_t value) {
			set_CP_GAIN_R40(value);
			writeRegister(40);
		}

		void set_CP_IUP_R40 (const uint8_t value) {
			setBits(&m_registers[40], 8, value, 0, 5);
		}
		void write_CP_IUP_R40 (const uint8_t value) {
			set_CP_IUP_R40(value);
			writeRegister(40);
		}

		// Register 41
		void set_CP_IDN_R41 (const uint8_t value) {
			setBits(&m_registers[41], 0, value, 0, 5);
		}
		void write_CP_IDN_R41 (const uint8_t value) {
			set_CP_IDN_R41(value);
			writeRegister(41);
		}

		void set_EXTVCO_CP_GAIN_R41 (const uint8_t value) {
			setBits(&m_registers[41], 5, value, 0, 1);
		}
		void write_EXTVCO_CP_GAIN_R41 (const uint8_t value) {
			set_EXTVCO_CP_GAIN_R41(value);
			writeRegister(41);
		}
	};

} /* namespace Lmx2571 */
