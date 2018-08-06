#pragma once
#include <stdint.h>
#include <src/Message.h>

namespace Lmx2571 {

class Device {
public:
	static Device& getInstance () {
		static Device instance;
		return instance;
	}

	void setRegister (const uint8_t adrr, const uint16_t data);
	uint16_t getRegister (const uint8_t adrr);

private:
	SmartBfArray<Message> m_txMsg;
	SmartBfArray<Message> m_rxMsg;

	Device ();
	Device (Device const&);
	void operator= (Device const&);

};

} /* namespace Lmx2571 */
