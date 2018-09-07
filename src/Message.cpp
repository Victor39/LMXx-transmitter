#include <string.h>
#include "Message.h"

namespace lmx2571 {
	const uint16_t Message::m_size = 3;

	void Message::read (const uint8_t * pBuffer) {
		rw() = ((pBuffer[0] >> 7) & 0x01);
		address() = (pBuffer[0] & 0x7F);
		data() = (pBuffer[1] << 8 | pBuffer[2]);
	}

	void Message::write (uint8_t * pBuffer) const {
		memset(pBuffer, 0x00, size());
		pBuffer[0] |= (rw() << 7) & 0x01;
		pBuffer[0] |= (data() & 0x7F);
		pBuffer[1] |= (data() >> 8 & 0xFF);
		pBuffer[2] |= (data() & 0xFF);
	}


} /* namespace lmx2571 */
