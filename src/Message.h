#pragma once
#include <smartbfarray.h>

namespace Lmx2571 {

	class Message {
	public:
		BitField<1> rw;
		BitField<7> address;
		BitField<16> data;

	};

} /* namespace Lmx2571 */
