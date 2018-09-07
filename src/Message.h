#pragma once

#include <stdint.h>
#include <stdio.h>

namespace lmx2571 {

	class Message {
	public:

		Message () :
				m_rw(0), m_address(0), m_data(0) {
		}

		virtual ~Message () {
		}

		void read (const uint8_t * pBuffer);
		void write (uint8_t * pBuffer) const;

		std::size_t size () const {
			return m_size;
		}

		uint8_t& rw () {
			return m_rw;
		}
		const uint8_t& rw () const {
			return m_rw;
		}

		uint8_t& address () {
			return m_address;
		}
		const uint8_t& address () const {
			return m_address;
		}

		uint16_t& data () {
			return m_data;
		}
		const uint16_t& data () const {
			return m_data;
		}

	private:
		uint8_t m_rw;
		uint8_t m_address;
		uint16_t m_data;

		static const uint16_t m_size;

	};

} /* namespace lmx2571 */
