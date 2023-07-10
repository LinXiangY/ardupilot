/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AE_SlewingEncoder.h"
#include "AE_SlewingEncoder_Backend_Serial.h"


class AE_SlewingEncoder_OID : public AE_SlewingEncoder_Backend_Serial
{
public:
    // constructor
    // AE_SlewingEncoder_OID(AE_SlewingEncoder::SlewingEncoder_State &_state);

    using AE_SlewingEncoder_Backend_Serial::AE_SlewingEncoder_Backend_Serial;

     // void init_serial(uint8_t serial_instance) override;

private:

    bool get_slewing_encoder_reading(uint32_t &reading_full_count, uint16_t &reading_ammeter_amperes) override;

    uint32_t last_reading_ms;
    uint8_t linebuf[12];
    uint8_t linebuf_len;


};
