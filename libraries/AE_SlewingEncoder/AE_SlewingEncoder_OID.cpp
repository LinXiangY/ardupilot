#include <AP_HAL/AP_HAL.h>
#include "AE_SlewingEncoder_OID.h"

extern const AP_HAL::HAL& hal;

#define OID_AMMETER_FRAME_HEADER    0x01
#define OID_AMMETER_FRAME_HEADER2   0x03
#define OID_AMMETER_FRAME_LENGTH    11

// format of serial packets received from slewing encoder and ammeter sensors at 2 in 1 mode
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header            0x01
// byte 1               Frame header            0x03
// byte 2               DATA_LENGTH             encoder and ammeter these 2 data totle length of byte, default is 0x06
// byte 3               Encoder_full_count_L1   slewing spin count raw data high 8 bit
// byte 4               Encoder_full_count_L2   slewing spin count raw data low 8 bit
// byte 5               Encoder_full_count_L3   slewing raw data high 8 bit
// byte 6               Encoder_full_count_L4   slewing raw data low 8 bit
// byte 7               Ammeter_H               ammeter raw data high 8 bit
// byte 8               Ammeter_L               ammeter raw data low 8 bit
// byte 9               Checksum                low 8 bits of Checksum byte, sum of bytes 0 to bytes 16
// byte 10              Checksum                high 8 bits of Checksum byte, sum of bytes 0 to bytes 16

// 2 in 1 module return two datas: ammeter in amperes, and full_count = reading_total_count * state.max_single_turn_count + reading_sigle_count
bool AE_SlewingEncoder_OID::get_slewing_encoder_reading(uint32_t &reading_full_count, uint16_t &reading_ammeter_amperes)
{
    if (uart == nullptr) {
        return false;
    }

    uint32_t sum_full_turns = 0;
    uint16_t sum_ammeter_amperes = 0;
    uint16_t count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;
    

    // read any available lines from the 2 in 1 module
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x01, add to buffer
        if (linebuf_len == 0) {
            if (c == OID_AMMETER_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == OID_AMMETER_FRAME_HEADER2) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;

            // if buffer now has 11 items try to decode it
            if (linebuf_len == OID_AMMETER_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[OID_AMMETER_FRAME_LENGTH-1]<<8) | linebuf[OID_AMMETER_FRAME_LENGTH-2];
                if (crc == calc_crc_modbus(linebuf, OID_AMMETER_FRAME_LENGTH-2)) {
                    // receive the full turn counts
                    uint32_t full_turn_count = ((uint32_t)linebuf[3] << 24) | ((uint32_t)linebuf[4] << 16) | ((uint16_t)linebuf[5] << 8) | linebuf[6];                    
                    // get the current data
                    uint16_t ammeter_amperes = ((uint16_t)linebuf[7] << 8) | linebuf[8];
                    // data validity verification, full_turn_count = 64*4096
                    if (full_turn_count > state.max_single_turn_count*state.max_total_turns_count || ammeter_amperes > 3000) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if (full_turn_count < 1)  {
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add turns to sum
                        sum_full_turns += full_turn_count;
                        sum_ammeter_amperes += ammeter_amperes;
                        count++;
                    }
                }

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average count of readings
        reading_full_count  = sum_full_turns / count;
        reading_ammeter_amperes = sum_ammeter_amperes / count;
// hal.console->printf("\r\n---5---AE_SlewingEncoder_OID::get_reading: reading_full_count = %lu\t, reading_total_count = %u\t,reading_sigle_count = %u\t,reading_ammeter_amperes = %u---\r\n", reading_full_count, reading_total_count, reading_sigle_count, reading_ammeter_amperes);

        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if only out of positive readings, should store the angle difference between base and arm of the excavator.
        // we should test the absolute encoder if exceed the positive max range, to see what will happen.
        // need to do the store work


        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if only out of negtive range readings, should store the angle difference between base and arm of the excavator.
        // we should test the absolute encoder if exceed the negtive min range, to see what will happen.
        // need to do the store work


        return true;
    }

    // no readings so return false
    return false;
}


