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

#include "AP_Inclination_3HDA436Ts_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

#define HDA436T_HDR 0x03   // Header Byte from USD1_Serial
#define HDA436T_DATA_LENGTH 0x18 // length of Data for Byte of HDA436T_Serial at 3 in 1 mode

extern const AP_HAL::HAL& hal;

#define INCLI_FRAME_HEADER 0x03
#define INCLI_FRAME_LENGTH 45
#define ROLL_YAW_OFFSET 180000
#define PITCH_OFFSET 90000
#define INCLINATION_ROLL_MAX_DEGREE 180
#define INCLINATION_PITCH_MAX_DEGREE 180
#define INCLINATION_YAW_MAX_DEGREE 180
#define ENCODER_DEG_MAX 360
#define ENCODER_DEG_MIN 0

// format of serial packets received from 3 inclination HDA436T sensors at 3 in 1 mode
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header        0x03
// byte 1               Frame header        0x03
// byte 2               DATA_LENGTH         roll pitch yaw temperature these 4 data totle length of byte, default is 0x18
// byte 3               Boom_ROLL1_L        roll raw data 1 low 8 bits
// byte 4               Boom_ROLL1_H        roll raw data 1 high 8 bits
// byte 5               Boom_ROLL2_L        roll raw data 2 low 8 bits
// bute 6               Boom_ROLL2_H        roll raw data 2 high 8 bits
// byte 7               Boom_PITCH1_L       pitch raw data 1 low 8 bits
// byte 8               Boom_PITCH1_H       pitch raw data 1 high 8 bits
// byte 9               Boom_PITCH2_L       pitch raw data 2 low 8 bits
// byte 10              Boom_PITCH2_H       pitch raw data 2 high 8 bits
// byte 11              Boom_YAW1_L         yaw raw data 1 low 8 bits
// bute 12              Boom_YAW1_H         yaw raw data 1 high 8 bits
// byte 13              Boom_YAW2_L         yaw raw data 1 low 8 bits
// byte 14              Boom_YAW2_H         yaw raw data 1 high 8 bits
// byte 15              Forearm_ROLL1_L     roll raw data 1 low 8 bits
// byte 16              Forearm_ROLL1_H     roll raw data 1 high 8 bits
// byte 17              Forearm_ROLL2_L     roll raw data 2 low 8 bits
// byte 18              Forearm_ROLL2_H     roll raw data 2 high 8 bits
// byte 19              Forearm_PITCH1_L    pitch raw data 1 low 8 bits
// byte 20              Forearm_PITCH1_H    pitch raw data 1 high 8 bits
// byte 21              Forearm_PITCH2_L    pitch raw data 2 low 8 bits
// byte 22              Forearm_PITCH2_H    pitch raw data 2 high 8 bits
// byte 23              Forearm_YAW1_L      yaw raw data 1 low 8 bits
// byte 24              Forearm_YAW1_H      yaw raw data 1 high 8 bits
// byte 25              Forearm_YAW2_L      yaw raw data 1 low 8 bits
// byte 26              Forearm_YAW2_H      yaw raw data 1 high 8 bits
// byte 27              Bucket_ROLL1_L      roll raw data 1 low 8 bits
// byte 28              Bucket_ROLL1_H      roll raw data 1 high 8 bits
// byte 29              Bucket_ROLL2_L      roll raw data 2 low 8 bits
// byte 30              Bucket_ROLL2_H      roll raw data 2 high 8 bits
// byte 31              Bucket_PITCH1_L      roll raw data 1 low 8 bits
// byte 32              Bucket_PITCH1_H      roll raw data 1 high 8 bits
// byte 33              Bucket_PITCH2_L      roll raw data 2 high 8 bits
// byte 34              Bucket_PITCH2_H      roll raw data 2 high 8 bits
// byte 35              Bucket_YAW1_L       yaw raw data 1 low 8 bits
// byte 36              Bucket_YAW1_H       yaw raw data 1 high 8 bits
// byte 37              Bucket_YAW2_L       yaw raw data 1 low 8 bits
// byte 38              Bucket_YAW2_H       yaw raw data 1 high 8 bits
// byte 39              Encoder_spin_count_L    slewing spin count raw data low 8 bit
// byte 40              Encoder_spin_count_H    slewing spin count raw data high 8 bit
// byte 41              Encoder_deg_L       slewing raw data low 8 bit
// byte 42              Encoder_deg_H       slewing raw data high 8 bit
// byte 43              Checksum            low 8 bits of Checksum byte, sum of bytes 0 to bytes 16
// byte 44              Checksum            high 8 bits of Checksum byte, sum of bytes 0 to bytes 16

// read - return last value measured by sensor
// reading_roll_deg.x is the boom roll angle, reading_roll_deg.y is the forearm roll angle, reading_roll_deg.z is the bucket roll angle.
// reading_yaw_deg.x is the boom yaw angle, reading_yaw_deg.y is the forearm yaw angle, reading_yaw_deg.z is the bucket yaw angle,
bool AP_Inclination_3HDA436Ts_Serial::get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, float &reading_slewing_deg, InstallLocation location)
{
    if (uart == nullptr) {
        return false;
    }

    float boom_sum_roll_deg = 0;
    float boom_sum_pitch_deg = 0;
    float boom_sum_yaw_deg = 0;
    float forearm_sum_roll_deg = 0;
    float forearm_sum_pitch_deg = 0;
    float forearm_sum_yaw_deg = 0;
    float bucket_sum_roll_deg = 0;
    float bucket_sum_pitch_deg = 0;
    float bucket_sum_yaw_deg = 0;
    float slewing_sum_deg = 0; 
    uint16_t boom_count = 0;
    uint16_t forearm_count = 0;
    uint16_t bucket_count = 0;
    uint16_t slewing_count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;



    // read any available lines from the inclination
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x03, add to buffer
        if (linebuf_len == 0) {
            if (c == INCLI_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == INCLI_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 19 items try to decode it
            if (linebuf_len == INCLI_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[INCLI_FRAME_LENGTH-1]<<8) | linebuf[INCLI_FRAME_LENGTH-2];
                if (crc == calc_crc_modbus(linebuf, INCLI_FRAME_LENGTH-2)) {
                    // calculate boom roll and yaw angle
                    int32_t boom_roll_raw = ((uint32_t)linebuf[6] << 24) | ((uint32_t)linebuf[5] << 16) | ((uint16_t)linebuf[4] << 8) | linebuf[3];
                    int32_t boom_pitch_raw = ((uint32_t)linebuf[10] << 24) | ((uint32_t)linebuf[9] << 16) | ((uint16_t)linebuf[8] << 8) | linebuf[7];
                    int32_t boom_yaw_raw = ((uint32_t)linebuf[14] << 24) | ((uint32_t)linebuf[13] << 16) | ((uint16_t)linebuf[12] << 8) | linebuf[11];
                    float boom_roll = (float)((boom_roll_raw - ROLL_YAW_OFFSET)*0.001);
                    float boom_pitch = (float)((boom_pitch_raw - ROLL_YAW_OFFSET)*0.001);
                    float boom_yaw = (float)((boom_yaw_raw - ROLL_YAW_OFFSET)*0.001);
                    if (boom_roll > INCLINATION_ROLL_MAX_DEGREE || boom_pitch > INCLINATION_PITCH_MAX_DEGREE || boom_yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if ((boom_roll < - INCLINATION_ROLL_MAX_DEGREE) || (boom_pitch < - INCLINATION_PITCH_MAX_DEGREE) || (boom_yaw < - INCLINATION_YAW_MAX_DEGREE)) {
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        boom_sum_roll_deg += boom_roll;
                        boom_sum_pitch_deg += boom_pitch;
                        boom_sum_yaw_deg += boom_yaw;
                        boom_count++;
                    }

                    // calculate forearm roll and yaw angle
                    int32_t forearm_roll_raw = ((uint32_t)linebuf[18] << 24) | ((uint32_t)linebuf[17] << 16) | ((uint16_t)linebuf[16] << 8) | linebuf[15];
                    int32_t forearm_pitch_raw = ((uint32_t)linebuf[22] << 24) | ((uint32_t)linebuf[21] << 16) | ((uint16_t)linebuf[20] << 8) | linebuf[19];
                    int32_t forearm_yaw_raw = ((uint32_t)linebuf[26] << 24) | ((uint32_t)linebuf[25] << 16) | ((uint16_t)linebuf[24] << 8) | linebuf[23];
                    float forearm_roll = (float)((forearm_roll_raw - ROLL_YAW_OFFSET)*0.001);
                    float forearm_pitch = (float)((forearm_pitch_raw - ROLL_YAW_OFFSET)*0.001);
                    float forearm_yaw = (float)((forearm_yaw_raw - ROLL_YAW_OFFSET)*0.001);
                    if (forearm_roll > INCLINATION_ROLL_MAX_DEGREE || forearm_pitch > INCLINATION_PITCH_MAX_DEGREE || forearm_yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if ((forearm_roll < - INCLINATION_ROLL_MAX_DEGREE) || (forearm_pitch <  - INCLINATION_PITCH_MAX_DEGREE) || (forearm_yaw < - INCLINATION_YAW_MAX_DEGREE)) {
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        forearm_sum_roll_deg += forearm_roll;
                        forearm_sum_pitch_deg += forearm_pitch;
                        forearm_sum_yaw_deg += forearm_yaw;
                        forearm_count++;
                    }

                    // calculate bucket roll and yaw angle
                    int32_t bucket_roll_raw = ((uint32_t)linebuf[30] << 24) | ((uint32_t)linebuf[29] << 16) | ((uint16_t)linebuf[28] << 8) | linebuf[27];
                    int32_t bucket_pitch_raw = ((uint32_t)linebuf[34] << 24) | ((uint32_t)linebuf[33] << 16) | ((uint16_t)linebuf[32] << 8) | linebuf[31];
                    int32_t bucket_yaw_raw = ((uint32_t)linebuf[38] << 24) | ((uint32_t)linebuf[37] << 16) | ((uint16_t)linebuf[36] << 8) | linebuf[35];
                    float bucket_roll = (float)((bucket_roll_raw - ROLL_YAW_OFFSET)*0.001);
                    float bucket_pitch = (float)((bucket_pitch_raw - ROLL_YAW_OFFSET)*0.001);
                    float bucket_yaw = (float)((bucket_yaw_raw - ROLL_YAW_OFFSET)*0.001);
                    if (bucket_roll > INCLINATION_ROLL_MAX_DEGREE || bucket_pitch > INCLINATION_PITCH_MAX_DEGREE || bucket_yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if ((bucket_roll < - INCLINATION_ROLL_MAX_DEGREE) || (bucket_pitch <  - INCLINATION_PITCH_MAX_DEGREE) || (bucket_yaw < - INCLINATION_YAW_MAX_DEGREE)) {
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        bucket_sum_roll_deg += bucket_roll;
                        bucket_sum_pitch_deg += bucket_pitch;
                        bucket_sum_yaw_deg += bucket_yaw;
                        bucket_count++;
                    }

                    // calculate slewing deg
                    int32_t slewing_deg_raw = (uint16_t)linebuf[42] << 8 | linebuf[41];
                    float slewing_deg = float(slewing_deg_raw)*360/65535;
                    if(slewing_deg > ENCODER_DEG_MAX){
                        count_out_of_positive_range++;
                    } else if (slewing_deg < ENCODER_DEG_MIN){
                        count_out_of_negtive_range++;
                    } else {
                        slewing_sum_deg += slewing_deg;
                        slewing_count++;
                    }
                }

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (boom_count > 0 && forearm_count > 0 && bucket_count > 0 && slewing_count >0) {
        // return average distance of readings
        reading_roll_deg.x = boom_sum_roll_deg / boom_count;
        reading_pitch_deg.x = boom_sum_pitch_deg / boom_count;
        reading_yaw_deg.x = boom_sum_yaw_deg / boom_count;

        reading_roll_deg.y = forearm_sum_roll_deg / forearm_count;
        reading_pitch_deg.y = forearm_sum_pitch_deg / forearm_count;
        reading_yaw_deg.y = forearm_sum_yaw_deg / forearm_count;

        reading_roll_deg.z = bucket_sum_roll_deg / bucket_count;
        reading_pitch_deg.z = bucket_sum_pitch_deg / bucket_count;
        reading_yaw_deg.z = bucket_sum_yaw_deg / bucket_count;

        reading_slewing_deg = slewing_sum_deg / slewing_count;
        //hal.console->printf("\r\nAP_Inclination_3HDA436Ts_Serial::get_reading: boom_roll = %f\t,forearm_roll = %f\t,bucket_roll = %f\r\n", reading_roll_deg.x, reading_roll_deg.y, reading_roll_deg.z);

        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        reading_roll_deg.x = INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.x = INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.x = INCLINATION_YAW_MAX_DEGREE;

        reading_roll_deg.y = INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.y = INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.y = INCLINATION_YAW_MAX_DEGREE;

        reading_roll_deg.z = INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.z = INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.z = INCLINATION_YAW_MAX_DEGREE;

        reading_slewing_deg = ENCODER_DEG_MAX;

        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        reading_roll_deg.x = -INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.x = -INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.x = -INCLINATION_YAW_MAX_DEGREE;

        reading_roll_deg.y = -INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.y = -INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.y = -INCLINATION_YAW_MAX_DEGREE;

        reading_roll_deg.z = -INCLINATION_ROLL_MAX_DEGREE;
        reading_pitch_deg.z = -INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg.z = -INCLINATION_YAW_MAX_DEGREE;

        reading_slewing_deg = ENCODER_DEG_MIN;

        return true;
    }

    // no readings so return false
    return false;
}

// if we set the serialx_protocol=50, serialx_BAUD=115 and incli_type=Type::three_HDA436Ts_Serial,
// then we use only one uart for 3 backend drivers update.
void AP_Inclination_3HDA436Ts_Serial::init_serial(uint8_t serial_instance)
{
    if (serial_instance > 0) {
        return;
    }

    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Inclination, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}
