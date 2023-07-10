#include "AP_Inclination_HDA436T_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define HDA436T_FRAME_HEADER 0x03
#define HDA436T_FRAME_LENGTH 19
#define HDA436T_DATA_LENGTH 0x0E // length of Data for Byte of HDA436T_Serial
#define ROLL_YAW_OFFSET 180000
#define PITCH_OFFSET 90000
#define INCLINATION_ROLL_MAX_DEGREE 180
#define INCLINATION_PITCH_MAX_DEGREE 90
#define INCLINATION_YAW_MAX_DEGREE 180

// format of serial packets received from inclination sensor HDA436T
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x03
// byte 1               Frame header    0x03
// byte 2               DATA_LENGTH     roll pitch yaw temperature these 4 data totle length of byte, default is 0x0E
// byte 3               ROLL1_L         roll raw data 1 low 8 bits
// byte 4               ROLL1_H         roll raw data 1 high 8 bits
// byte 5               ROLL2_L         roll raw data 2 low 8 bits
// bute 6               ROLL2_H         roll raw data 2 high 8 bits
// byte 7               PITCH1_L        pitch raw data 1 low 8 bits
// bute 8               PITCH1_H        pitch raw data 1 high 8 bits
// byte 9               PITCH2_L        pitch raw data 2 low 8 bits
// byte 10              PITCH2_H        pitch raw data 2 high 8 bits
// byte 11              YAW1_L          yaw raw data 1 low 8 bits
// byte 12              YAW1_H          yaw raw data 1 high 8 bits
// byte 13              YAW2_L          yaw raw data 1 low 8 bits
// byte 14              YAW2_H          yaw raw data 1 high 8 bits
// byte 15              TEMPERATURE1    high 4 bits is positive or negtive?  0:P   1:N ;  low 4 bits is tens bit of tempterature
// byte 16              TEMPERATURE2    high 4 bits is ones bit of tempterature;   low 4 bits is decimal bit of tempterature
// byte 17              Checksum        high 8 bits of Checksum byte, sum of bytes 0 to bytes 16
// byte 18              Checksum        low  8 bits of Checksum byte, sum of bytes 0 to bytes 16


// read - return last value measured by sensor
bool AP_Inclination_HDA436T_Serial::get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, InstallLocation location)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_roll_deg = 0;
    float sum_pitch_deg = 0;
    float sum_yaw_deg = 0;
    uint16_t count = 0;
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
            if (c == HDA436T_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == HDA436T_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 19 items try to decode it
            if (linebuf_len == HDA436T_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[18]<<8) | linebuf[17];
                if (crc == calc_crc_modbus(linebuf, 17)) {
                    // calculate roll angle
                    int32_t roll_raw = ((uint32_t)linebuf[6] << 24) | ((uint32_t)linebuf[5] << 16) | ((uint16_t)linebuf[4] << 8) | linebuf[3];
                    int32_t pitch_raw = ((uint32_t)linebuf[10] << 24) | ((uint32_t)linebuf[9] << 16) | ((uint16_t)linebuf[8] << 8) | linebuf[7];
                    int32_t yaw_raw = ((uint32_t)linebuf[14] << 24) | ((uint32_t)linebuf[13] << 16) | ((uint16_t)linebuf[12] << 8) | linebuf[11];
                    float roll = (float)((roll_raw - ROLL_YAW_OFFSET)*0.001);
                    float pitch = (float)((pitch_raw - PITCH_OFFSET)*0.001);
                    float yaw = (float)((yaw_raw - ROLL_YAW_OFFSET)*0.001);
                    if (roll > INCLINATION_ROLL_MAX_DEGREE || pitch > INCLINATION_PITCH_MAX_DEGREE || yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if ((roll < - INCLINATION_ROLL_MAX_DEGREE) || (pitch < - INCLINATION_PITCH_MAX_DEGREE) || (yaw < - INCLINATION_YAW_MAX_DEGREE)) {
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        //hal.console->printf("555inclination tilt sensor uart: %f\t, %lu\t,  %lu\r\n", roll, roll_raw, (roll_raw - ROLL_YAW_OFFSET));
                        sum_roll_deg += roll;
                        sum_pitch_deg += pitch;
                        sum_yaw_deg += yaw;
                        count++;
                    }
                }

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        switch (location) {
        case InstallLocation::Boom:     //reading_roll/pitch/yaw_deg.x denote boom angle
            reading_roll_deg.x  = sum_roll_deg  / count;
            reading_pitch_deg.x = sum_pitch_deg / count;
            reading_yaw_deg.x   = sum_yaw_deg   / count;
            break;
        case InstallLocation::Forearm:     //reading_roll/pitch/yaw_deg.y denote forearm angle
            reading_roll_deg.y  = sum_roll_deg  / count;
            reading_pitch_deg.y = sum_pitch_deg / count;
            reading_yaw_deg.y   = sum_yaw_deg   / count;
            break;
        case InstallLocation::Bucket:     //reading_roll/pitch/yaw_deg.z denote bucket angle
            reading_roll_deg.z  = sum_roll_deg  / count;
            reading_pitch_deg.z = sum_pitch_deg / count;
            reading_yaw_deg.z   = sum_yaw_deg   / count;
            break;

        default:
            reading_roll_deg.x  = sum_roll_deg  / count; // LOCATION_NONE we treat it as boom angle
            reading_pitch_deg.x = sum_pitch_deg / count;
            reading_yaw_deg.x   = sum_yaw_deg   / count;
            break;
        }

        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        switch (location) {
        case InstallLocation::Boom:     //reading_roll/pitch/yaw_deg.x denote boom angle
            reading_roll_deg.x  = INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.x = INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.x   = INCLINATION_YAW_MAX_DEGREE;
            break;
        case InstallLocation::Forearm:     //reading_roll/pitch/yaw_deg.y denote forearm angle
            reading_roll_deg.y  = INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.y = INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.y   = INCLINATION_YAW_MAX_DEGREE;
            break;
        case InstallLocation::Bucket:     //reading_roll/pitch/yaw_deg.z denote bucket angle
            reading_roll_deg.z  = INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.z = INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.z   = INCLINATION_YAW_MAX_DEGREE;
            break;

        default:
            reading_roll_deg.x  = INCLINATION_ROLL_MAX_DEGREE; // LOCATION_NONE we treat it as boom angle
            reading_pitch_deg.x = INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.x   = INCLINATION_YAW_MAX_DEGREE;
            break;
        }
        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        switch (location) {
        case InstallLocation::Boom:     //reading_roll/pitch/yaw_deg.x denote boom angle
            reading_roll_deg.x  = -INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.x = -INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.x   = -INCLINATION_YAW_MAX_DEGREE;
            break;
        case InstallLocation::Forearm:     //reading_roll/pitch/yaw_deg.y denote forearm angle
            reading_roll_deg.y  = -INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.y = -INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.y   = -INCLINATION_YAW_MAX_DEGREE;
            break;
        case InstallLocation::Bucket:     //reading_roll/pitch/yaw_deg.z denote bucket angle
            reading_roll_deg.z  = -INCLINATION_ROLL_MAX_DEGREE;
            reading_pitch_deg.z = -INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.z   = -INCLINATION_YAW_MAX_DEGREE;
            break;

        default:
            reading_roll_deg.x  = -INCLINATION_ROLL_MAX_DEGREE; // LOCATION_NONE we treat it as boom angle
            reading_pitch_deg.x = -INCLINATION_PITCH_MAX_DEGREE;
            reading_yaw_deg.x   = -INCLINATION_YAW_MAX_DEGREE;
            break;
        }
        return true;
    }

    // no readings so return false
    return false;
}

// get temperature reading
// We write the function of reading temperature here in advance for later use.
// if use we should make a private var _temp, and get the _temp at get_reading().
bool AP_Inclination_HDA436T_Serial::get_temp_C_from_loc(enum InstallLocation location, float &temp) const
{
    // if use, need to do

    return true;
}
