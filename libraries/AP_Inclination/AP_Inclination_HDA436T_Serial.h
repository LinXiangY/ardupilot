#pragma once

#include "AP_Inclination.h"
#include "AP_Inclination_Backend_Serial.h"

/*
This is the serial port data reading of a single inclination sensor.
In order to be compatible with the three-in-one inclination sensor,
we change its data structure, that is, roll_deg, pitch_deg, yaw_deg in Inclination_State are all changed to Vector3f,
roll_deg.x represents the angle of the boom,
roll_deg.y represents the angle of the forearm,
roll_deg.z represents the angle of the bucket
*/
class AP_Inclination_HDA436T_Serial : public AP_Inclination_Backend_Serial
{

public:

    using AP_Inclination_Backend_Serial::AP_Inclination_Backend_Serial;

private:

    // get a reading
    // bool get_reading(float &reading_roll_m, float &reading_yaw_m) override;
    // bool get_reading(Vector3f &reading_roll_deg, Vector3f &reading_yaw_deg, InstallLocation loc) override;
    bool get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, InstallLocation location) override;
    
    // get temperature reading in C.  returns true on success and populates temp argument
    bool get_temp_C_from_loc(enum InstallLocation location, float &temp) const override;

    uint8_t  linebuf[20];
    uint8_t  linebuf_len;
};
