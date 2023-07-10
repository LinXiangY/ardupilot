#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AE_SlewingEncoder.h"

class AE_SlewingEncoder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AE_SlewingEncoder_Backend(AE_SlewingEncoder::SlewingEncoder_State &_state);

    // we declare a virtual destructor so that WheelEncoder drivers can
    // override with a custom destructor if need be
    virtual ~AE_SlewingEncoder_Backend(void) {}

    // update the state structure. All backends must implement this.
    virtual void update() = 0;

    virtual void init_serial(uint8_t serial_instance) {};

/////////////////////////////////////////////////
    virtual int16_t max_single_turn_count() const { return state.max_single_turn_count; }

    virtual int16_t max_total_turns_count() const { return state.max_total_turns_count; }

    enum AE_SlewingEncoder::Install_Location location() const { return (AE_SlewingEncoder::Install_Location)state.location.get(); }

    float get_angle_diff_base2arm() const { return state.angle_deg_diff_base2arm; }

    uint16_t single_turn_count() const { return state.single_turn_count; }
    
    uint16_t total_turns_count() const { return state.total_turns_count; }

    uint32_t full_turns_counts() const { return state.full_turn_count; }           

    AE_SlewingEncoder::SlewingEncoder_Status status() const;

    AE_SlewingEncoder::SlewingEncoder_Type type() const { return (AE_SlewingEncoder::SlewingEncoder_Type)state.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t range_valid_count() const { return state.encoder_valid_count; }

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:

    // update status based on encoder measurement
    void update_status();

    // set status and update valid_count
    void set_status(AE_SlewingEncoder::SlewingEncoder_Status status);

    AE_SlewingEncoder::SlewingEncoder_State &state;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

};
