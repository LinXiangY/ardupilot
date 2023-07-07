#pragma once

#include "AP_Inclination_Backend.h"

class AP_Inclination_Backend_Serial : public AP_Inclination_Backend
{
public:
    // constructor
    AP_Inclination_Backend_Serial(Inclination::Inclination_State &_state,
                                  AP_Inclination_Params &_params);

    void init_serial(uint8_t serial_instance) override;
    // static detection function
    static bool detect(uint8_t serial_instance);

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const
    {
        return 0;
    }
    virtual uint16_t tx_bufsize() const
    {
        return 0;
    }

    AP_HAL::UARTDriver *uart = nullptr;

    // update state; not all backends call this!
    virtual void update(enum InstallLocation location) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    //virtual bool get_reading(float &reading_roll_deg, float &reading_yaw_deg) = 0;
    virtual bool get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, InstallLocation loc) = 0;

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const
    {
        return 200;
    }
};
