#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>
#include "AE_SlewingEncoder_Backend.h"

class AE_SlewingEncoder_Backend_Serial : public AE_SlewingEncoder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AE_SlewingEncoder_Backend_Serial(AE_SlewingEncoder::SlewingEncoder_State &_state);

    void init_serial(uint8_t serial_instance) override;

    // static detection function
    static bool detect(uint8_t serial_instance);
  
protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }
    
    AP_HAL::UARTDriver *uart = nullptr;

    // update state; not all backends call this!
    virtual void update(void) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_slewing_encoder_reading(uint32_t &reading_full_count,
                                                uint16_t &reading_ammeter_amperes) = 0;

    // virtual bool get_slewing_encoder_reading(uint16_t &reading_total_count,
    //                                             uint16_t &reading_sigle_count,
    //                                             uint16_t &reading_ammeter_amperes) = 0;

    float calc_angle_diff_base2arm();

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }

};
