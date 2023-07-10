#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AE_SlewingEncoder_Backend_Serial.h"

extern const AP_HAL::HAL& hal;

// base class constructor.
AE_SlewingEncoder_Backend_Serial::AE_SlewingEncoder_Backend_Serial(AE_SlewingEncoder::SlewingEncoder_State &_state) :
    AE_SlewingEncoder_Backend(_state)
{

}

void AE_SlewingEncoder_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SlewingEncoder, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AE_SlewingEncoder_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_SlewingEncoder, serial_instance);
}

/* 
   detect if an encoder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AE_SlewingEncoder_Backend_Serial::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_SlewingEncoder, serial_instance);
}


float AE_SlewingEncoder_Backend_Serial::calc_angle_diff_base2arm()
{
    uint32_t middle_count = state.max_single_turn_count*state.max_total_turns_count/2;
    float diff_deg = (float)((state.full_turn_count - middle_count) * 360 / state.turns_count_per_slewing_revolution);
    // return wrap_PI(radians(diff_deg));
    // return wrap_360_cd(diff_deg);
    return wrap_PI(radians(diff_deg));
    // return diff_deg;
}

/*
   update the state of the sensor
*/
void AE_SlewingEncoder_Backend_Serial::update(void)
{
    if (get_slewing_encoder_reading(state.full_turn_count, state.ammeter_amperes)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        // calculate total and single turns
        state.total_turns_count = state.full_turn_count/state.max_single_turn_count;
        state.single_turn_count = state.full_turn_count%state.max_single_turn_count;
        state.angle_deg_diff_base2arm = calc_angle_diff_base2arm();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        set_status(AE_SlewingEncoder::SlewingEncoder_Status::NoData);
    }
}

