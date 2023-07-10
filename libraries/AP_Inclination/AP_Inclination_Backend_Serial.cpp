#include <AP_HAL/AP_HAL.h>
#include "AP_Inclination_Backend_Serial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL &hal;

/*
   The constructor also initialises the inclination. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the inclination
*/
AP_Inclination_Backend_Serial::AP_Inclination_Backend_Serial(
    Inclination::Inclination_State &_state,
    AP_Inclination_Params &_params) :
    AP_Inclination_Backend(_state, _params)
{

}

void AP_Inclination_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Inclination, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AP_Inclination_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Inclination, serial_instance);
}

/*
   detect if a Serial inclination is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_Inclination_Backend_Serial::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Inclination, serial_instance);
}

/*
   update the state of the sensor
*/
void AP_Inclination_Backend_Serial::update(enum InstallLocation loc)
{
    if (get_reading(state.roll_deg, state.pitch_deg, state.yaw_deg, loc)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status(loc);
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        set_status(Inclination::Status::NoData);
    }
}
