#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AE_SlewingEncoder.h"
#include "AE_SlewingEncoder_Backend.h"

extern const AP_HAL::HAL& hal;

// base class constructor.
AE_SlewingEncoder_Backend::AE_SlewingEncoder_Backend(AE_SlewingEncoder::SlewingEncoder_State &_state):
        state(_state)
{

}

AE_SlewingEncoder::SlewingEncoder_Status AE_SlewingEncoder_Backend::status() const {
    if (type() == AE_SlewingEncoder::SlewingEncoder_Type::SlewingEncoder_TYPE_NONE) {
        // turned off at runtime?
        return AE_SlewingEncoder::SlewingEncoder_Status::NotConnected;
    }
    return state.status;
}

    // true if sensor is returning data
bool AE_SlewingEncoder_Backend::has_data() const {
        return ((state.status != AE_SlewingEncoder::NotConnected) &&
                (state.status != AE_SlewingEncoder::NoData));
    }

// update status based on slewing encoder measurement
void AE_SlewingEncoder_Backend::update_status()
{
    uint16_t full_count = state.total_turns_count * state.max_single_turn_count + state.single_turn_count;
    uint16_t max_full_count = 64 * state.max_single_turn_count + state.single_turn_count;
    // check measured data
    if (full_count > max_full_count) {
        set_status(AE_SlewingEncoder::SlewingEncoder_Status::OutOfTotalCountRangeHigh);
    } else if (full_count < 1) {
        set_status(AE_SlewingEncoder::SlewingEncoder_Status::OutOfTotalCountRangeLow);
    } else {
        set_status(AE_SlewingEncoder::SlewingEncoder_Status::Good);
    }
}

// set status and update valid count
void AE_SlewingEncoder_Backend::set_status(AE_SlewingEncoder::SlewingEncoder_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == AE_SlewingEncoder::SlewingEncoder_Status::Good) {
        if (state.encoder_valid_count < 10) {
            state.encoder_valid_count++;
        }
    } else {
        state.encoder_valid_count = 0;
    }
}
