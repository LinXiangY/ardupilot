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

#include "AP_Inclination.h"
#include "AP_Inclination_HDA436T_Serial.h"
#include "AP_Inclination_3HDA436Ts_Serial.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Inclination::var_info[] = {

    // @Group: 1_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 25, Inclination, AP_Inclination_Params),

    // @Group: 1_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, Inclination, backend_var_info[0]),

#if INCLINATION_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, Inclination, AP_Inclination_Params),

    // @Group: 2_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, Inclination, backend_var_info[1]),
#endif

#if INCLINATION_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, Inclination, AP_Inclination_Params),

    // @Group: 3_
    // @Path: AP_Inclination_Params.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, Inclination, backend_var_info[2]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *Inclination::backend_var_info[INCLINATION_MAX_INSTANCES];

Inclination::Inclination()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Inclination must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the Inclination class. We do detection of attached
  Inclinations here. For now we won't allow for hot-plugging of
  Inclinations.
 */
void Inclination::init(InstallLocation location_default)
{
    if (init_done) {
        // init called a 2nd time
        return;
    }
    init_done = true;

    // set orientation defaults
    for (uint8_t i=0; i<INCLINATION_MAX_INSTANCES; i++) {
        params[i].location.set_default(location_default + i+1);
    }

    for (uint8_t i=0, serial_instance = 0; i<INCLINATION_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update Inclination state for all instances. This should be called at
  around 10Hz by main loop
 */
void Inclination::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a Inclination at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }

            drivers[i]->update((InstallLocation)(i+1));
        }
    }

    //if AP_Inclination_3HDA436Ts_Serial is used, then update the other 2 drivers state by hand.
    if ((Type)params[0].type.get() == Type::three_HDA436Ts_Serial) {
        state[1] = state[0];
        state[2] = state[0];
    }

#if HAL_LOGGING_ENABLED
    Log_ICLI();
#endif
}

bool Inclination::_add_backend(AP_Inclination_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= INCLINATION_MAX_INSTANCES) {
        AP_HAL::panic("Too many INCLINATION backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);
    return true;
}

/*
  detect if an instance of a inclination is connected.
 */
void Inclination::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    const Type _type = (Type)params[instance].type.get();
    switch (_type) {

    case Type::HDA436T_Serial:
        if (AP_Inclination_HDA436T_Serial::detect(serial_instance)) {
            _add_backend(new AP_Inclination_HDA436T_Serial(state[instance], params[instance]), instance, serial_instance++);
        }
        break;

    case Type::three_HDA436Ts_Serial:
        if (AP_Inclination_3HDA436Ts_Serial::detect(serial_instance)) {
            _add_backend(new AP_Inclination_3HDA436Ts_Serial(state[instance], params[instance]), instance, serial_instance++);
            _add_backend(new AP_Inclination_3HDA436Ts_Serial(state[instance+1], params[instance+1]), instance+1, serial_instance++);
            _add_backend(new AP_Inclination_3HDA436Ts_Serial(state[instance+2], params[instance+2]), instance+2, serial_instance++);
        }
        break;

    //     case Type::SIM:
    // #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    //         _add_backend(new AP_Inclination_SITL(state[instance], params[instance], instance), instance);
    // #endif
    //         break;

    case Type::NONE:
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
}

AP_Inclination_Backend *Inclination::get_backend(uint8_t id) const
{
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

Inclination::Status Inclination::status_location(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

// return true if we have a Inclination with the specified install location
bool Inclination::has_location(enum InstallLocation location) const
{
    return (find_instance(location) != nullptr);
}

// find first Inclination instance with the specified install location
AP_Inclination_Backend *Inclination::find_instance(enum InstallLocation location) const
{
    // first try for a Inclination that is in the specified location
    for (uint8_t i=0; i<num_instances; i++) {
        AP_Inclination_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->location() == location &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct location
    for (uint8_t i=0; i<num_instances; i++) {
        AP_Inclination_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->location() == location) {
            return backend;
        }
    }
    return nullptr;
}

Vector3f Inclination::get_deg_location(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        AP_HAL::panic("Inclination backend is nullptr, please check the hardware and param table:ICLI->LOCATION.");
        return {0,0,0};
    }
    return backend->get_deg_from_location(location);
}
float Inclination::roll_deg_location(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        AP_HAL::panic("Inclination backend is nullptr, please check the hardware and param table:ICLI->LOCATION.");
        return 0;
    }
    return backend->get_roll_deg_from_location(location);
}
float Inclination::yaw_deg_location(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        AP_HAL::panic("Inclination backend is nullptr, please check the hardware and param table:ICLI->LOCATION.");
        return 0;
    }
    return backend->get_yaw_deg_from_location(location);
}
bool Inclination::has_data_location(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint32_t Inclination::last_reading_ms(enum InstallLocation location) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

// get temperature reading in C.  returns true on success and populates temp argument
bool Inclination::get_temp_C_location(enum InstallLocation location, float &temp) const
{
    AP_Inclination_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_temp_C_from_loc(location, temp);
}

//Write an ICLI (inclination) packet
void Inclination::Log_ICLI() const
{
    if (_log_icli_bit == uint32_t(-1)) {
        return;
    }

    /** This comment can be canceled after the ground station modifies param:LOG_BITMASK
        #define MASK_LOG_ICLI (1UL<<21)
        In inclination.set_log_icli_bit(MASK_LOG_ICLI);  */

    // AP_Logger &logger = AP::logger();
    // if (!logger.should_log(_log_icli_bit)) {
    //     return;
    // }

    for (uint8_t i=0; i<INCLINATION_MAX_INSTANCES; i++) {
        const AP_Inclination_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_ICLI_t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_ICLI_MSG),
                    time_us      : AP_HAL::micros64(),
                    instance     : i,
                    roll_deg     : s->get_roll_deg_from_location(s->location()),
                    yaw_deg      : s->get_yaw_deg_from_location(s->location()),
                    status       : (uint8_t)s->status(),
                    location     : s->location(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

bool Inclination::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < INCLINATION_MAX_INSTANCES; i++) {
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Inclination %X: Not Detected", i + 1);
            return false;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "Inclination %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "Inclination %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:
            break;
        }
    }

    return true;
}

Inclination *Inclination::_singleton;

namespace AP
{

Inclination *inclination()
{
    return Inclination::get_singleton();
}

}
