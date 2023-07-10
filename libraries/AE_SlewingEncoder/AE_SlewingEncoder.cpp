#include <AP_Logger/AP_Logger.h>
#include <AP_Inclination/AP_Inclination.h>
#include "AE_SlewingEncoder.h"
#include "AE_SlewingEncoder_OID.h"
#include "AE_SlewingEncoder_SITL.h"


extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AE_SlewingEncoder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: AE_SlewingEncoder type
    // @Description: What type of AE_SlewingEncoder is connected
    // @Values: 0:None,1:OID,10:SITL
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 0, AE_SlewingEncoder, _state[0].type, (uint8_t)SlewingEncoder_TYPE_NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: _CPSR
    // @DisplayName: SlewingEncoder turns Count Per excavator Slewing 1 Revolution
    // @Description: One turn of the excavator slewing table corresponds to the number of turns of the encoder
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_CPSR",   1, AE_SlewingEncoder, _state[0].turns_count_per_slewing_revolution, SLEWING_ENCODER_CPSR_DEFAULT),

    // @Param: _MXSTC
    // @DisplayName: SlewingEncoder max single turn count per revolusion
    // @Description: different encoder has different max revolusion counts
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MXSTC",  2, AE_SlewingEncoder, _state[0].max_single_turn_count, MAX_SINGLE_TURN_COUNT_DEFAULT),

    // @Param: _MXTTC
    // @DisplayName: SlewingEncoder total revolusion turns count
    // @Description: different encoder has different total revolusion counts
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MXTTC",  3, AE_SlewingEncoder, _state[0].max_total_turns_count, MAX_TOTAL_TURNS_COUNT_DEFAULT),

    // @Param: _LOCAT
    // @DisplayName: encoder install location
    // @Description: Location of absolute encoder
    // @Values: 0:INSTALL_NONE, 1: INSTALL_SLEWING
    // @User: Advanced
    AP_GROUPINFO("_LOCAT", 4, AE_SlewingEncoder, _state[0].location, Install_Location::INSTALL_SLEWING),

    AP_GROUPEND
};


AE_SlewingEncoder::AE_SlewingEncoder()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AE_SlewingEncoder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/* 
  initialise the AE_SlewingEncoder class. We do detection of attached slewing 
  encoder here. For now we won't allow for hot-plugging of slewing encoder.
  */
void AE_SlewingEncoder::init(void)
{   
    if (init_done) {
        // init called a 2nd time?
        return;
    }
    init_done = true;
    
    for (uint8_t i=0, serial_instance = 0; i<SLEWINGENCODER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN rangefinder may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise variables
        _state[i].total_turns_count = MAX_TOTAL_TURNS_COUNT_DEFAULT;
        _state[i].single_turn_count = MAX_SINGLE_TURN_COUNT_DEFAULT;
        _state[i].ammeter_amperes = 0;
        // initialise status
        _state[i].status = NotConnected;
        _state[i].encoder_valid_count = 0;

    }
}

// update SlewingEncoder state for it's instance. This should be called by main loop
void AE_SlewingEncoder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_state[i].type == SlewingEncoder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                _state[i].status = NotConnected;
                _state[i].encoder_valid_count = 0;
                // _state[i].angle_deg_diff_base2arm = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#if HAL_LOGGING_ENABLED
    Log_SLEN();
#endif    
}

bool AE_SlewingEncoder::_add_backend(AE_SlewingEncoder_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (num_instances == SLEWINGENCODER_MAX_INSTANCES) {
        AP_HAL::panic("Too many Slewing Encoder backends");
    }
    if (drivers[num_instances] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a Slewing Encoder is connected. 
 */
void AE_SlewingEncoder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum SlewingEncoder_Type _type = (enum SlewingEncoder_Type)_state[instance].type.get();
    switch (_type) {
    case SlewingEncoder_TYPE_OID:
         if (AE_SlewingEncoder_OID::detect(serial_instance)) {
            _add_backend(new AE_SlewingEncoder_OID(_state[instance]), instance, serial_instance++);
        }
        break;

    case SlewingEncoder_TYPE_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        _add_backend(new AE_SlewingEncoder_SITL(_state[instance], instance), instance);
#endif
        break;
        
    case SlewingEncoder_TYPE_NONE:
        break;
    default:
        break;
    }
}

AE_SlewingEncoder_Backend *AE_SlewingEncoder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == SlewingEncoder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

// find first range finder instance with the specified orientation
AE_SlewingEncoder_Backend *AE_SlewingEncoder::find_instance(enum Install_Location location) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AE_SlewingEncoder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->location() != location) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

float AE_SlewingEncoder::get_angle_deg_diff_base2arm_loc(enum Install_Location location) const
{
    AE_SlewingEncoder_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_angle_diff_base2arm();
}

uint16_t AE_SlewingEncoder::get_single_turn_count_loc(enum Install_Location location) const
{
    AE_SlewingEncoder_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->single_turn_count();
}

uint16_t AE_SlewingEncoder::get_total_turns_count_loc(enum Install_Location location) const
{
    AE_SlewingEncoder_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->total_turns_count();
}

uint32_t AE_SlewingEncoder::get_full_turns_counts_loc(enum Install_Location location) const
{
    AE_SlewingEncoder_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->full_turns_counts();
}

// check if an instance is activated
bool AE_SlewingEncoder::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (drivers[instance]->type() == SlewingEncoder_Type::SlewingEncoder_TYPE_NONE) {
        return false;
    }
    return true;
}

// log Slewing Encoder information
void AE_SlewingEncoder::Log_SLEN() const
{
    // return immediately if no slewing encoder are enabled
    if (!enabled(0)) {
        return;
    }

     const AP_AHRS &ahrs = AP::ahrs();
     Inclination *inclination = AP::inclination();
    if (inclination == nullptr) {
        AP_HAL::panic("AE_RobotArmInfo_TBM should be got inclination data before calc");
        return;
    }    

    struct log_SlewingEncoder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SLEN_MSG),
        time_us                 : AP_HAL::micros64(),
        angle_diff_base2arm     : degrees(get_angle_deg_diff_base2arm_loc(Install_Location::INSTALL_SLEWING)),
        single_turn_count       : get_single_turn_count_loc(Install_Location::INSTALL_SLEWING),
        total_turns_count       : get_total_turns_count_loc(Install_Location::INSTALL_SLEWING),        
        amp_yaw                 : (uint16_t)ahrs.yaw_sensor,
        full_turns_counts       : get_full_turns_counts_loc(Install_Location::INSTALL_SLEWING),
        inclination_yaw         : inclination->yaw_deg_location(Boom)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// get the system time (in milliseconds) of the last update
uint32_t AE_SlewingEncoder::get_last_reading_ms(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= SLEWINGENCODER_MAX_INSTANCES) {
        return 0;
    }
    return _state[instance].last_reading_ms;
}

bool AE_SlewingEncoder::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < SLEWINGENCODER_MAX_INSTANCES; i++) {
        if ((SlewingEncoder_Type)_state[i].type.get() == SlewingEncoder_Type::SlewingEncoder_TYPE_NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SlewingEncoder %X: Not Detected", i + 1);
            return false;
        }

        switch (drivers[i]->status()) {
        case SlewingEncoder_Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "SlewingEncoder %X: No Data", i + 1);
            return false;
        case SlewingEncoder_Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "SlewingEncoder %X: Not Connected", i + 1);
            return false;
        case SlewingEncoder_Status::OutOfTotalCountRangeHigh:
        case SlewingEncoder_Status::OutOfTotalCountRangeLow:
        case SlewingEncoder_Status::Good:  
            break;
        }
    }

    return true;
}

// singleton instance
AE_SlewingEncoder *AE_SlewingEncoder::_singleton;

namespace AE {
    AE_SlewingEncoder *slewingencoder()
{
    return AE_SlewingEncoder::get_singleton();
}

}
