#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_Inclination_Params.h"

// Maximum number of inclination sensor instances available on this platform
#ifndef INCLINATION_MAX_INSTANCES
#define INCLINATION_MAX_INSTANCES 3
#endif

class AP_Inclination_Backend;

class Inclination
{
    friend class AP_Inclination_Backend;

public:
    Inclination();

    /* Do not allow copies */
    Inclination(const Inclination &other) = delete;
    Inclination &operator=(const Inclination&) = delete;

    // Inclination driver types
    enum class Type {
        NONE   = 0,
        HDA436T_Serial = 1,
        three_HDA436Ts_Serial = 2,
        SIM = 100,
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };

    // The Inclination_State structure is filled in by the backend driver
    struct Inclination_State {
        Vector3f roll_deg;                 // roll_deg.x/.y/.z denote boom/forearm/bucket angle in degree
        Vector3f pitch_deg;                // pitch_deg.x/.y/.z denote boom/forearm/bucket angle in degree
        Vector3f yaw_deg;                  // yaw_deg.x/.y/.z denote boom/forearm/bucket angle in degree
        Vector3f temperature;              // temperature.x/.y/.z denote boom/forearm/bucket angle in degree
        enum Inclination::Status status; // sensor status
        uint8_t  range_valid_count;     // number of consecutive valid readings (maxes out at 3)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[INCLINATION_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void set_log_icli_bit(uint32_t log_icli_bit)
    {
        _log_icli_bit = log_icli_bit;
    }


    //Return the number of inclination instances.
    uint8_t num_sensors(void) const
    {
        return num_instances;
    }

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // detect and initialise any available inclinations
    void init(InstallLocation location);

    // update state of all inclinations. Should be called at around
    // 10Hz from main loop
    void update(void);

    // return true if we have a inclination with the specified install location
    bool has_location(enum InstallLocation location) const;

    // find inclination instance with the specified installation location
    AP_Inclination_Backend *find_instance(enum InstallLocation location) const;

    AP_Inclination_Backend *get_backend(uint8_t id) const;

    // get inclination type for an ID
    Type get_type(uint8_t id) const
    {
        return id >= INCLINATION_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    // methods to return an angle on a particular installation location from
    // any sensor which can current supply it
    float slewing_deg_location(enum InstallLocation location) const;
    float roll_deg_location(enum InstallLocation location) const;
    float yaw_deg_location(enum InstallLocation location) const;
    //return Vector3f deg.x/y/z is roll/pitch/yaw
    Vector3f get_deg_location(enum InstallLocation location) const;
    Inclination::Status status_location(enum InstallLocation location) const;
    bool has_data_location(enum InstallLocation location) const;
    uint32_t last_reading_ms(enum InstallLocation location) const;
    bool get_temp_C_location(enum InstallLocation location, float &temp) const;

    static Inclination *get_singleton(void)
    {
        return _singleton;
    }

protected:
    AP_Inclination_Params params[INCLINATION_MAX_INSTANCES];

private:
    static Inclination *_singleton;

    Inclination_State state[INCLINATION_MAX_INSTANCES];
    AP_Inclination_Backend *drivers[INCLINATION_MAX_INSTANCES];
    uint8_t num_instances;
    bool init_done;
    HAL_Semaphore detect_sem;

    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AP_Inclination_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    uint32_t _log_icli_bit = -1;
    void Log_ICLI() const;
};

namespace AP
{
Inclination *inclination();
};
