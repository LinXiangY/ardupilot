#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>



#define SLEWING_ENCODER_CPSR_DEFAULT    21510   // encoder turns count per full revolution of the excavator slewing table
#define MAX_SINGLE_TURN_COUNT_DEFAULT   4096    // encoder max single turn count per revolusion
#define MAX_TOTAL_TURNS_COUNT_DEFAULT   64      // encoder total revolusion turns count
#define SLEWINGENCODER_MAX_INSTANCES    1       // for now we only use one absolute encoder

class AE_SlewingEncoder_Backend; 
 
class AE_SlewingEncoder
{
public:
    friend class AP_WheelEncoder_Backend;
    // friend class AE_SlewingEncoder_OID;
    // friend class AE_SlewingEncoder_SITL;

    AE_SlewingEncoder();    

    /* Do not allow copies */
    AE_SlewingEncoder(const AE_SlewingEncoder &other) = delete;
    AE_SlewingEncoder &operator=(const AE_SlewingEncoder&) = delete;

    // get singleton instance
    static AE_SlewingEncoder *get_singleton() {
        return _singleton;
    }

    // SlewingEncoder driver types
    enum SlewingEncoder_Type : uint8_t {
        SlewingEncoder_TYPE_NONE    =   0,
        SlewingEncoder_TYPE_OID     =   1,
        SlewingEncoder_TYPE_SITL    =  10,
    };

    enum SlewingEncoder_Status {
        NotConnected = 0,
        NoData,
        OutOfTotalCountRangeLow,
        OutOfTotalCountRangeHigh,
        Good
    };

    enum Install_Location : uint8_t {
    INSTALL_NONE                = 0,
    INSTALL_SLEWING              = 1,
};

    // The SlewingEncoder structure is filled in by the backend driver
    struct SlewingEncoder_State {
        // Data should be filled in manually
        AP_Int8     type;               // encoder type
        AP_Int16    max_total_turns_count;   // different encoder has different max revolusion counts, default is 64 for encoder OID
        AP_Int16    max_single_turn_count;   // different encoder has different max decision counts, default is 4096 for encoder OID
        AP_Int16    turns_count_per_slewing_revolution; // The full count corresponding to one revolution of the turntable (the sum of total_turns_count and single_turn_count)
        AP_Int8     location;           //install location of this encoder

        //data which is read from sensor directly
        uint32_t    full_turn_count;    // total_turns_count * max_single_turn_count + single_turn_count
        uint16_t    ammeter_amperes;    // the whole machine's real time current in amperes

        // data which is calculated from directly read
        uint16_t    total_turns_count;  // total encoder turns value
        uint16_t    single_turn_count;  // count value for a single turn measured in degree
        float       angle_deg_diff_base2arm;    // angle difference between base and arm

        enum SlewingEncoder_Status  status;     // sensor status
        uint8_t     encoder_valid_count;    // number of consecutive valid readings (maxes out at 10), if >3 we consider the data valid
        uint32_t    last_reading_ms;     // time of last reading        
    };

    // init - perform require initialisation including detecting which protocol to use
    void init(void);

    // update state of encoder. Should be called from main loop
    void update(void);

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // return true if the instance is enabled
    bool enabled(uint8_t instance) const;

    // find first slewing encoder instance with the specified install location
    AE_SlewingEncoder_Backend *find_instance(enum Install_Location location) const;

    AE_SlewingEncoder_Backend *get_backend(uint8_t id) const;

    // methods to return an angle difference between base and arm on an install location
    // from any sensor which can current supply it
    float get_angle_deg_diff_base2arm_loc(enum Install_Location location) const;
    uint16_t get_single_turn_count_loc(enum Install_Location location) const;
    uint16_t get_total_turns_count_loc(enum Install_Location location) const;
    uint32_t get_full_turns_counts_loc(enum Install_Location location) const;
    
    // get the system time (in milliseconds) of the last update
    uint32_t get_last_reading_ms(uint8_t instance) const;

    bool _add_backend(AE_SlewingEncoder_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    // log data to logger, Write an SLEN (Slewing Encoder) packet
    void Log_SLEN() const;

    // returns true if pre-arm checks have passed for slewing encoder 
    bool pre_arm_check() const;

    static const struct AP_Param::GroupInfo var_info[];


private:
    static AE_SlewingEncoder *_singleton;
    SlewingEncoder_State _state[SLEWINGENCODER_MAX_INSTANCES];
    AE_SlewingEncoder_Backend *drivers[SLEWINGENCODER_MAX_INSTANCES];
    uint8_t num_instances;
    bool init_done;
    HAL_Semaphore detect_sem;
    void detect_instance(uint8_t instance, uint8_t& serial_instance);
};

namespace AE {
    AE_SlewingEncoder *slewingencoder();
}
