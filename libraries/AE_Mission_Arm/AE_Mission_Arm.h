#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AE_RobotArmWP/AE_RobotArmWP.h>

#define AE_Mission_Arm_CMD_INDEX_NONE           65535   // command index of 65535 means invalid or missing command

#define AE_Mission_Arm_EEPROM_COMMAND_SIZE      16      // size in bytes of RobotArmLocation

#define AE_Mission_Arm_FIRST_REAL_COMMAND       0       // first command index

class AE_Mission_Arm
{
public:
    /* Do not allow copies */
    AE_Mission_Arm(const AE_Mission_Arm &other) = delete;
    AE_Mission_Arm &operator=(const AE_Mission_Arm&) = delete;

    // main program function pointers
    FUNCTOR_TYPEDEF(mission_cmd_fn_t, bool, const RobotArmLocation&);
    FUNCTOR_TYPEDEF(mission_complete_fn_t, void);

    // constructor
    AE_Mission_Arm(mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
        _cmd_start_fn(cmd_start_fn),
        _cmd_verify_fn(cmd_verify_fn),
        _mission_complete_fn(mission_complete_fn)
    {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (_singleton != nullptr) {
            AP_HAL::panic("Mission must be singleton");
        }
#endif
        _singleton = this;

        // clear commands
        _current_index = AE_Mission_Arm_CMD_INDEX_NONE;

        _flags.state = MISSION_COMPLETE;
    }

    // get singleton instance
    static AE_Mission_Arm *get_singleton()
    {
        return _singleton;
    }

    /// num_commands - returns total number of commands in the mission
    uint16_t num_commands() const
    {
        return _cmd_total;
    }

    /// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
    void stop();

    void start_or_resume();

    /// resume - continues the mission execution from where we last left off
    ///     previous running commands will be re-initialized
    void start();

    /// reset - reset mission to the first command
    void reset();

    // set_current_cmd - jumps to command specified by index
    bool set_current_cmd(uint16_t index);

    void set_cmd_total(uint8_t num);

    // mission state enumeration
    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_COMPLETE=2
    };

    /// status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped
    mission_state state() const
    {
        return _flags.state;
    }

    struct Mission_Flags {
        mission_state state;
        bool cmd_loaded;
    } _flags;

    /// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
    ///     should be called at 10hz or higher
    void update();

private:

    bool advance_current_cmd(uint16_t starting_index=0);

    bool get_next_cmd(uint16_t start_index, RobotArmLocation& cmd, bool send_gcs_msg=true);

    bool read_cmd_from_storage(uint16_t index, RobotArmLocation& cmd) const;

    bool start_command(const RobotArmLocation& cmd);

    bool verify_command(const RobotArmLocation& cmd);

    void complete();

    AP_Int16      _cmd_total;  // total number of commands in the mission
    struct  RobotArmLocation _cmd; // current command

    uint16_t _current_index;             // current commands position in the command list

    static StorageAccess _storage;

    // multi-thread support. This is static so it can be used from
    // const functions
    static HAL_Semaphore _rsem;

    static AE_Mission_Arm *_singleton;

    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes
};

namespace AE
{
AE_Mission_Arm *ae_mission_arm();
};
