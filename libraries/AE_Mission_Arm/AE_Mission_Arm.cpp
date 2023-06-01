#include "AE_Mission_Arm.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

HAL_Semaphore AE_Mission_Arm::_rsem;

StorageAccess AE_Mission_Arm::_storage(StorageManager::StorageRobotArmWP);

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
void AE_Mission_Arm::stop()
{
    _flags.state = MISSION_STOPPED;
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AE_Mission_Arm::start()
{
    _flags.state = MISSION_RUNNING;

    reset(); // reset mission to the first command, resets jump tracking

    // advance to the first command
    if (!advance_current_cmd()) {
        // on failure set mission complete
        complete();
    }
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AE_Mission_Arm::start_or_resume()
{
    // if mission had completed then start it from the first command
    if (_flags.state == MISSION_COMPLETE) {
        gcs().send_text(MAV_SEVERITY_INFO, "start()");

        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid command index restart from beginning
        if (_current_index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }
    }

    // ensure cache coherence
    if (!read_cmd_from_storage(_current_index, _cmd)) {
        // if we failed to read the command from storage, then the command may have
        // been from a previously loaded mission it is illogical to ever resume
        // flying to a command that has been excluded from the current mission
        start();
        return;
    }

    // restart active command. We run these on resume()
    // regardless of whether the mission was stopped, as we may be
    // re-entering TBM mode and the cmd callback needs to be run
    // to setup the current target waypoint
    if (_flags.cmd_loaded) {
        // restart the active command
        set_current_cmd(_current_index);
    }
}

/// reset - reset mission to the first command
void AE_Mission_Arm::reset()
{
    _flags.cmd_loaded      = false;
    _current_index         = AP_MISSION_CMD_INDEX_NONE;
}

// set_current_cmd - jumps to command specified by index
bool AE_Mission_Arm::set_current_cmd(uint16_t index)
{
    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop current cmd
    _flags.cmd_loaded = false;

    // if the mission is stopped or completed move the index to the specified point and set the state to stopped
    // so that if the user resumes the mission it will begin at the specified index
    if (_flags.state != MISSION_RUNNING) {
        RobotArmLocation cmd = {};
        // get next command
        if (!get_next_cmd(index, cmd, true)) {
            _current_index = AP_MISSION_CMD_INDEX_NONE;
            return false;
        }

        _cmd = cmd;
        _flags.cmd_loaded = true;


        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        return true;
    }

    // the state must be MISSION_RUNNING, allow advance_current_cmd() to manage starting the item
    if (!advance_current_cmd(index)) {
        // on failure set mission complete
        complete();
        return false;
    }

    // if we got this far we must have successfully advanced the command
    return true;
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AE_Mission_Arm::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    // check if we have an active command
    if (!_flags.cmd_loaded || _current_index == AE_Mission_Arm_CMD_INDEX_NONE) {
        // advance in mission if no active command
        if (!advance_current_cmd()) {
            // failure to advance command means mission has completed
            complete();
            return;
        }
    } else {
        // run the active command
        if (verify_command(_cmd)) {
            // market _cmd as complete (it will be started on the next iteration)
            _flags.cmd_loaded = false;
            // immediately advance to the next mission command
            if (!advance_current_cmd()) {
                // failure to advance command means mission has completed
                complete();
                return;
            }
        }
    }
}

bool AE_Mission_Arm::advance_current_cmd(uint16_t starting_index)
{
    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current command has not completed
    if (_flags.cmd_loaded) {
        return false;
    }

    // get starting point for search
    uint16_t cmd_index = starting_index > 0 ? starting_index - 1 : _current_index;
    if (cmd_index == AE_Mission_Arm_CMD_INDEX_NONE) {
        // start from beginning of the mission command list
        cmd_index = AE_Mission_Arm_FIRST_REAL_COMMAND;
    } else {
        // start from one position past the current command
        cmd_index++;
    }

    RobotArmLocation cmd = {};

    if (!get_next_cmd(cmd_index, cmd, true)) {
        return false;
    }

    _cmd = cmd;
    _current_index++;

    if (start_command(_cmd)) {
        _flags.cmd_loaded = true;
    }

    return true;
}

bool AE_Mission_Arm::get_next_cmd(uint16_t start_index, RobotArmLocation& cmd, bool send_gcs_msg)
{
    uint16_t cmd_index = start_index;
    RobotArmLocation temp_cmd;

    if (cmd_index < (unsigned)_cmd_total) {
        // load the next command
        if (!read_cmd_from_storage(cmd_index, temp_cmd)) {

            return false;
        }

        cmd = temp_cmd;

        return true;
    }

    return false;
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AE_Mission_Arm::read_cmd_from_storage(uint16_t index, RobotArmLocation& cmd) const
{
    WITH_SEMAPHORE(_rsem);

    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // ensure all bytes of cmd are zeroed
    cmd = {};

    const uint16_t pos_in_storage = index * AE_Mission_Arm_EEPROM_COMMAND_SIZE;

    _storage.read_block(&cmd, pos_in_storage, AE_Mission_Arm_EEPROM_COMMAND_SIZE);

    // return success
    return true;
}

bool AE_Mission_Arm::start_command(const RobotArmLocation& cmd)
{
    return _cmd_start_fn(cmd);
}

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AE_Mission_Arm::complete()
{
    if (_flags.state == MISSION_STOPPED) {
        return;
    }

    // flag mission as complete
    _flags.state = MISSION_COMPLETE;

    // callback to main program's mission complete function
    _mission_complete_fn();

    _cmd_total.set_and_save(0);
}

bool AE_Mission_Arm::verify_command(const RobotArmLocation& cmd)
{
    return _cmd_verify_fn(cmd);
}

void AE_Mission_Arm::set_cmd_total(uint8_t num)
{
    _cmd_total.set_and_save(num);
}

// singleton instance
AE_Mission_Arm *AE_Mission_Arm::_singleton;

namespace AE
{

AE_Mission_Arm *ae_mission_arm()
{
    return AE_Mission_Arm::get_singleton();
}

}
