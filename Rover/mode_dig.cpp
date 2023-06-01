#include "mode.h"
#include "Rover.h"

bool ModeDig::_enter()
{
    stop_vehicle();

    // fail to enter tbm if no mission commands // modify
    if (g2.rbtarmwp.get_rbtarm_waypoint_total() < 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "No Mission. Can't set %s.", name4());
        return false;
    }

    if (!__enter()) {
        gcs().send_text(MAV_SEVERITY_INFO, "fail to enter mode %s", name4());
        return false;
    }

    mission.set_cmd_total(g2.rbtarmwp.get_rbtarm_waypoint_total());

    // set flag to start mission
    waiting_to_start = true;

    return true;
}

void ModeDig::update()
{
    // start or update mission
    if (waiting_to_start) {
        mission.start_or_resume();

        waiting_to_start = false;
    } else {
        mission.update();
    }
}

bool ModeDig::_start_command(const RobotArmLocation& cmd)
{
    bool ret = false;

    ret |= start_command(cmd);

    gcs().send_text(MAV_SEVERITY_INFO, "Mode%s::start_command()\n", name4());

    return ret;
}

void ModeDig::_exit_mission()
{
    exit_mission();

    gcs().send_text(MAV_SEVERITY_INFO, "Mode%s::exit_mission()\n", name4());
}

bool ModeDig::_verify_command_callback(const RobotArmLocation& cmd)
{
    bool ret = false;

    gcs().send_text(MAV_SEVERITY_INFO, "reached_destination ?\n");
    gcs().send_text(MAV_SEVERITY_INFO, ".......................\n");

    ret |= verify_command_callback(cmd);

    return ret;
}

void ModeDig::_exit()
{
    // stop running the mission
    if (mission.state() == AE_Mission_Arm::MISSION_RUNNING) {
        mission.stop();
    }

    __exit();
}
