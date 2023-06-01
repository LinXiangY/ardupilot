#include "mode.h"
#include "Rover.h"

bool ModeTBM::__enter()
{
    // do some config (tbm)
    // ...
    gcs().send_text(MAV_SEVERITY_INFO, "ModeTBM::__enter()");

    return true;
}

bool ModeTBM::start_command(const RobotArmLocation& cmd)
{
    // nav to target loc (tbm)
    // ...

    return true;
}

bool ModeTBM::verify_command_callback(const RobotArmLocation& cmd)
{
    // reached_destination ? (tbm)
    // ...

    gcs().send_text(MAV_SEVERITY_INFO, "reach x:%f, y:%f, z:%f\n", cmd.xhorizontal, cmd.yvertical, cmd.zalt);
    
    return true;
}

void ModeTBM::exit_mission()
{
    
}

void ModeTBM::__exit()
{

}
