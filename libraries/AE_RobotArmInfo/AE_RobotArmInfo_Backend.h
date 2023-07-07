#pragma once

#include <AP_Math/AP_Math.h>
#include "AE_RobotArmInfo.h"
#include <AP_HAL/AP_HAL.h>

class AE_RobotArmInfo_Backend
{
public:
    // Constructor
    AE_RobotArmInfo_Backend(const AE_RobotArmInfo& frontend, AE_RobotArmInfo::Robot_Arm_State& state) :
        _frontend(frontend),
        _state(state) {}

    // destructor
    virtual ~AE_RobotArmInfo_Backend() {}

    // perform any required initialisation of backend
    virtual void init() = 0;

    // retrieve updates from sensor
    virtual void update() = 0;

    virtual bool get_healthy() const {return _state.flags.healthy;};

    virtual void set_healthy(bool ishealthy){_state.flags.healthy = ishealthy;};

    // parses a mavlink message from the GCS or companion computer
    // virtual void handle_msg(const mavlink_rbt_arm_info_t &packet) {};

    // get excavator parameter,distance is mm,deg calculate need to be transformed to radians
    AE_RobotArmInfo::EXCVT_PARAM get_ex_param(void) const { return _frontend.excavtor_param; }

    // get TBM parameter,distance is mm,deg calculate need to be transformed to radians
    AE_RobotArmInfo::TBM_PARAM get_tbm_param(void) const { return _frontend.tbm_param; }
    
protected:
    const AE_RobotArmInfo&  _frontend;          // reference to robot arm info front end
    AE_RobotArmInfo::Robot_Arm_State &_state;   // reference to this instances state
};
