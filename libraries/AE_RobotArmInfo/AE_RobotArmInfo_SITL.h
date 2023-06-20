
#pragma once
#include "AE_RobotArmInfo_Backend.h"


class AE_RobotArmInfo_SITL : public AE_RobotArmInfo_Backend
{

public:
    // Constructor
    using AE_RobotArmInfo_Backend::AE_RobotArmInfo_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;


    //get the cutting header state at base's body frame
    // AP_TBM_Info::TBM_Cutting_Header_State get_TBM_cutting_header_state() const { return _cutting_header_state; }

    //update the cutting header state at base's body frame
    // void  AP_TBM_Info::update_TBM_cutting_header_state(const Vector3f &boom_atti_deg);    

private:

    // AP_TBM_Info::TBM_Cutting_Header_State _cutting_header_state;

};
