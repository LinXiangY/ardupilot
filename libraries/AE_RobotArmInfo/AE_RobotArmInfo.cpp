#include <AP_AHRS/AP_AHRS.h>

#include "AE_RobotArmInfo.h"
#include "AE_RobotArmInfo_Backend.h"
#include "AE_RobotArmInfo_TBM.h"
#include "AE_RobotArmInfo_Excavator.h"
#include "AE_RobotArmInfo_SITL.h"

extern const AP_HAL::HAL &hal;


const AP_Param::GroupInfo AE_RobotArmInfo::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: AE_RobotArmInfo enabled/disabled
    // @Description: AE_RobotArmInfo enabled/disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AE_RobotArmInfo, _enabled, 0, AP_PARAM_FLAG_ENABLE),
    // AP_GROUPINFO("ENABLED",    1, AE_RobotArmInfo, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: AE_RobotArmInfo Type
    // @Description: AE_RobotArmInfo Type
    // @Values: 0:None, 1:TBM, 2:EXCAVATOR, 3:SITL
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AE_RobotArmInfo, _type, 0),

    // @Param: BOOM_CYLINDER_MAX
    // @DisplayName: Boom cylinder maximum stroke
    // @Description: The maximum stroke of the boom oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("CLD1_BM_MX",    2, AE_RobotArmInfo, excavtor_param._cylinder_max[0], 0),

    // @Param: FOREARM_CYLINDER_MAX
    // @DisplayName: Forearm cylinder maximum stroke
    // @Description: The maximum stroke of the forearm oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("CLD2_FM_MX",    3, AE_RobotArmInfo, excavtor_param._cylinder_max[1], 0),

    // @Param: BOOM_CYLINDER_MAX
    // @DisplayName: Bucket cylinder maximum stroke
    // @Description: The maximum stroke of the bucket oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("CLD3_BK_MX",    4, AE_RobotArmInfo, excavtor_param._cylinder_max[2], 0),

    //////////////////////////////////////////
    //Excavator param
    // @Param: Distance_Boom_to_Slewing
    // @DisplayName: distance boom to slewing
    // @Description: distance_boom_to_slewing
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV1_JC",    5, AE_RobotArmInfo, excavtor_param._mm_JC, 39),

    // @Param: Distance_CF
    // @DisplayName: distance CF
    // @Description: distance CF
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV2_CF",    6, AE_RobotArmInfo, excavtor_param._mm_CF, 408),

    // @Param: Distance_FQ
    // @DisplayName: distance FQ
    // @Description: distance FQ
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV3_FQ",    7, AE_RobotArmInfo, excavtor_param._mm_FQ, 194),

    // @Param: Distance_QV
    // @DisplayName: distance QV
    // @Description: distance QV
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV4_QV",    8, AE_RobotArmInfo, excavtor_param._mm_QV, 147),

    // @Param: Distance JL
    // @DisplayName: distance JL
    // @Description: distance JL
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV5_JL",    9, AE_RobotArmInfo, excavtor_param._mm_JL, 184),

    // @Param: Distance_QN
    // @DisplayName: distance QN
    // @Description: distance_QN
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV6_QN",    10, AE_RobotArmInfo, excavtor_param._mm_QN, 33),

    // @Param: Distance_QK
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV7_QK",    11, AE_RobotArmInfo, excavtor_param._mm_QK, 44),

    // @Param: Distance_MK
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV8_MK",    12, AE_RobotArmInfo, excavtor_param._mm_MK, 54),

    // @Param: Distance_MN
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV9_MN",    13, AE_RobotArmInfo,excavtor_param._mm_MN, 53),

    // @Param: Distance_BC
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV10_BC",    14, AE_RobotArmInfo, excavtor_param._mm_BC, 196.8),

    // @Param: Distance_AC
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV11_AC",    15, AE_RobotArmInfo, excavtor_param._mm_AC, 66),

    // @Param: Distance_DF
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV12_DF",    16, AE_RobotArmInfo, excavtor_param._mm_DF, 234.5),

    // @Param: Distance_GN
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV13_GN",    17, AE_RobotArmInfo, excavtor_param._mm_GN, 183),

    // @Param: Distance_EF
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_EXV14_EF",    18, AE_RobotArmInfo, excavtor_param._mm_EF, 66),

    // @Param: Angle_BFC
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX1_BFC",    19, AE_RobotArmInfo, excavtor_param._deg_BFC, 26),

    // @Param: Angle_FNQ
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX2_FNQ",    20, AE_RobotArmInfo, excavtor_param._deg_FNQ, 176),

    // @Param: Angle_GNF
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX3_GNF",    21, AE_RobotArmInfo, excavtor_param._deg_GNF, 21),

    // @Param: Angle_NQF
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX4_NQF",    22, AE_RobotArmInfo, excavtor_param._deg_NQF, 6),

    // @Param: Angle_KQV
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX5_KQV",    23, AE_RobotArmInfo, excavtor_param._deg_KQV, 110),

    // @Param: Angle_BCF
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX6_BCF",    24, AE_RobotArmInfo, excavtor_param._deg_BCF, 35),

    // @Param: Angle_TCA
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX7_TCA",    25, AE_RobotArmInfo, excavtor_param._deg_TCA, 45),

    // @Param: Angle_DFC
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX8_DFC",    26, AE_RobotArmInfo, excavtor_param._deg_DFC, 50),

    // @Param: Angle_QFG
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX9_QFG",    27, AE_RobotArmInfo, excavtor_param._deg_QFG, 97),

    // @Param: Angle_GFE
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX10_GFE",    28, AE_RobotArmInfo, excavtor_param._deg_GFE, 46),

    // @Param: Angle_GNQ
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_EX11_GNQ",    29, AE_RobotArmInfo, excavtor_param._deg_GNQ, 162),
    //Excavator param
    //////////////////////////////////////////
    //////////////////////////////////////////
    //TBM param
    // @Param: TBM cutting header BOOM_CYLINDER_MAX
    // @DisplayName: TBM cutting header boom cylinder maximum stroke
    // @Description: The maximum stroke of the TBM cutting header boom oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("TBM0_BM_MX",    30, AE_RobotArmInfo, tbm_param._ch_cylinder_max[0], 0),

    // @Param: TBM_CUTTING_HEADER_HORIZONTAL_CYLINDER_MAX
    // @DisplayName: TBM cutting header horizontal cylinder maximum stroke
    // @Description: The maximum stroke of the TBM cutting header horizontal oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("TBM1_HZ_MX",    31, AE_RobotArmInfo, tbm_param._ch_cylinder_max[1], 0),
    
    // @Param: Distance_AC
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_TBM2_AC",    32, AE_RobotArmInfo, tbm_param._mm_AC, 66),
    
    // @Param: Distance_BC
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_TBM3_BC",    33, AE_RobotArmInfo, tbm_param._mm_BC, 196.8),

    // @Param: Distance_CF
    // @DisplayName: distance CF
    // @Description: distance CF
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_TBM4_CF",    34, AE_RobotArmInfo, tbm_param._mm_CF, 408),

    // @Param: Distance JL
    // @DisplayName: distance JL
    // @Description: distance JL
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_TBM5_JL",    35, AE_RobotArmInfo, tbm_param._mm_JL, 184),

    // @Param: Distance_Boom_to_Slew
    // @DisplayName: distance boom to slew
    // @Description: distance_boom_to_slew
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("_TBM6_JC",    36, AE_RobotArmInfo, tbm_param._mm_JC, 39),

    // @Param: Angle_BFC
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_TBM7_BFC",    37, AE_RobotArmInfo, tbm_param._deg_BFC, 26),

    // @Param: Angle_BCF
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_TBM8_BCF",    38, AE_RobotArmInfo, tbm_param._deg_BCF, 35),

    // @Param: Angle_TCA
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("_TBM9_TCA",    39, AE_RobotArmInfo, tbm_param._deg_TCA, 45),

    // @Param: TBM_BACK_SUPPORT_LEFT_CYLINDER_MAX
    // @DisplayName: TBM back support leg left cylinder maximum stroke
    // @Description: The maximum stroke of the TBM back support leg left oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("T10_BSL_MX",    40, AE_RobotArmInfo, tbm_param._bsl_cylinder_max[0], 0),

    // @Param: TBM_BACK_SUPPORT_RIGHT_CYLINDER_MAX
    // @DisplayName: TBM back support leg right cylinder maximum stroke
    // @Description: The maximum stroke of the TBM back support leg right oil cylinder.
    // @Range: 0 260
    // @Increment: 0.1
    // @User: Advanced
    // @Units: mm
    AP_GROUPINFO("T11_BSR_MX",    41, AE_RobotArmInfo, tbm_param._bsl_cylinder_max[1], 0),


    //TBM param
    //////////////////////////////////////////
    AP_GROUPEND
};

// Default constructor.
AE_RobotArmInfo::AE_RobotArmInfo()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AE_RobotArmInfo must be singleton");
    }
    _singleton = this;

    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}


// perform any required initialisation of robot arm info
// update_rate_hz should be the rate at which the update method will be called in hz
void AE_RobotArmInfo::init(void)
{
    // exit immediately if init has already been run
    if (_backend != nullptr) {
        return;
    }
hal.console->printf("\r\n-------------AE_RobotArmInfo::init--------------...\r\n");
    // default health to false
    _backend = nullptr;
    _backend_state.flags.healthy = false;

    // instantiate backend based on type parameter
    switch ((Type)(_type.get())) {
        // no type defined
        case Type::NONE:
        default:
            return;
        // TBM
        case Type::TBM:
            _backend = new AE_RobotArmInfo_TBM(*this, _backend_state);
            break;
        // excavator
        case Type::EXCAVATOR:
            _backend = new AE_RobotArmInfo_Excavator(*this, _backend_state);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case Type::SITL:
            _backend = new AE_RobotArmInfo_SITL(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != nullptr) {
        _backend->init();
    }
}

// update - give chance to inclination sensors to updates the robot arm infomation
void AE_RobotArmInfo::update()
{
    // exit immediately if not enabled
    if (_backend == nullptr) {
        return;
    }

    // update estimator of robot arm info
    if (_backend != nullptr && _enabled) {
        _backend->update();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_log_ms > 40) {  // 25Hz
        last_log_ms = now;
        Write_RobotArmInfo();
    }
}

void AE_RobotArmInfo::Write_RobotArmInfo()
{
    // AP::logger().Write("ARMP", "TimeUS,tip_x,tip_y,tip_z,boom_cyl,forearm_cyl,bucket_cyl,boom,s,b",
    //             "sm", // units: seconds, meters
    //             "FB", // mult: 1e-6, 1e-2
    //             "Qfffffffff", // format: uint64_t, float, float, float
    //             AP_HAL::micros64(),
    //             (float)_position_tip_to_body.x,
    //             (float)_position_tip_to_body.y,
    //             (float)_position_tip_to_body.z,
    //             (float)_cyl_boom_length,
    //             (float)_cyl_forearm_length,
    //             (float)_cyl_bucket_length,
    //             (float)_euler_boom_e2b_from_sensor.x,
    //             (float)_euler_forearm_e2b_from_sensor.x,
    //             (float)_euler_bucket_e2b_from_sensor.x);
}

// handle_msg - Process a LANDING_TARGET mavlink message
// void AE_RobotArmInfo::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
// {
//     // run backend update
//     if (_backend != nullptr) {
//         _backend->handle_msg(packet, timestamp_ms);
//     }
// }

// singleton instance
AE_RobotArmInfo *AE_RobotArmInfo::_singleton;

namespace AE {

AE_RobotArmInfo *ae_robot_arm_info()
{
    return AE_RobotArmInfo::get_singleton();
}

}
