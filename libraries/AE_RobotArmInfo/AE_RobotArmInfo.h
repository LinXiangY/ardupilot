#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#ifndef OIL_CYLINDER_NUM_MAX
#define OIL_CYLINDER_NUM_MAX 2
#endif

class AE_RobotArmInfo_Backend;

class AE_RobotArmInfo   
{
    // declare backends as friends
    friend class AE_RobotArmInfo_Backend;

public:
    AE_RobotArmInfo();

    /* Do not allow copies */
    AE_RobotArmInfo(const AE_RobotArmInfo &other) = delete;
    AE_RobotArmInfo &operator=(const AE_RobotArmInfo&) = delete;

    // return singleton
    static AE_RobotArmInfo *get_singleton() {
        return _singleton;
    }

    // types of robot arm (used for RBTAM_TYPE parameter)
    enum class Type : uint8_t {
        NONE = 0,
        TBM = 1,
        EXCAVATOR = 2,
        SITL = 3,
    };

    enum class Component_name : uint8_t {
        TBM_Cutting_Header =    0,
        EXCAVATOR_ARM =         1,
        TBM_BACK_SUPPORT_LEG =  2,
        TBM_Shovel =            3,
    };

    // Excavator Oli Cylinder Name
    enum class Ex_OC_Name : uint8_t {
        OC_EX_BOOM =         0,
        OC_EX_FOREARM =      1,
        OC_EX_BUCKET =       2,
    };

    // TBM Cutting Header Oli Cylinder Name
    enum class TBM_CH_OC_Name : uint8_t {
        OC_TBM_CUTHEADER_VERTIC =         0,
        OC_TBM_CUTHEADER_HORIZ =      1,
    };

    // TBM Back Support Leg Oli Cylinder Name
    enum class TBM_BSL_OC_Name : uint8_t {
        OC_TBM_BACK_LEG_LEFT =      0,
        OC_TBM_BACK_LEG_RIGHT =     1,
    };

    struct Ex_Cylinder_State {
        float length_mm;                        // mm
        float velocity_mms;                     // mm/s
        float length_max_mm;                    // mm        
        enum Ex_OC_Name cylinder_name;
    };

    struct TBM_CH_Cylinder_State {
        float length_mm;                        // mm
        float velocity_mms;                     // mm/s
        float length_max_mm;                    // mm        
        enum TBM_CH_OC_Name cylinder_name;
    };

    struct AE_RobotArmInfo_Flags {
        uint8_t healthy : 1; // true if sensor is healthy
    } _flags;

    // perform any required initialisation of different arm type
    // update_rate_hz should be the rate at which the update method will be called in hz
    void init(void);

    // calculate cylinder length and component position using inclination sensor data, should be called at 100hz
    void update(void);

    // return true if the backend calculated result is healthy
    bool is_healthy() const { return _backend_state.flags.healthy; }

    //get the cutting header state at base's body frame
    // AE_RobotArmInfo_TBM::TBM_Cutting_Header_State get_TBM_cutting_header_state() const { return _cutting_header_state; }

    // return the current state of the location of the target
    // Component_State get_component_state() const { return _component_state; }

    // process a CUTTING_COAL mavlink message
    // void handle_msg(const mavlink_cutting_coal_t &packet, uint32_t timestamp_ms);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];   

private:

    // backend state
    struct Robot_Arm_State {
        AE_RobotArmInfo_Flags flags;
        uint8_t         type;     //enum AE_RobotArmInfo::Type
        // enum AE_RobotArmInfo::Component_name component;
        // Vector3f body_frame_boom_endpoint_pos;
        // Vector3f body_frame_forearm_endpoint_pos;
        // Vector3f body_frame_bucket_endpoint_pos;
        
        // struct Cylinder_State status[OIL_CYLINDER_NUM_MAX];
    }_backend_state;

    uint32_t    _last_update_ms;    // system time in millisecond when update was last called
    AE_RobotArmInfo_Backend *_backend;  // pointers to different types of backend robot arm instance

    // write out RBTARM message to log:
    void Write_RobotArmInfo();
    uint32_t last_log_ms;  // last time we logged

    // below are parameters shown at the mission planner if the first param "_enabled" is 1
    AP_Int8             _enabled;               // enabled/disabled
    AP_Enum<Type>       _type;                  // Robot Arm Info Type
    //////////////////////////////////////////
    // Excavator param, refer to the website below to determine the specific meaning of the paramters
    // https://gitee.com/andychen183/roadheader_gcs/issues/I7BLTG
    struct EXCVT_PARAM{
        AP_Float    _cylinder_max[3];     // The maximum stroke of the boom/forearm/bucket oil cylinder
        AP_Float    _mm_JC;
        AP_Float    _mm_CF;
        AP_Float    _mm_FQ;
        AP_Float    _mm_QV;
        AP_Float    _mm_JL;
        AP_Float    _mm_QN;
        AP_Float    _mm_QK;
        AP_Float    _mm_MK;
        AP_Float    _mm_MN;
        AP_Float    _mm_BC;
        AP_Float    _mm_AC;
        AP_Float    _mm_DF;
        AP_Float    _mm_GN;
        AP_Float    _mm_EF;
        AP_Float    _deg_BFC;
        AP_Float    _deg_FNQ;
        AP_Float    _deg_GNF;
        AP_Float    _deg_NQF;
        AP_Float    _deg_KQV;
        AP_Float    _deg_BCF;
        AP_Float    _deg_TCA;
        AP_Float    _deg_DFC;
        AP_Float    _deg_QFG;
        AP_Float    _deg_GFE;
        AP_Float    _deg_GNQ;
    } excavtor_param;

    //////////////////////////////////////////
    // TBM param, refer to the website below to determine the specific meaning of the paramters
    // 
    struct TBM_PARAM{
        //TBM_PARAM
    } tbm_param;

    static AE_RobotArmInfo *_singleton; //singleton
};

namespace AE {
    AE_RobotArmInfo *ae_robot_arm_info();
};
