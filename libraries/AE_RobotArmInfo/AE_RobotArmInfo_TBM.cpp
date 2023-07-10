#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include "AE_RobotArmInfo_TBM.h"
#include <AP_Inclination/AP_Inclination.h>
#include <AE_SlewingEncoder/AE_SlewingEncoder.h>


// perform any required initialisation of backend
void AE_RobotArmInfo_TBM::init()
{
    // update health
    _state.flags.healthy = true;
    // // 在这里把前端的Robot_Arm_State::_backend_state赋初值，
    // // 因为前端的状态量是为了给挖掘机/掘进机不同的后端使用的，所以包含的内容比较多，进入不同的后端需要根据不同后端类型进行相应初始化
    _state.type = (int8_t)AE_RobotArmInfo::Type::TBM;
    
    _cuthead_state.cutheader_horizon_pos = 0;
    _cuthead_state.cutheader_vertical_vel = 0;
    _cuthead_state.cutheader_horizon_vel = 0;
    _back_leg_state.back_support_leg_height = 0;
    _back_leg_state.back_support_leg_hor = 0;
    _last_t_us = AP_HAL::micros64();
    _cutting_header_height_last = 0;
    _cutting_header_hor_last = 0;

    for(uint8_t i=0;i<OIL_CYLINDER_NUM_MAX;i++)
    {
      _cuthead_state.cylinder_status[i].cylinder_name = (AE_RobotArmInfo::TBM_CH_OC_Name)i;
      _cuthead_state.cylinder_status[i].length_max_mm = get_tbm_param()._ch_cylinder_max[i];
      _cuthead_state.cylinder_status[i].length_mm = 0;
      _cuthead_state.cylinder_status[i].velocity_mms = 0;
    }

    for(uint8_t i=0;i<OIL_CYLINDER_NUM_MAX;i++)
    {
      _back_leg_state.cylinder_status[i].cylinder_name = (AE_RobotArmInfo::TBM_BSL_OC_Name)i;
      _back_leg_state.cylinder_status[i].length_max_mm = get_tbm_param()._bsl_cylinder_max[i];
      _back_leg_state.cylinder_status[i].length_mm = 0;
    }


}

// retrieve updates from sensor
void AE_RobotArmInfo_TBM::update()
{

    Inclination *inclination = AP::inclination();
    if(inclination == nullptr){
        return;
    }

    AE_SlewingEncoder *slewingencoder = AE::slewingencoder();
    if(slewingencoder == nullptr){
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();
    if(!calc_TBM_info(ahrs,inclination,slewingencoder)){
       // update health
       _state.flags.healthy = false;
    }    
    
    //计算完成后把挖掘机类的私有变量赋给结构体供外界调用
    if(check_if_cutting_head_info_valid(_cuthead_state))
    {
      _state.flags.healthy = true;
    }
    Write_TBM_CutheadInfo();

    return;
}

bool AE_RobotArmInfo_TBM::calc_TBM_info(const AP_AHRS &ahrs, const Inclination *inclination, const AE_SlewingEncoder *slewingencoder)
{
    if(!update_TBM_cutting_header_state(ahrs,inclination,slewingencoder))
    {
        return false;
    }
    return true;
}
//update the cutting header state at base's body frame
bool AE_RobotArmInfo_TBM::update_TBM_cutting_header_state(const AP_AHRS &ahrs, const Inclination *inclination, const AE_SlewingEncoder *slewingencoder)
{
    //get this time and dt
    uint64_t t_us = AP_HAL::micros64();
    _dt = float((t_us - _last_t_us))/1000000;

    //get transformation matrix for axis change
    Matrix3f transformation;
    Matrix3f boom_matrix;
    transformation.from_euler(ahrs.get_roll(),ahrs.get_pitch(),radians(inclination->yaw_deg_location(Boom)));
    if(!transformation.invert())
    {
        return false;
    }
    Vector3f _euler_boom_e2b_from_sensor = inclination->get_deg_location(Boom);

    //Convert the sensor angle to the angle under the body coordinate system
    boom_matrix.from_euler(radians(_euler_boom_e2b_from_sensor.x),radians(_euler_boom_e2b_from_sensor.y),radians(_euler_boom_e2b_from_sensor.z));
    boom_matrix = transformation*boom_matrix;
    boom_matrix.to_euler(&_euler_boom_e2b_from_sensor.x,&_euler_boom_e2b_from_sensor.y,&_euler_boom_e2b_from_sensor.z);

    //calculate cutting head infomation including height,hor,cyl length,height speed,hor speed 
    //height and hor
    float boom_to_body = _euler_boom_e2b_from_sensor.y + radians(get_ex_param()._deg_BFC);

    // slewing deg while cutting head is in mid deg=0,should use information from encoder but now use inclination
    float slewing_to_body = slewingencoder->get_angle_deg_diff_base2arm_loc(slewingencoder->INSTALL_SLEWING);
    _cuthead_state.cutheader_height =  get_ex_param()._mm_CF*sinf(boom_to_body) + get_ex_param()._mm_JL;
    _cuthead_state.cutheader_horizon_pos = sinf(slewing_to_body)*(get_ex_param()._mm_CF*cosf(boom_to_body) + get_ex_param()._mm_JC);

    //calculate cutting head height cyl length
    float angle_ACB = radians(get_ex_param()._deg_TCA) + radians(get_ex_param()._deg_BCF) + boom_to_body;
    _cuthead_state.cylinder_status[0].length_mm = sqrt(get_ex_param()._mm_AC * get_ex_param()._mm_AC + 
    get_ex_param()._mm_BC * get_ex_param()._mm_BC - 2*get_ex_param()._mm_AC*get_ex_param()._mm_BC*cos(angle_ACB));

    //calculate cutting head hor cyl length
    // _cuthead_state.cylinder_status[1].length_mm = sqrt( 2*get_tbm_param()._mm_OB*get_tbm_param()._mm_OB*(1-cos(slewing_to_body))
    // + get_tbm_param()._mm_AB*get_tbm_param()._mm_AB - 2*get_tbm_param()._mm_OB*get_tbm_param()._mm_AB*sqrt(2*(1-cos(slewing_to_body)))*
    // sin(radians(get_tbm_param()._deg_OBA)-slewing_to_body/2))-get_tbm_param()._mm_AB;

    //speed
    _cuthead_state.cutheader_vertical_vel = (_cuthead_state.cutheader_height - _cutting_header_height_last)/_dt;
    _cuthead_state.cutheader_horizon_vel = (_cuthead_state.cutheader_horizon_pos - _cutting_header_hor_last)/_dt;

    //save this infomation to last infomation for next calculate
    _last_t_us = t_us;
    _cutting_header_height_last = _cuthead_state.cutheader_height;
    _cutting_header_hor_last = _cuthead_state.cutheader_horizon_pos;

    return true;
}

//check if cutting info out of range
bool AE_RobotArmInfo_TBM::check_if_cutting_head_info_valid(struct TBM_Cutting_Header_State& cutting_header_state)
{
    if(cutting_header_state.cylinder_status[0].length_mm > cutting_header_state.cylinder_status[0].length_max_mm 
    //|| cutting_header_state.cylinder_status[1].length_mm > cutting_header_state.cylinder_status[1].length_max_mm
    //|| back_leg_state.cylinder_status[0].length_mm > back_leg_state.cylinder_status[0].length_max_mm
    )
    {  
        return false;
    }
    return true;
}

// bool AE_RobotArmInfo_TBM::update_TBM_back_leg_state(void)
// {
//     float _deg_theta=0; //deg get from back leg sensor
//     _back_leg_state.cylinder_status[0].length_mm = sqrt(get_tbm_param()._mm_CD*get_tbm_param()._mm_CD + get_tbm_param()._mm_DF*get_tbm_param()._mm_DF
//     -2*get_tbm_param()._mm_CD*get_tbm_param()._mm_DF*cosf(radians(get_tbm_param()._deg_CDH)+_deg_theta));

//     float deg_dcf = ((get_tbm_param()._mm_CD*get_tbm_param()._mm_CD + _back_leg_state.cylinder_status[0].length_mm*_back_leg_state.cylinder_status[0].length_mm
//     -get_tbm_param()._mm_DF*get_tbm_param()._mm_DF)/(2*get_tbm_param()._mm_CD*_back_leg_state.cylinder_status[0].length_mm));

//     float deg_fcH = M_PI - deg_dcf - radians(get_tbm_param()._deg_CDH);
//     _back_leg_state.back_support_leg_height = get_tbm_param()._mm_CI + _back_leg_state.cylinder_status[0].length_mm * sinf(deg_fcH);
//     _back_leg_state.back_support_leg_hor = get_tbm_param()._mm_GI + _back_leg_state.cylinder_status[0].length_mm * cosf(deg_fcH);

//     return true;
// }



void AE_RobotArmInfo_TBM::Write_TBM_CutheadInfo()
{
    AP::logger().Write("ARMP", "TimeUS,height,horiz,boomLength,heightvel,horvel",
                       "sm", // units: seconds, meters
                       "FB", // mult: 1e-6, 1e-2
                       "Qfff", // format: uint64_t, float, float, float
                       AP_HAL::micros64(),
                       (float)_cuthead_state.cutheader_height,
                       (float)_cuthead_state.cutheader_horizon_pos,
                       (float)_cuthead_state.cylinder_status[0].length_mm,
                       (float)_cuthead_state.cutheader_vertical_vel,
                       (float)_cuthead_state.cutheader_horizon_vel);
}



