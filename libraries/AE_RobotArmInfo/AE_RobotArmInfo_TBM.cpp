#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AE_RobotArmInfo_TBM.h"
#include <AP_Inclination/AP_Inclination.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AE_RobotArmInfo_Excavator.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;
 // perform any required initialisation of backend
void AE_RobotArmInfo_TBM::init()
{
    // update health
    _state.flags.healthy = true;
    // // 在这里把前端的Robot_Arm_State::_backend_state赋初值，
    // // 因为前端的状态量是为了给挖掘机/掘进机不同的后端使用的，所以包含的内容比较多，进入不同的后端需要根据不同后端类型进行相应初始化
    _state.type = (int8_t)AE_RobotArmInfo::Type::TBM;
    
    _cutting_header_state.cutting_header_height = 0;
    _cutting_header_state.cutting_header_hor = 0;
    _cutting_header_state.cutting_header_speed = 0;

    for(uint8_t i=0;i<OIL_CYLINDER_NUM_MAX;i++)
    {
      _cutting_header_state.cylinder_status[i].cylinder_name = (AE_RobotArmInfo::TBM_CH_OC_Name)i;
      _cutting_header_state.cylinder_status[i].length_max_mm = get_ex_param()._cylinder_max[i];
      _cutting_header_state.cylinder_status[i].length_mm = 0;
      _cutting_header_state.cylinder_status[i].velocity_mms = 0;
    }
}

// retrieve updates from sensor
void AE_RobotArmInfo_TBM::update()
{

    if(!calc_TBM_info()){
       // update health
       _state.flags.healthy = false;
    }    
    
    //计算完成后把挖掘机类的私有变量赋给结构体供外界调用
    if(check_if_cutting_head_info_valid(_cutting_header_state))
    {
      _state.flags.healthy = true;
    }
    Write_TBM_headInfo();

    return;
}

bool AE_RobotArmInfo_TBM::calc_TBM_info(void)
{
    if(!update_TBM_cutting_header_state())
    {
        return false;
    }
    return true;
}
//update the cutting header state at base's body frame
bool AE_RobotArmInfo_TBM::update_TBM_cutting_header_state(void)
{
    Inclination *inclination = AP::inclination();
    if(inclination == nullptr){
        return false;
    }
    const AP_AHRS &ahrs = AP::ahrs();
    Matrix3f transformation;
    Matrix3f boom_matrix;
    transformation.from_euler(ahrs.get_roll(),ahrs.get_pitch(),radians(inclination->yaw_deg_location(Boom)));
    if(!transformation.invert())
    {
        return false;
    }
    Vector3f _euler_boom_e2b_from_sensor = inclination->get_deg_location(Boom);
    boom_matrix.from_euler(radians(_euler_boom_e2b_from_sensor.x),radians(_euler_boom_e2b_from_sensor.y),radians(_euler_boom_e2b_from_sensor.z));
    boom_matrix = transformation*boom_matrix;
    boom_matrix.to_euler(&_euler_boom_e2b_from_sensor.x,&_euler_boom_e2b_from_sensor.y,&_euler_boom_e2b_from_sensor.z);
    float boom_to_body = _euler_boom_e2b_from_sensor.y + get_ex_param()._deg_BFC;
    float slewing_to_body = radians(inclination->yaw_deg_location(Boom));
    _cutting_header_state.cutting_header_height =  get_ex_param()._mm_CF*sinf(boom_to_body) + get_ex_param()._mm_JL;
    _cutting_header_state.cutting_header_hor = sinf(slewing_to_body)*(get_ex_param()._mm_CF*cosf(boom_to_body) + get_ex_param()._mm_JC);
    float angle_ACB = get_ex_param()._deg_TCA + get_ex_param()._deg_BCF + boom_to_body;
    _cutting_header_state.cylinder_status[0].length_mm = sqrt(get_ex_param()._mm_AC * get_ex_param()._mm_AC + get_ex_param()._mm_BC * get_ex_param()._mm_BC - 2*get_ex_param()._mm_AC*get_ex_param()._mm_BC*cos(angle_ACB));
    return true;
}

bool AE_RobotArmInfo_TBM::check_if_cutting_head_info_valid(struct TBM_Cutting_Header_State& cutting_header_state)
{
    if(cutting_header_state.cylinder_status[0].length_mm > cutting_header_state.cylinder_status[0].length_max_mm)
    {
        return true;
    }
    return false;
}


void AE_RobotArmInfo_TBM::Write_TBM_headInfo()
{
    AP::logger().Write("ARMP", "TimeUS,height,horizontal,boom_cyl",
                "sm", // units: seconds, meters
                "FB", // mult: 1e-6, 1e-2
                "Qfff", // format: uint64_t, float, float, float
                AP_HAL::micros64(),
                (float)_cutting_header_state.cutting_header_height,
                (float)_cutting_header_state.cutting_header_hor,
                (float)_cutting_header_state.cylinder_status[0].length_mm);
}

 



