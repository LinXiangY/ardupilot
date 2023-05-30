/// @file    AE_RobotArmWP.h
/// @brief   Handles robotic arm way point storage, retrieval and lookup.
#include "AE_RobotArmWP.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <StorageManager/StorageManager.h>

#if HAL_ROBOTARMWP_ENABLED
// storage object
StorageAccess AE_RobotArmWP::_storage(StorageManager::StorageRobotArmWP);

assert_storage_size<RobotArmLocation, 16> _assert_storage_size_RobotArmLocation;

const AP_Param::GroupInfo AE_RobotArmWP::var_info[] = {
    // @Param: TOTAL
    // @DisplayName: Robotic Arm Way Points Total
    // @Description: Number of robotic arm way points currently loaded
    // @User: Advanced
    AP_GROUPINFO("TOTAL", 0, AE_RobotArmWP, _rbtarm_waypoint_total_count, 0),

    AP_GROUPEND
};

// constructor
AE_RobotArmWP::AE_RobotArmWP()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("robotic arm way points must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// get a robotic arm way point from EEPROM
bool AE_RobotArmWP::get_rbtarm_waypoint_with_index(uint8_t i, RobotArmLocation &ret) const
{
    if (i >= (uint8_t) _rbtarm_waypoint_total_count) {
        return false;
    }

    _storage.read_block(&ret, i * sizeof(RobotArmLocation), sizeof(RobotArmLocation));

    if (abs(ret.xhorizontal) > 1 && abs(ret.yvertical) > 1) {
        return false; // sanity check,for the range of RobotArmLocation is 0-100
    }

    return true;
}

void AE_RobotArmWP::truncate(uint8_t num)
{
    if (num > _rbtarm_waypoint_total_count) {
        // we never make the space larger this way
        return;
    }
    _rbtarm_waypoint_total_count.set_and_save_ifchanged(num);
}

bool AE_RobotArmWP::append(const RobotArmLocation &loc)
{
    const uint8_t current_total = get_rbtarm_waypoint_total();
    _rbtarm_waypoint_total_count.set_and_save_ifchanged(current_total + 1);
    if (!set_rbtarm_waypoint_with_index(current_total, loc)) {
        _rbtarm_waypoint_total_count.set_and_save_ifchanged(current_total);
        return false;
    }
    return true;
}

// save a robotic arm way point to EEPROM - this assumes that the AE_RobotArmWP_TOTAL param has been incremented beforehand, which is the case in Mission Planner
bool AE_RobotArmWP::set_rbtarm_waypoint_with_index(uint8_t i, const RobotArmLocation &rbtArmLoc)
{
    if (i >= (uint8_t) _rbtarm_waypoint_total_count) {
        return false;
    }

    if (i >= get_rbtarm_waypoint_max()) {
        return false;
    }

    _storage.write_block(i * sizeof(RobotArmLocation), &rbtArmLoc, sizeof(RobotArmLocation));

    _last_change_time_ms = AP_HAL::millis();

    AP::logger().Write_RobotArmWayPoint(_rbtarm_waypoint_total_count, i, rbtArmLoc);

    return true;
}

// singleton instance
AE_RobotArmWP *AE_RobotArmWP::_singleton;

namespace AE
{

AE_RobotArmWP *robotarmwp()
{
    return AE_RobotArmWP::get_singleton();
}

}
#endif //HAL_ROBOTARMWP_ENABLED
