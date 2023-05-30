/// @file    AE_RobotArmWP.h
/// @brief   Handles robotic arm way point storage, retrieval and lookup

/*
 * The AE_RobotArmWP library:
 *
 * Initial implementation: andychen, May 2023
 *
 * - responsible for managing a list of robotic arm way points
 * - reads and writes the robotic arm way points to storage
 * - provides access to the robotic arm way points
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_ROBOTARMWP_ENABLED
#define HAL_ROBOTARMWP_ENABLED 1
#endif

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

struct PACKED RobotArmLocation {
    float xhorizontal;          //Horizontal position of cutting_head/bucket_tip in rear view (relatively in the range 0-1)
    float yvertical;          //Vertical position of cutting_head/bucket_tip in rear view (relatively in the range 0-1)
    float zalt;        //The altitude of the cutting_head/bucket_tip in meters (relatively);
    float flags;      //reserve
};

/// @class    AE_RobotArmWP
/// @brief    Object managing robotic arm way points
class AE_RobotArmWP
{
public:
    AE_RobotArmWP();

    /* Do not allow copies */
    AE_RobotArmWP(const AE_RobotArmWP &other) = delete;
    AE_RobotArmWP &operator=(const AE_RobotArmWP&) = delete;

    // data handling
    bool get_rbtarm_waypoint_with_index(uint8_t i, RobotArmLocation &ret) const;
    bool set_rbtarm_waypoint_with_index(uint8_t i, const RobotArmLocation &rbtArmLoc);
    uint8_t get_rbtarm_waypoint_total() const
    {
        return (uint8_t)_rbtarm_waypoint_total_count;
    }
    uint8_t get_rbtarm_waypoint_max(void) const
    {
        const uint16_t ret = _storage.size() / uint16_t(sizeof(RobotArmLocation));
        if (ret > 255) {
            return 255;
        }
        return (uint8_t)ret;
    }
    // reduce point count:
    void truncate(uint8_t num);
    // append a robotic arm way point to the list
    bool append(const RobotArmLocation &loc) WARN_IF_UNUSED;

    // last time robotic arm way points changed
    uint32_t last_change_time_ms(void) const
    {
        return _last_change_time_ms;
    }

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AE_RobotArmWP *get_singleton()
    {
        return _singleton;
    }


private:
    static AE_RobotArmWP *_singleton;

    virtual bool is_valid(const RobotArmLocation &robot_arm_waypoint) const
    {
        return true;
    }

    static StorageAccess _storage;

    // parameters
    AP_Int8  _rbtarm_waypoint_total_count;


    uint32_t _last_change_time_ms = 0xFFFFFFFF;
};

namespace AE
{
AE_RobotArmWP *robotarmwp();
};
