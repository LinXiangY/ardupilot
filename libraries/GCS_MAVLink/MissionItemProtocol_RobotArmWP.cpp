/*
  Implementation details for transfering robot arm waypoint information using
  the MISSION_ITEM protocol to and from a GCS.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "MissionItemProtocol_RobotArmWP.h"

#include <AP_Logger/AP_Logger.h>
#include <AE_RobotArmWP/AE_RobotArmWP.h>
#include <GCS_MAVLink/GCS.h>


MAV_MISSION_RESULT MissionItemProtocol_RobotArmWP::append_item(const mavlink_mission_item_int_t &cmd)
{
    RobotArmLocation rbtArmloc;
    const MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_RobotArmLocation(cmd, rbtArmloc);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    if (!robotarmwp.append(rbtArmloc)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_RobotArmWP::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Robot arm waypoints received");
    AP::logger().Write_RobotArmWP();
    return MAV_MISSION_ACCEPTED;
}

bool MissionItemProtocol_RobotArmWP::clear_all_items()
{
    robotarmwp.truncate(0);
    return true;
}

MAV_MISSION_RESULT MissionItemProtocol_RobotArmWP::convert_MISSION_ITEM_INT_to_RobotArmLocation(const mavlink_mission_item_int_t &cmd, RobotArmLocation &ret)
{
    if (cmd.command != MAV_CMD_NAV_ROBOTARM_POINT) {
        return MAV_MISSION_UNSUPPORTED;
    }
    if (cmd.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
        cmd.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        return MAV_MISSION_UNSUPPORTED_FRAME;
    }
    if (!abs((int8_t)cmd.param1)<1) {
        return MAV_MISSION_INVALID_PARAM5_X;
    }
    if (!abs((int8_t)cmd.param2)<1) {
        return MAV_MISSION_INVALID_PARAM6_Y;
    }
    if (!abs((int8_t)cmd.param3)<1) {
        return MAV_MISSION_INVALID_PARAM7;
    }
    ret = {};
    ret.xhorizontal = cmd.param1;
    ret.yvertical   = cmd.param2;
    ret.zalt        = cmd.param3;
    return MAV_MISSION_ACCEPTED;
}

/*
  static function to get robot arm waypoint item as mavlink_mission_item_int_t
 */
bool MissionItemProtocol_RobotArmWP::get_item_as_mission_item(uint16_t seq,
                                                         mavlink_mission_item_int_t &ret_packet)
{
    auto *robotarmwpp = AE::robotarmwp();

    RobotArmLocation robotarmloc;

    if (robotarmwpp == nullptr || !robotarmwpp->get_rbtarm_waypoint_with_index(seq, robotarmloc)) {
        return false;
    }

    ret_packet.frame = MAV_FRAME_BODY_FRD;
    ret_packet.command = MAV_CMD_NAV_ROBOTARM_POINT;
    ret_packet.param1 = robotarmloc.xhorizontal;
    ret_packet.param2 = robotarmloc.yvertical;
    ret_packet.param3 = robotarmloc.zalt;

    return true;
}

MAV_MISSION_RESULT MissionItemProtocol_RobotArmWP::get_item(const GCS_MAVLINK &_link,
                                                       const mavlink_message_t &msg,
                                                       const mavlink_mission_request_int_t &packet,
                                                       mavlink_mission_item_int_t &ret_packet)
{
    if (!get_item_as_mission_item(packet.seq, ret_packet)) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }
    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_RobotArmWP::item_count() const {
    return robotarmwp.get_rbtarm_waypoint_total();
}

uint16_t MissionItemProtocol_RobotArmWP::max_items() const {
    return robotarmwp.get_rbtarm_waypoint_max();
}

MAV_MISSION_RESULT MissionItemProtocol_RobotArmWP::replace_item(const mavlink_mission_item_int_t &cmd)
{
    RobotArmLocation robotarmloc;
    const MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_RobotArmLocation(cmd, robotarmloc);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    if (!robotarmwp.set_rbtarm_waypoint_with_index(cmd.seq, robotarmloc)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_RobotArmWP::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Rally upload timeout");
}

void MissionItemProtocol_RobotArmWP::truncate(const mavlink_mission_count_t &packet)
{
    robotarmwp.truncate(packet.count);
}
