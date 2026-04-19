#!/usr/bin/env python3

import ast
import yaml
from typing import Dict, Any, Optional

import rospy
from std_msgs.msg import String, Int32


class TplinkParser:
    """
    Parser cho dữ liệu TP-Link từ topic `ddb/tplink/log`.

    Node này:
    - subscribe dữ liệu trạng thái Wi-Fi / TP-Link từ `ddb/tplink/log`
    - trích xuất RSSI
    - ánh xạ AP/BSSID hiện tại sang robot tương ứng
    - publish RSSI lên `ddb/tplink/rssi/<robot>`
    """

    def __init__(self, this_robot: str, robot_configs: Dict[str, Any], radio_configs: Dict[str, Any]):
        assert isinstance(this_robot, str)
        assert isinstance(robot_configs, dict)
        assert isinstance(radio_configs, dict)

        self.this_robot = this_robot
        self.robot_cfg = robot_configs
        self.radio_cfg = radio_configs

        self.no_rssi = -1

        rospy.loginfo(f"{self.this_robot} - TP-Link Parser - Starting")

        # Map radio -> robot
        # Ví dụ: radio 'tplink_a' đang được robot 'ugv1' dùng
        self.radio_to_robot: Dict[str, str] = {}
        for robot, cfg in self.robot_cfg.items():
            radio_name = cfg.get("using-radio")
            if radio_name:
                self.radio_to_robot[radio_name] = robot

        # Publisher cho từng robot khác
        self.robot_publishers: Dict[str, rospy.Publisher] = {}
        for robot in self.robot_cfg.keys():
            if robot == self.this_robot:
                continue
            topic = f"ddb/tplink/rssi/{robot}"
            self.robot_publishers[robot] = rospy.Publisher(topic, Int32, queue_size=10)

        rospy.Subscriber("ddb/tplink/log", String, self.update_dict)

    def normalize_mac(self, mac: Optional[str]) -> Optional[str]:
        if mac is None:
            return None
        return mac.strip().lower()

    def find_robot_from_bssid(self, bssid: Optional[str]) -> Optional[str]:
        """
        Tìm robot từ BSSID/MAC AP hiện tại.
        So sánh với `radio_configs[radio]['MAC-address']`.
        """
        bssid = self.normalize_mac(bssid)
        if not bssid:
            return None

        for radio_name, radio_data in self.radio_cfg.items():
            mac_list = radio_data.get("MAC-address", [])
            for mac in mac_list:
                if self.normalize_mac(mac) == bssid:
                    return self.radio_to_robot.get(radio_name)
        return None

    def find_robot_from_ssid(self, ssid: Optional[str]) -> Optional[str]:
        """
        Fallback nếu không match được bằng MAC.

        Mặc định thử:
        - ssid == tên radio trong config
        - hoặc radio có field 'ssid'
        """
        if not ssid:
            return None

        ssid = ssid.strip()

        for radio_name, radio_data in self.radio_cfg.items():
            configured_ssid = radio_data.get("ssid")
            if configured_ssid and configured_ssid == ssid:
                return self.radio_to_robot.get(radio_name)

            if radio_name == ssid:
                return self.radio_to_robot.get(radio_name)

        return None

    def resolve_target_robot(self, data_dict: Dict[str, Any]) -> Optional[str]:
        """
        Xác định robot mà RSSI này đang đại diện.
        Ưu tiên BSSID trước, sau đó mới fallback sang SSID.
        """
        bssid = data_dict.get("bssid")
        ssid = data_dict.get("ssid")

        robot = self.find_robot_from_bssid(bssid)
        if robot:
            return robot

        robot = self.find_robot_from_ssid(ssid)
        if robot:
            return robot

        return None

    def update_dict(self, msg: String):
        """
        Callback xử lý log TP-Link mới từ topic `ddb/tplink/log`.
        """
        try:
            data_dict = ast.literal_eval(msg.data)
        except Exception as e:
            rospy.logerr(f"{self.this_robot} - TP-Link Parser - invalid message format: {e}")
            return

        if not isinstance(data_dict, dict):
            rospy.logerr(f"{self.this_robot} - TP-Link Parser - incoming data is not a dictionary")
            return

        if "error" in data_dict:
            rospy.logwarn_throttle(5.0, f"{self.this_robot} - TP-Link Parser - source node error: {data_dict['error']}")
            return

        connected = data_dict.get("connected", False)
        rssi = data_dict.get("rssi_dbm", None)

        target_robot = self.resolve_target_robot(data_dict)

        if target_robot is None:
            rospy.logwarn_throttle(
                5.0,
                f"{self.this_robot} - TP-Link Parser - could not map incoming TP-Link log to any robot "
                f"(ssid={data_dict.get('ssid')}, bssid={data_dict.get('bssid')})"
            )
            return

        if target_robot == self.this_robot:
            # Không publish RSSI của chính robot này lên topic theo dõi peer
            return

        if target_robot not in self.robot_publishers:
            rospy.logwarn(f"{self.this_robot} - TP-Link Parser - no publisher for robot {target_robot}")
            return

        if connected and rssi is not None:
            self.robot_publishers[target_robot].publish(int(rssi))
        else:
            # Mất kết nối hoặc không có RSSI
            self.robot_publishers[target_robot].publish(self.no_rssi)


def main():
    rospy.init_node("tplink_listener", anonymous=False)

    robot_name = rospy.get_param("~robot_name", "charon")

    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    if robot_name not in robot_configs.keys():
        rospy.signal_shutdown("Robot not in config file")
        rospy.spin()

    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)

    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.signal_shutdown("Radio not in config file")
        rospy.spin()

    TplinkParser(robot_name, robot_configs, radio_configs)
    rospy.spin()


if __name__ == "__main__":
    main()