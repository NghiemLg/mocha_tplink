#!/usr/bin/env python3
import os
import re
import subprocess
from typing import Dict, Any, Optional

import rospy
from std_msgs.msg import String


def find_wifi_interface(preferred_interface: Optional[str] = None) -> Optional[str]:
    """
    Tìm interface Wi-Fi.
    Nếu user truyền preferred_interface và interface đó tồn tại thì dùng luôn.
    Nếu không, tự tìm interface bắt đầu bằng 'wl'.
    """
    if preferred_interface and os.path.exists(f"/sys/class/net/{preferred_interface}"):
        return preferred_interface

    try:
        for iface in os.listdir("/sys/class/net"):
            if iface.startswith("wl"):
                return iface
    except Exception:
        return None

    return None


def run_command(cmd: list) -> str:
    """
    Chạy command và trả về stdout dạng string.
    Raise exception nếu command fail.
    """
    return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)


def parse_iw_link(output: str) -> Dict[str, Any]:
    """
    Parse output của:
        iw dev <iface> link

    Ví dụ output khi connected:
        Connected to xx:xx:xx:xx:xx:xx (on wlp0s20f3)
            SSID: ivsr
            freq: 2412
            RX: ...
            TX: ...
            signal: -47 dBm
            tx bitrate: 72.2 MBit/s

    Nếu chưa connect thường có:
        Not connected.
    """
    result: Dict[str, Any] = {
        "connected": False,
        "bssid": None,
        "ssid": None,
        "freq_mhz": None,
        "rssi_dbm": None,
        "tx_bitrate": None,
    }

    if "Not connected" in output:
        return result

    connected_match = re.search(r"Connected to\s+([0-9a-fA-F:]{17})", output)
    ssid_match = re.search(r"SSID:\s*(.+)", output)
    freq_match = re.search(r"freq:\s*(\d+)", output)
    signal_match = re.search(r"signal:\s*(-?\d+)\s*dBm", output)
    bitrate_match = re.search(r"tx bitrate:\s*(.+)", output)

    result["connected"] = True
    result["bssid"] = connected_match.group(1) if connected_match else None
    result["ssid"] = ssid_match.group(1).strip() if ssid_match else None
    result["freq_mhz"] = int(freq_match.group(1)) if freq_match else None
    result["rssi_dbm"] = int(signal_match.group(1)) if signal_match else None
    result["tx_bitrate"] = bitrate_match.group(1).strip() if bitrate_match else None

    return result


def parse_iw_info(output: str) -> Dict[str, Any]:
    """
    Parse output của:
        iw dev <iface> info

    Lấy thêm txpower nếu có.
    """
    result: Dict[str, Any] = {
        "tx_power_dbm": None,
    }

    txpower_match = re.search(r"txpower\s+([0-9.]+)\s+dBm", output)
    if txpower_match:
        result["tx_power_dbm"] = float(txpower_match.group(1))

    return result


def get_tplink_status(interface: Optional[str] = None) -> Dict[str, Any]:
    """
    Hàm lõi để chương trình khác có thể reuse.

    Trả về dictionary:
    {
        "interface": ...,
        "connected": ...,
        "ssid": ...,
        "bssid": ...,
        "rssi_dbm": ...,
        ...
    }

    hoặc:
    {
        "error": "..."
    }
    """
    iface = find_wifi_interface(interface)
    if not iface:
        return {"error": "No Wi-Fi interface found"}

    try:
        link_out = run_command(["iw", "dev", iface, "link"])
        info_out = run_command(["iw", "dev", iface, "info"])

        link_data = parse_iw_link(link_out)
        info_data = parse_iw_info(info_out)

        result: Dict[str, Any] = {
            "interface": iface,
            "device_type": "tplink",
            "source": "iw",
            "raw_link": link_out.strip(),
            "timestamp": rospy.Time.now().to_sec(),
        }
        result.update(link_data)
        result.update(info_data)

        return result

    except FileNotFoundError:
        return {"error": "Command 'iw' not found. Install it with: sudo apt install iw"}
    except subprocess.CalledProcessError as e:
        return {"error": f"iw command failed: {e.output.strip()}"}
    except Exception as e:
        return {"error": str(e)}


def main():
    rospy.init_node("tplink_query", anonymous=False)

    publish_topic = rospy.get_param("~publish_topic", "ddb/tplink/log")
    preferred_interface = rospy.get_param("~interface", "")
    rate_hz = rospy.get_param("~rate", 1.0)

    pub = rospy.Publisher(publish_topic, String, queue_size=10)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("tplink_query started")
    rospy.loginfo(f"Publishing TP-Link status to: {publish_topic}")

    while not rospy.is_shutdown():
        status = get_tplink_status(preferred_interface if preferred_interface else None)

        if "error" in status:
            rospy.logwarn_throttle(5.0, f"tplink_query: {status['error']}")
        else:
            pub.publish(str(status))

        rate.sleep()


if __name__ == "__main__":
    main()