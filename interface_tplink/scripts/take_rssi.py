#!/usr/bin/env python3
import subprocess
import re
import os
from typing import Dict


def get_wifi_info() -> Dict[str, str]:
    """
    Trả về:
    {
        "interface": str,
        "ssid": str,
        "rssi_dbm": str
    }
    hoặc {"error": "..."}
    """

    try:
        # 1. tìm interface wifi (wl*)
        interface = next(
            iface for iface in os.listdir("/sys/class/net")
            if iface.startswith("wl")
        )
    except StopIteration:
        return {"error": "No Wi-Fi interface found"}

    try:
        # 2. gọi iw
        output = subprocess.check_output(
            ["iw", "dev", interface, "link"],
            text=True
        )

        # 3. parse
        ssid_match = re.search(r"SSID:\s*(.+)", output)
        rssi_match = re.search(r"signal:\s*(-?\d+)\s*dBm", output)

        if not ssid_match or not rssi_match:
            return {"error": "Not connected to any AP"}

        return {
            "interface": interface,
            "ssid": ssid_match.group(1).strip(),
            "rssi_dbm": rssi_match.group(1)
        }

    except Exception as e:
        return {"error": str(e)}


def main():
    info = get_wifi_info()

    if "error" in info:
        print("Error:", info["error"])
        return

    print(f"Interface : {info['interface']}")
    print(f"AP Name   : {info['ssid']}")
    print(f"RSSI      : {info['rssi_dbm']} dBm")


if __name__ == "__main__":
    main()