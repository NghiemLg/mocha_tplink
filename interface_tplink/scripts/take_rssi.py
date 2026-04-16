#!/usr/bin/env python3
"""
Python 3 script to get RSSI (signal strength) and SNR of the connected Wi-Fi Access Point (AP/router).
Works on Linux (most common for this). Zero external dependencies — only stdlib + subprocess.
Tested patterns for both modern (iw) and classic (iwconfig) tools.
"""

import subprocess
import re
import os
from typing import Dict, Optional


def get_wifi_rssi_snr() -> Dict:
    """
    Returns RSSI (dBm), Noise (dBm), SNR (dB) and AP info for the currently connected Wi-Fi.
    Automatically detects the first wireless interface.
    """
    # ── 1. Auto-detect wireless interface (wlan*, wl*, wlp*, etc.)
    try:
        ifaces = [
            iface for iface in os.listdir("/sys/class/net")
            if any(k in iface.lower() for k in ("wlan", "wl", "wifi", "wlp"))
        ]
        if not ifaces:
            return {"error": "No wireless interface found on this system."}
        interface = ifaces[0]          # take the first one (usually the active one)
    except Exception:
        interface = "wlan0"            # fallback

    # ── 2. Try iwconfig first (gives both RSSI + Noise → SNR)
    try:
        output = subprocess.check_output(
            ["iwconfig", interface],
            text=True,
            stderr=subprocess.DEVNULL
        )

        # Parse AP name (SSID)
        ssid_match = re.search(r'ESSID:"(.*?)"', output)
        ssid = ssid_match.group(1) if ssid_match else "Unknown AP"

        # Parse RSSI (Signal level)
        rssi_match = re.search(r'Signal level\s*=\s*(-?\d+)\s*dBm', output, re.IGNORECASE)
        rssi = int(rssi_match.group(1)) if rssi_match else None

        # Parse Noise level
        noise_match = re.search(r'Noise level\s*=\s*(-?\d+)\s*dBm', output, re.IGNORECASE)
        noise = int(noise_match.group(1)) if noise_match else None

        snr = rssi - noise if rssi is not None and noise is not None else None

        return {
            "interface": interface,
            "ssid": ssid,               # name of the Access Point
            "rssi_dbm": rssi,
            "noise_dbm": noise,
            "snr_db": snr,
            "method": "iwconfig",
            "raw": output.strip()
        }

    except (subprocess.CalledProcessError, FileNotFoundError):
        # ── 3. Fallback to modern "iw" tool (only RSSI, no Noise/SNR)
        try:
            output = subprocess.check_output(
                ["iw", "dev", interface, "link"],
                text=True,
                stderr=subprocess.DEVNULL
            )

            ssid_match = re.search(r'SSID:\s*(.+)', output)
            ssid = ssid_match.group(1).strip() if ssid_match else "Unknown AP"

            rssi_match = re.search(r'signal:\s*(-?\d+)\s*dBm', output)
            rssi = int(rssi_match.group(1)) if rssi_match else None

            return {
                "interface": interface,
                "ssid": ssid,
                "rssi_dbm": rssi,
                "noise_dbm": None,
                "snr_db": None,          # iw link does not expose noise
                "method": "iw",
                "raw": output.strip()
            }
        except Exception as e:
            return {"error": f"Could not read Wi-Fi info: {e}"}

    except Exception as e:
        return {"error": str(e)}


if __name__ == "__main__":
    info = get_wifi_rssi_snr()

    if "error" in info:
        print(" Error:", info["error"])
        print("\nTips:")
        print("   • Install tools if missing:  sudo apt install wireless-tools iw")
        print("   • Make sure Wi-Fi is connected")
        print("   • Run as normal user (no sudo needed)")
    else:
        print(" Connected Access Point (AP)")
        print(f"   Interface : {info['interface']}")
        print(f"   AP Name   : {info['ssid']}")
        print(f"   RSSI      : {info['rssi_dbm']} dBm")
        if info["snr_db"] is not None:
            print(f"   Noise     : {info['noise_dbm']} dBm")
            print(f"   SNR       : {info['snr_db']} dB")
        else:
            print("   SNR       : Not available with current tool (iwconfig gives best SNR)")
        print("\nRaw output for debugging:")
        print(info["raw"])