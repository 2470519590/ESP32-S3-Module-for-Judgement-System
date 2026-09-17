#!/usr/bin/env python3
"""Scan nearby BLE devices on Windows and suggest an address for ``assign``.

Requires the ``bleak`` package.  This scanner intentionally uses BLE discovery:
it cannot list Bluetooth Classic-only devices that do not advertise over BLE.
"""

from __future__ import annotations

import argparse
import asyncio
import re
import sys
from dataclasses import dataclass, field
from typing import Any

try:
    from bleak import BleakScanner
except ImportError:
    BleakScanner = None  # type: ignore[assignment,misc]


NAME_KEYWORDS = (
    "xbox",
    "controller",
    "gamepad",
    "wireless controller",
    "dualsense",
    "dualshock",
    "8bitdo",
    "playstation",
    "joy-con",
    "joycon",
)

# Bluetooth SIG Human Interface Device service.  It is a useful extra signal,
# but is deliberately not sufficient by itself because keyboards/mice use it too.
HID_SERVICE_UUID = "00001812-0000-1000-8000-00805f9b34fb"
MICROSOFT_COMPANY_ID = 0x0006
MAC_RE = re.compile(r"^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$")


class DependencyMissingError(Exception):
    """Raised when the optional BLE scanning dependency is unavailable."""


@dataclass
class DeviceRecord:
    address: str
    name: str
    rssi: int | None
    service_uuids: tuple[str, ...] = field(default_factory=tuple)
    manufacturer_ids: tuple[int, ...] = field(default_factory=tuple)

    @property
    def evidence(self) -> str:
        parts: list[str] = []
        lowered_name = self.name.lower()
        keywords = [word for word in NAME_KEYWORDS if word in lowered_name]
        if keywords:
            parts.append("name:" + keywords[0])
        if HID_SERVICE_UUID in self.service_uuids:
            parts.append("svc:HID(1812)")
        if MICROSOFT_COMPANY_ID in self.manufacturer_ids:
            parts.append("mfg:Microsoft(0006)")
        if not parts and self.service_uuids:
            parts.append("svc:" + self.service_uuids[0])
        if not parts and self.manufacturer_ids:
            parts.append("mfg:" + ",".join(f"{item:04X}" for item in self.manufacturer_ids))
        return "; ".join(parts) if parts else "no advertised name/service data"

    @property
    def is_gamepad(self) -> bool:
        name_match = any(word in self.name.lower() for word in NAME_KEYWORDS)
        # A Microsoft manufacturer block plus the HID service is a strong
        # controller signal even when the device has no local name.
        microsoft_hid = (
            MICROSOFT_COMPANY_ID in self.manufacturer_ids
            and HID_SERVICE_UUID in self.service_uuids
        )
        return name_match or microsoft_hid


def normalise_address(address: str) -> str:
    """Keep Windows MAC addresses readable while still deduplicating safely."""
    return address.upper()


def format_rssi(rssi: int | None) -> str:
    return f"{rssi:d}" if rssi is not None else "N/A"


def clip(value: str, width: int) -> str:
    return value if len(value) <= width else value[: width - 3] + "..."


def print_table(records: list[DeviceRecord]) -> None:
    headers = ("Rank", "Type", "Name", "MAC address", "RSSI", "Evidence / advertised UUID")
    widths = (5, 7, 24, 17, 7, 52)
    line = "  ".join("-" * width for width in widths)
    print("  ".join(f"{header:<{width}}" for header, width in zip(headers, widths)))
    print(line)
    for rank, record in enumerate(records, start=1):
        row = (
            str(rank),
            "GAMEPAD" if record.is_gamepad else "OTHER",
            record.name,
            record.address,
            format_rssi(record.rssi),
            record.evidence,
        )
        print("  ".join(f"{clip(value, width):<{width}}" for value, width in zip(row, widths)))


async def scan(seconds: float) -> list[DeviceRecord]:
    if BleakScanner is None:
        raise DependencyMissingError(
            "Missing dependency: install Bleak with:\n"
            "  conda run -n py310 python -m pip install bleak"
        )

    strongest: dict[str, DeviceRecord] = {}

    def detected(device: Any, advertisement: Any) -> None:
        address = normalise_address(str(device.address))
        name = advertisement.local_name or device.name or "<unknown>"
        rssi = getattr(advertisement, "rssi", None)
        service_uuids = tuple(sorted(str(uuid).lower() for uuid in (advertisement.service_uuids or [])))
        manufacturer_data = getattr(advertisement, "manufacturer_data", {}) or {}
        manufacturer_ids = tuple(sorted(int(company_id) for company_id in manufacturer_data))
        candidate = DeviceRecord(address, name, rssi, service_uuids, manufacturer_ids)
        previous = strongest.get(address)
        # Keep the strongest received advertisement for each address.  A first
        # record without RSSI is retained only until a measured record arrives.
        if previous is None or rssi is not None and (previous.rssi is None or rssi > previous.rssi):
            strongest[address] = candidate

    scanner = BleakScanner(detection_callback=detected)
    try:
        await scanner.start()
        await asyncio.sleep(seconds)
    finally:
        await scanner.stop()

    return sorted(
        strongest.values(),
        key=lambda record: (not record.is_gamepad, -(record.rssi if record.rssi is not None else -999)),
    )


def describe_scan_error(error: Exception) -> str:
    message = str(error).strip()
    lower = message.lower()
    if any(word in lower for word in ("bluetooth", "radio", "adapter", "powered off", "not available")):
        return "Bluetooth scan failed. Check that Windows Bluetooth is enabled and the adapter is available."
    if any(word in lower for word in ("access", "permission", "denied")):
        return "Bluetooth scan failed because Windows denied access. Enable Bluetooth permissions and try again."
    return "Bluetooth scan failed" + (f": {message}" if message else ".")


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan nearby BLE devices for an ESP32 gamepad address.")
    parser.add_argument("--seconds", type=float, default=8.0, help="scan duration in seconds (default: 8)")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be greater than zero")

    print(f"Scanning for nearby BLE devices for {args.seconds:g} seconds...")
    try:
        records = asyncio.run(scan(args.seconds))
    except KeyboardInterrupt:
        print("\nScan cancelled.")
        return 130
    except DependencyMissingError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    except Exception as error:  # Windows adapter/API errors vary by driver version.
        print(f"ERROR: {describe_scan_error(error)}", file=sys.stderr)
        return 1

    if records:
        print_table(records)
        best = records[0]
        if not MAC_RE.match(best.address):
            print("\nNo Windows MAC-format address was available for the CLI example.")
        else:
            print("\nUse this controller MAC after the target ESP32 is the only unassigned board online:")
            print(f"  assign ROBOT_ID {best.address}")
    else:
        print("No BLE devices found. Make sure the target controller is powered on and advertising.")
        print("\nExample: assign ROBOT_ID AA:BB:CC:DD:EE:FF")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
