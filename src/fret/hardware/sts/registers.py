"""STS3215 register addresses and encoding bits.

Source (cited, do not trust numbers from memory):

* LeRobot Feetech tables (Apache-2.0), commit ``e99c55af``:
  https://github.com/huggingface/lerobot/blob/e99c55af/src/lerobot/motors/feetech/tables.py
* Feetech STS/SMS memory table:
  http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0

STS3215 model number is ``777`` (LeRobot ``MODEL_NUMBER_TABLE``).
"""

from __future__ import annotations

from typing import Final

# (address, size_bytes) — STS/SMS series control table (LeRobot).
FIRMWARE_MAJOR_VERSION: Final[tuple[int, int]] = (0, 1)
FIRMWARE_MINOR_VERSION: Final[tuple[int, int]] = (1, 1)
MODEL_NUMBER: Final[tuple[int, int]] = (3, 2)

ID: Final[tuple[int, int]] = (5, 1)
BAUD_RATE: Final[tuple[int, int]] = (6, 1)
RETURN_DELAY_TIME: Final[tuple[int, int]] = (7, 1)
MIN_POSITION_LIMIT: Final[tuple[int, int]] = (9, 2)
MAX_POSITION_LIMIT: Final[tuple[int, int]] = (11, 2)
MAX_TEMPERATURE_LIMIT: Final[tuple[int, int]] = (13, 1)
MAX_VOLTAGE_LIMIT: Final[tuple[int, int]] = (14, 1)
MIN_VOLTAGE_LIMIT: Final[tuple[int, int]] = (15, 1)
MAX_TORQUE_LIMIT: Final[tuple[int, int]] = (16, 2)
PHASE: Final[tuple[int, int]] = (18, 1)
P_COEFFICIENT: Final[tuple[int, int]] = (21, 1)
PROTECTION_CURRENT: Final[tuple[int, int]] = (28, 2)
HOMING_OFFSET: Final[tuple[int, int]] = (31, 2)
OPERATING_MODE: Final[tuple[int, int]] = (33, 1)

TORQUE_ENABLE: Final[tuple[int, int]] = (40, 1)
ACCELERATION: Final[tuple[int, int]] = (41, 1)
GOAL_POSITION: Final[tuple[int, int]] = (42, 2)
GOAL_TIME: Final[tuple[int, int]] = (44, 2)
GOAL_VELOCITY: Final[tuple[int, int]] = (46, 2)
TORQUE_LIMIT: Final[tuple[int, int]] = (48, 2)
LOCK: Final[tuple[int, int]] = (55, 1)

PRESENT_POSITION: Final[tuple[int, int]] = (56, 2)
PRESENT_VELOCITY: Final[tuple[int, int]] = (58, 2)
PRESENT_LOAD: Final[tuple[int, int]] = (60, 2)
PRESENT_VOLTAGE: Final[tuple[int, int]] = (62, 1)
PRESENT_TEMPERATURE: Final[tuple[int, int]] = (63, 1)
STATUS: Final[tuple[int, int]] = (65, 1)
MOVING: Final[tuple[int, int]] = (66, 1)
PRESENT_CURRENT: Final[tuple[int, int]] = (69, 2)

# Sign-magnitude bit index (LeRobot STS_SMS_SERIES_ENCODINGS_TABLE).
SIGN_BIT_PRESENT_LOAD: Final[int] = 10
SIGN_BIT_HOMING_OFFSET: Final[int] = 11
SIGN_BIT_POSITION: Final[int] = 15
SIGN_BIT_VELOCITY: Final[int] = 15

STS3215_MODEL_NUMBER: Final[int] = 777
STS3215_RESOLUTION: Final[int] = 4096
STS3215_MID_TICKS: Final[int] = 2048

BAUD_RATE_TABLE: Final[dict[int, int]] = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}

CONTROL_TABLE: Final[dict[str, tuple[int, int]]] = {
    "Firmware_Major_Version": FIRMWARE_MAJOR_VERSION,
    "Firmware_Minor_Version": FIRMWARE_MINOR_VERSION,
    "Model_Number": MODEL_NUMBER,
    "ID": ID,
    "Baud_Rate": BAUD_RATE,
    "Return_Delay_Time": RETURN_DELAY_TIME,
    "Min_Position_Limit": MIN_POSITION_LIMIT,
    "Max_Position_Limit": MAX_POSITION_LIMIT,
    "Max_Temperature_Limit": MAX_TEMPERATURE_LIMIT,
    "Max_Voltage_Limit": MAX_VOLTAGE_LIMIT,
    "Min_Voltage_Limit": MIN_VOLTAGE_LIMIT,
    "Max_Torque_Limit": MAX_TORQUE_LIMIT,
    "Phase": PHASE,
    "P_Coefficient": P_COEFFICIENT,
    "Protection_Current": PROTECTION_CURRENT,
    "Homing_Offset": HOMING_OFFSET,
    "Operating_Mode": OPERATING_MODE,
    "Torque_Enable": TORQUE_ENABLE,
    "Acceleration": ACCELERATION,
    "Goal_Position": GOAL_POSITION,
    "Goal_Time": GOAL_TIME,
    "Goal_Velocity": GOAL_VELOCITY,
    "Torque_Limit": TORQUE_LIMIT,
    "Lock": LOCK,
    "Present_Position": PRESENT_POSITION,
    "Present_Velocity": PRESENT_VELOCITY,
    "Present_Load": PRESENT_LOAD,
    "Present_Voltage": PRESENT_VOLTAGE,
    "Present_Temperature": PRESENT_TEMPERATURE,
    "Status": STATUS,
    "Moving": MOVING,
    "Present_Current": PRESENT_CURRENT,
}
