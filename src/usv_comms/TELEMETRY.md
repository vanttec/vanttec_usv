# STM32 telemetry over the SiK radio

## Jetson data path

`usv_can` receives Classical CAN through Linux SocketCAN (default interface
`can0`; the VTEC launch defaults to `can_vtec`) at the configured 125 kbit/s.
The vendored `CANHandler` is the Linux SocketCAN adapter; the STM32 firmware's
HAL API is not linked into the Jetson node.

The CAN receiver publishes these dashboard values:

| ROS topic | Type | Source |
|---|---|---|
| `/usv/telemetry/heartbeat` | `std_msgs/UInt32` | `0x010`, STM32 uptime in ms |
| `/usv/telemetry/op_mode` | `std_msgs/UInt8` | `0x010` byte 4: 0 auto, 1 manual, 2 disabled |
| `/usv/telemetry/auxiliary_selector` | `std_msgs/UInt8` | `0x010` byte 5 |
| `/usv/telemetry/armed` | `std_msgs/UInt8` | `0x010` byte 6 |
| `/usv/telemetry/battery_voltage` | `std_msgs/Float32` | `0x108`, local STM32 battery monitor, little-endian float32 |

The separate `/usv/battery` publication remains associated with direct CAN ID
`0x200` reception. The handoff says that frame is not retransmitted by the
STM32, so it is only available if the Jetson is physically on the shared bus.
The `0x01E` Jetson-to-STM32 watchdog and `0x112` ping marker are not dashboard
heartbeat values. No motor or actuator command was added to this telemetry path.

## SiK radio payload

`sik_boat_node` selects the enabled `boat_to_station` topics in
`topics_config.yaml`; the corresponding station node must use the same topic
ordering and codec. Radio serial settings remain `/dev/ttyHOLY` at 115200 baud.
Each frame is `FE | message_id:u8 | payload_length:u16_be | payload | FD`.
The payload contains a count, per-value type markers, and encoded values.
Marker 0 is IEEE float32 big-endian, marker 1 is a UTF-8 string with a u16
length, marker 2 is unsigned int32 big-endian, and marker 3 is signed int32
big-endian. The integer markers preserve the STM32 uptime value without
float32 precision loss.

The root-level `receiver.py` decodes this SiK framing and writes telemetry to
`usv_data` with tag `vessel=usv_1`; raw `heartbeat` remains an integer uptime
counter. Its message-ID table follows the enabled `boat_to_station` entries in
`topics_config.yaml`, so keep that table synchronized if the config ordering
changes. The serial format currently has no sequence number or checksum, and
the radio-to-Influx path has not been validated on hardware.
