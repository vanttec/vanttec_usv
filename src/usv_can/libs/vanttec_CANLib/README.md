# Vanttec CANLib
<img src="https://vanttec.com/static/media/LogoAzul_Negro.532425998e530ea873e1.png" width="300">
Multi-platform CAN communications library.

## Description

Library with helper functions to serialize / deserialize primitive data types into the Vanttec CAN Message protocol. 
Message can have variable length, with a minimum of 2 bytes for the base protocol (ID and size) and a max data length of 6 bytes (CAN message limit).
| Bytes | 0 | 1 | 2...7 |
| --- | --- | --- | --- |
| | Message ID Byte | Message Len | Message Data |

## Getting Started

### Prerequisites
```
No dependencies are needed
```

### Usage

* Use `src/Vanttec_CANLib/` for a platform agnostic implementation.
* Use `src/Vanttec_CANLib_Linux/` for a Linux SocketCAN implementation.


## Version History
###TODO
