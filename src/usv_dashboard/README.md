# USV Dashboard

A lightweight web dashboard for monitoring the VANTTEC USV system status in real time.
Built with **Svelte + Vite** and communicates with ROS2 via **rosbridge WebSocket**.

---

## Architecture

```
Jetson (boat)                          Laptop (operator)
┌──────────────────────────┐           ┌──────────────────────────┐
│  ROS2 nodes              │           │  Browser                 │
│                          │           │                          │
│  /usv/status ────────────┼─────────► │  Dashboard               │
│  /usv/state/pose ────────┤ rosbridge │  ws://JETSON_IP:9090     │
│  (all other topics)      │ WebSocket │                          │
└──────────────────────────┘           └──────────────────────────┘
```

The Jetson runs `rosbridge_websocket`, which exposes all ROS2 topics over a WebSocket.
The dashboard runs entirely in the browser on the operator laptop — zero load on the Jetson beyond the bridge.

---

## Jetson Setup (one-time)

### 1. Install rosbridge

```bash
sudo apt install ros-humble-rosbridge-suite
```

### 2. Install image_transport plugins (for camera feed compression)

```bash
sudo apt install ros-humble-image-transport ros-humble-image-transport-plugins
```

`image-transport-plugins` provides the JPEG codec used by the `republish` node in `vision.launch.py`. Without it, the republish node will fail to start (but won't affect any other nodes).

### 3. Install the system status nodes dependencies

These should already be present if the workspace builds, but verify:

```bash
sudo apt install ros-humble-sbg-driver
```

### 3. Build the workspace

```bash
cd ~/vanttec_usv
colcon build --packages-select usv_interfaces usv_utils
source install/setup.bash
```

---

## Running on the Jetson

Launch rosbridge alongside your boat nodes. Add this to your launch file, or run it manually:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/vanttec_usv/install/setup.bash

# Start rosbridge (default port 9090)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Which status node to run

| Node | Use case |
|------|----------|
| `system_validation_node` | Hardware health + sensors + object list |
| `system_validation_node_rb` | All of the above + mission state machine |

```bash
# Standard node
ros2 run usv_utils system_validation_node

# OR mission-aware node (run this when missions are active)
ros2 run usv_utils system_validation_node_rb
```

Both publish to `/usv/status` on the same `usv_interfaces/msg/SystemStatus` message.

---

## Laptop Setup (one-time)

Requires **Node.js 18+**. Install via:

```bash
# Check version
node --version

# If not installed or outdated:
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

Then install dashboard dependencies:

```bash
cd src/usv_dashboard
npm install
```

---

## Running the Dashboard

```bash
cd src/usv_dashboard
npm run dev
```

Open your browser at `http://localhost:5173`.

In the top bar, enter the Jetson's IP and port:
```
ws://JETSON_IP:9090
```
Then click **Connect**. The status dot turns green when the WebSocket connects.

### Finding the Jetson's IP

Both the laptop and Jetson must be on the same network (competition field LAN or direct ethernet).

#### Method 1: On the Jetson itself (easiest)

SSH into the Jetson or connect a monitor/keyboard and run:

```bash
hostname -I
```

Or for more details:
```bash
ip a
# Look for the wlan0 or eth0 interface and find the inet address
```

#### Method 2: From your base station (scan the network)

If both devices are on the same WiFi network:

```bash
# Find devices on your network
nmap -sn 10.177.52.0/24
# (adjust the IP range based on your network)

# Or use arp-scan
sudo arp-scan --localnet
```

Look for a device named something like "jetson" or "nvidia".

#### Method 3: Check your router's DHCP client list

Access your WiFi router's admin panel (usually `192.168.1.1` or `10.0.0.1`) and look at connected devices - the Jetson should show up there.

#### Method 4: Set a static IP (recommended for competitions!)

Configure the Jetson to always use the same IP on the WiFi network. This way you always know it'll be at a fixed address (e.g., `192.168.1.100`).

On the Jetson, edit the network configuration or use NetworkManager:
```bash
# Using nmcli
sudo nmcli con mod <connection-name> ipv4.addresses 192.168.1.100/24
sudo nmcli con mod <connection-name> ipv4.method manual
```

#### Method 5: Use mDNS (if configured)

Some Jetson setups support mDNS, so you might be able to use:
```
ws://jetson.local:9090
```

instead of the IP address. Try this first - if it doesn't work, use the IP address.

**For competition/field use**, setting a **static IP** is strongly recommended so you don't have to hunt for it every time!

---

## Dashboard Panels

| Panel | Data source | Description |
|-------|-------------|-------------|
| Hardware | `/usv/status` | LED indicators for Camera, Lidar, CAN/STM |
| Operation Mode | `/usv/status` | AUTO / TELEOP / INACTIVE badge |
| Localization | `/usv/status` | EKF mode, GPS solution type, HDT status |
| Pose | `/usv/state/pose` | Live X, Y, heading (degrees) |
| Mission | `/usv/status` | Mission ID, state, running/complete |
| Objects | `/usv/status` | Table of detected objects with color, type, X/Y, UUID |

### Status Indicators

**EKF Status (`ekf_status`)**
| Value | Label | Color |
|-------|-------|-------|
| 0 | UNINITIALIZED | Red |
| 1 | VERTICAL_GYRO | Red |
| 2 | AHRS | Yellow |
| 3 | NAV_VELOCITY | Yellow |
| 4 | NAV_POSITION | Green |

**GPS Position Type (`gps_pos_status`)**
| Value | Label | Color |
|-------|-------|-------|
| 0 | NO_SOLUTION | Red |
| 1-2 | SINGLE | Yellow |
| 3-5 | PSRDIFF / SBAS / OMNISTAR | Yellow |
| 6 | RTK_FLOAT | Green |
| 7-10 | RTK_INT / PPP / FIXED | Green |

**Operation Mode (`op_mode`)**
| Value | Label | Color |
|-------|-------|-------|
| 0 | AUTO | Green |
| 1 | TELEOP | Yellow |
| 2 | INACTIVE | Red |

---

## Production Build

If you want to serve the dashboard as a static site (e.g. from the Jetson itself):

```bash
cd src/usv_dashboard
npm run build
# Output is in dist/ — open dist/index.html directly or serve with any static server
```

---

## Camera Feed Panel

The dashboard includes a camera panel that is **disabled by default**. Tick the **Enable** checkbox to start subscribing to the compressed image stream.

When disabled: zero topic subscriptions, zero bandwidth, zero CPU cost on the Jetson.
When enabled: subscribes to `/bebblebrox/video/compressed` (`sensor_msgs/CompressedImage`) — JPEG frames delivered as base64 over the WebSocket. A live FPS counter is shown in the panel header.

The compressed topic is created automatically by the `image_republish` node added to `vision.launch.py`. It remaps:
- input:  `/bebblebrox/video/image` (raw from beeblebrox)
- output: `/bebblebrox/video/compressed` (JPEG, consumed by the dashboard)

The republish node is idle when no subscriber is connected — it only encodes frames when the dashboard camera panel is enabled.

---

## rosshow (terminal viewer)

rosshow is included as a git submodule at `src/deps/rosshow` for quick terminal-based topic inspection on the Jetson.

### Setup (on the Jetson)

```bash
cd src/deps/rosshow
pip3 install -r requirements.txt
```

### Usage

```bash
# View camera feed in terminal
python3 rosshow.py /bebblebrox/video

# View any other topic
python3 rosshow.py /usv/state/pose
python3 rosshow.py /usv/status
```

rosshow renders topics as ASCII/Unicode in the terminal — useful for quickly checking sensor output over SSH without a display.

---

## Topics Subscribed

| Topic | Message Type | Node |
|-------|-------------|------|
| `/usv/status` | `usv_interfaces/msg/SystemStatus` | `system_validation_node` or `_rb` |
| `/usv/state/pose` | `geometry_msgs/msg/Pose2D` | `usv_localization` |
| `/bebblebrox/video/compressed` | `sensor_msgs/msg/CompressedImage` | `visionsystemx` (only when camera panel enabled) |

---

## SystemStatus Message Fields

```
uint8  ekf_status         # SBG EKF solution mode (0–4)
uint8  gps_pos_status     # GPS position type (0–10)
uint16 gps_hdt_status     # GPS heading status
bool   can_stm_status     # CAN/STM32 heartbeat (500 ms timeout)
bool   camera_status      # ZED camera frames (500 ms timeout)
bool   lidar_status       # VLP-16 pointcloud (500 ms timeout)
uint16 op_mode            # 0=AUTO, 1=TELEOP, 2=INACTIVE
int8   mission_id         # Active mission (rb node only)
int8   mission_state      # Mission state machine (rb node only)
int8   mission_status     # 0=running, 1=complete (rb node only)
Object[] obj_list         # Detected objects from /bebblebrox/objects/yolo
```

Each `Object` in `obj_list`:
```
float64 x, y      # World frame position (meters)
float64 v_x, v_y  # Velocity
int8    color     # 0=red, 1=green, 2=blue, 3=yellow, 4=black
string  type
string  uuid
```
