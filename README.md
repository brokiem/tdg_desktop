# TDG Direct Desktop — Low-Latency FPV Flight Controller & Receiver

A high-performance, low-latency Linux desktop application for controlling **WIFI_8K** toy drones and displaying real-time analog-style FPV video directly over Wi-Fi without intermediate hardware (no ESP32 or relay required).

```text
Laptop Gamepad ───SDL2───> UDP 8090 (Control Commands @ 25-200 Hz) ───> WIFI_8K Drone
WIFI_8K Camera ───UDP 8080───> JPEG Reassembly ───> Decoders ───> OpenGL / Dear ImGui
```

---

## Key Features

- **Ultra-Low Latency Video & Control**:
  - Unsynchronized OpenGL presentation capped at a precise **500 FPS** frame loop.
  - Direct UDP datagram reassembly with SOI/SOS/EOI validation and automatic resynchronization (< 50ms drop window).
  - Zero-queue single-slot mailbox (`take_latest()`) ensures zero frame buffer delay.
  - Controls sent dynamically on change (up to 200 Hz) with a 25 Hz keepalive heartbeat.

- **Analog FPV Presentation & OSD**:
  - **MAX7456-style Analog OSD Overlay**: Discrete H-sync horizontal jitter, RF luma flicker, black keying, and luma dispersion.
  - **Real System Telemetry**: Physical Wi-Fi RSSI signal strength from `/proc/net/wireless` with a 4-bar analog signal icon.
  - **Real-Time Clock & Metrics**: Live camera FPS, date/time timestamp, resolution, view modes (`NORM`, `WIDE`, `FULL`), and frame counters.
  - **Scanline Texture**: Optimized 1x4 alpha scanline filter for realistic interlaced structure without geometry overhead.
  - **OSD Font Scaling**: Adjust OSD text size dynamically (`-` / `+` buttons).

- **Asynchronous Video Recording**:
  - Record the preview area—including all video filters, reticles, OSD elements, and scanlines—at **30 FPS**.
  - Non-blocking `AsyncRecorder` pipeline pipes raw RGB framebuffers via GL readback to `ffmpeg` (`libx264` ultrafast preset).
  - Blinking red `REC` dot on OSD and live timer in UI.
  - Toggle via UI button, **D-Pad Down**, or the **'R'** key.

- **Multi-Threaded Image Enhancement**:
  - Edge-aware cross-filter denoising and contrast/vibrance tone curves run asynchronously on the decoder thread (`decode_loop()`), leaving the main UI/render thread completely unburdened.

- **True Time-Window FPS Metering**:
  - **CAM FPS**: True rate of newly decoded camera JPEG frames ($\Delta \text{stats.frames} / \Delta t$).
  - **DISPLAY FPS**: Unique frames presented to the UI ($ \text{consumed\_frames} / \Delta t $).
  - **RENDER FPS**: UI & OpenGL rendering loop rate (`ImGui::GetIO().Framerate`).

- **Interactive Virtual Gamepad & Controls**:
  - Mode 2 stick mapping (Left: Throttle/Yaw, Right: Pitch/Roll).
  - Dynamic virtual gamepad visualization with live stick tracking, D-pad, face buttons (A/B/X/Y), and shoulder/system buttons.
  - Integrated **Safety Lock** system to prevent accidental takeoff or commands.

---

## Requirements

- **OS**: Linux
- **Dependencies**:
  - CMake (>= 3.21)
  - C++20 compiler (`g++` or `clang++`)
  - SDL2 (`libsdl2-dev`)
  - OpenGL (`libgl1-mesa-dev`)
  - libjpeg (`libjpeg-dev`)
  - FFmpeg (`ffmpeg` binary in system `PATH` for video recording)

---

## Build & Run

1. **Install dependencies** (Debian/Ubuntu):
   ```bash
   sudo apt update
   sudo apt install build-essential cmake libsdl2-dev libgl1-mesa-dev libjpeg-dev ffmpeg
   ```

2. **Connect to the Drone**:
   - Power on the drone.
   - Connect your computer's Wi-Fi adapter to the drone's access point (e.g., `WIFI_8K_XXXXXX`).

3. **Compile and Launch**:
   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j$(nproc)
   ./build/tdg_direct_desktop
   ```

*(Dear ImGui is automatically downloaded and built via CMake FetchContent).*

---

## Gamepad & Keyboard Controls

### Gamepad Mapping (Mode 2)

| Input | Function |
|---|---|
| **Left Stick (Y-Axis)** | Throttle (0% at bottom, 100% at top) |
| **Left Stick (X-Axis)** | Yaw (Rotate left/right) |
| **Right Stick (Y-Axis)** | Pitch (Forward/backward) |
| **Right Stick (X-Axis)** | Roll (Bank left/right) |
| **A Button** | Toggle Safety Lock (ON/OFF) |
| **B Button** | Auto Land |
| **X Button** | Emergency Stop |
| **Y Button** | 360° Flip |
| **LB Bumper** | Lock Motors (Disarm) |
| **RB Bumper** | Unlock Motors (Arm) |
| **Back Button** | Calibrate Gyroscope |
| **Start Button** | Toggle Headless Mode |
| **D-Pad Down** | Toggle Video Recording |
| **D-Pad Up / Left / Right** | Cycle View Mode (`NORM` $\rightarrow$ `WIDE` $\rightarrow$ `FULL`) |

### Keyboard Shortcuts

- **`R`**: Toggle Video Recording
- **`V`** / **`F`**: Cycle View Mode (`NORM` / `WIDE` / `FULL`)

---

## Protocol & Architecture Details

- **Drone IP**: `192.168.4.153`
- **Video Port**: UDP `8080` (Camera stream)
- **Control Port**: UDP `8090` (E88 / WIFI_8K control packets)
- **Control Packet Structure**:
  `[0x66, Roll, Pitch, Throttle, Yaw, Flags, Checksum, 0x99]`
  - `Flags`:
    - Bit 0 (`0x01`): Takeoff
    - Bit 1 (`0x02`): Land
    - Bit 2 (`0x04`): Emergency Stop
    - Bit 3 (`0x08`): 360° Flip
    - Bit 4 (`0x10`): Headless Mode Toggle
    - Bit 5 (`0x20`): Lock Motors (Disarm)
    - Bit 6 (`0x40`): Unlock Motors (Arm)
    - Bit 7 (`0x80`): Calibrate Gyro
