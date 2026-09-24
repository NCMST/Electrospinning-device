# Electrospinning Device Controller Firmware

Control software for automated electrospinning equipment based on ESP32-S3 MCU running Zephyr RTOS.

## Development Environment Setup

Activate the Zephyr virtual environment and environment variables:

```bash
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh
```

Build firmware for WeAct ESP32-S3 board:

```bash
west build -b weact_esp32s3_b/esp32s3/procpu
```

Flash firmware to target device:

```bash
west flash
```

## System Architecture

The software utilizes Zephyr's SMP (Symmetric Multiprocessing) capabilities across both ESP32-S3 Xtensa cores (PRO_CPU / Core 0 and APP_CPU / Core 1). Threads are strictly pinned to specific cores to guarantee deterministic real-time control for motors and safety while handling non-deterministic networking tasks separately.

```mermaid
graph LR
    classDef core0 fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#fff;
    classDef core1 fill:#1e293b,stroke:#10b981,stroke-width:2px,color:#fff;
    classDef ipc fill:#0f172a,stroke:#f59e0b,stroke-width:2px,color:#fff;
    classDef nodeStyle fill:#334155,stroke:#64748b,color:#fff;

    subgraph C0["Core 0 (PRO_CPU)<br/>Comms & High-Level Logic"]
        direction TB
        MQTT["MQTT Client Thread"]:::nodeStyle
        USB["USB / Shell CLI Thread"]:::nodeStyle
        CALC["Telemetry & Flow Calculator"]:::nodeStyle
    end

    subgraph IPC["Inter-Core Communication (IPC)"]
        direction TB
        CMD_Q[("k_msgq: Command Queue")]:::nodeStyle
        TEL_Q[("k_msgq: Telemetry Queue")]:::nodeStyle
        MUTEX[("Shared State & Mutex")]:::nodeStyle
    end

    subgraph C1["Core 1 (APP_CPU)<br/>Real-Time Control Loop"]
        direction TB
        MOT["Stepper Driver (TMC2209)"]:::nodeStyle
        SAFE["Safety & Power (220V/ATX)"]:::nodeStyle
        UI["UI & Sensors (Encoder/OLED/SHT)"]:::nodeStyle
    end

    %% Flow Core 0 -> IPC
    MQTT -->|Push Cmd| CMD_Q
    USB -->|Push Cmd| CMD_Q
    TEL_Q -->|Read Data| CALC
    CALC -->|Publish| MQTT
    CALC -->|Print| USB
    CALC <-->|Read/Write| MUTEX

    %% Flow IPC -> Core 1
    CMD_Q -->|Pop Cmd| MOT
    CMD_Q -->|Pop Cmd| SAFE
    MOT -->|Push Status| TEL_Q
    SAFE -->|Push Status| TEL_Q
    UI -->|Push Sensors| TEL_Q
    MOT <-->|Sync State| MUTEX

    class C0 core0;
    class C1 core1;
    class IPC ipc;
```

## Core Allocation Strategy

- Core 0 (PRO_CPU): Connectivity & Logic
  - MQTT Task: andles Wi-Fi connection, subscribes to control topics, and publishes periodic telemetry payloads.
  - USB CLI Task: Real-time console interface via USB CDC-ACM for local calibration and configuration.
  - Telemetry & Volume Calculator: Computes total dispensed syringe volume ($V = \pi \cdot r^2 \cdot h$), linear flow rate ($\text{mL/h}$), elapsed operation time, and formats outgoing data buffers.
  
- Core 1 (APP_CPU): Hard Real-Time Execution
 - Stepper Motor Task: High-priority execution loop generating STEP/DIR pulse timing for TMC2209 drivers, acceleration profiles, and limit switch monitoring.
 - Safety & Power Control Task: Direct control of PS_ON (ATX24 power supply activation), monitoring Power Good (PG_F), and driving 220V Triac optocouplers.
 - Display & Human Interface: Periodic sampling of SHT30 sensor via I2C, reading rotary encoder quadratures, and driving the SSD1306 OLED screen.