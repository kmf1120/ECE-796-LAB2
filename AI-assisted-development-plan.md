# AI-assisted Development Plan

## Overview
This document summarizes the plan and implementation steps taken to integrate the LSM6DSV16X driver into the ESP‑IDF project to enable IMU data acquisition over I²C, configure the sensor for HAODR operation with SFLP enabled, capture ESP32‑S3 timestamps on gyroscope data‑ready interrupts, print acceleration, angular velocity, and Euler angles computed from quaternions, and transmit sensor data over BLE using the NimBLE stack.

# AI-Assisted Development Plan: LSM6DSV16X 120Hz IMU + BLE Transmission

### Phase 1: Hardware Abstraction & Communication
* **Objective:** Establish a robust I2C link between the ESP32-S3 and the LSM6DSV16X.
* **Tasks:**
    * Configure I2C Master driver (**GPIO8/SDA**, **GPIO9/SCL**) at **400kHz**.
    * Implement the `platform_write` and `platform_read` bridge functions to map the ST Microelectronics C driver to the ESP-IDF API.
    * Perform a 7-bit address shift from the library default (0xD5) to the ESP-standard (**0x6B**).
* **Validation:** Verify communication via the `WHO_AM_I` register (Expected: **0x70**).

### Phase 2: Sensor Initialization & ODR Configuration
* **Objective:** Configure the sensor for high-frequency, reliable sampling.
* **Tasks:**
    * Execute a **Software Reset** to ensure a clean register state.
    * Enable **Block Data Update (BDU)** to prevent "torn reads" (reading MSB/LSB from different clock cycles).
    * Configure Accelerometer (**±4g**) and Gyroscope (**±1000dps**) full-scale ranges.
    * Synchronize Output Data Rates (ODR) to **120Hz**.

### Phase 3: High-Precision Interrupt System
* **Objective:** Eliminate timing jitter by moving data acquisition to a hardware-triggered event.
* **Tasks:**
    * Configure **GPIO4** as a rising-edge interrupt source.
    * Develop an **IRAM-optimized ISR** to capture microsecond-precision timestamps using `esp_timer_get_time()`.
    * Route **DRDY** (Data Ready) signals from the sensor's internal logic to the physical **INT1** pin.
    * Implement a **FreeRTOS Queue** to pass timestamps from the ISR to the main processing task.
* **Validation:** Use `lsm6dsv16x_all_sources_get` to clear latched interrupts and ensure continuous firing.

### Phase 4: SFLP Engine & FIFO Management
* **Objective:** Leverage on-chip Sensor Fusion Low Power (SFLP) for stable orientation data.
* **Tasks:**
    * Enable the **SFLP Game Rotation Vector** (6-axis fusion).
    * Configure the **FIFO Buffer** in Stream Mode to batch quaternion packets.
    * Implement packet tagging logic to identify SFLP data within the FIFO stream.

### Phase 5: Data Processing & Coordinate Transformation
* **Objective:** Translate raw binary data into human-readable Euler angles.
* **Tasks:**
    * **Quaternion Decoding:** Convert Q14 fixed-point FIFO data into floating-point quaternions (w, x, y, z).
    * **Normalization:** Apply a unit-length constraint to the quaternion to prevent mathematical drift.
    * **Euler Conversion:** Implement the **ZYX Tait-Bryan convention** (atan2f and asinf) to derive Roll, Pitch, and Yaw.
    * **Comparison Logic:** Calculate an accelerometer-only tilt estimate to validate SFLP performance.

### Phase 6: Buffer Management & Verification
* **Objective:** Manage high-speed data and display results at a readable rate.
* **Tasks:**
    * Maintain a **120-sample circular buffer** (storing 1 second of motion history).
    * Implement a **1Hz "gate"** to print detailed diagnostics to the terminal every 120 samples.

### Phase 7: BLE Peripheral with NimBLE Stack (Lab 3)
* **Objective:** Transmit live IMU sensor data over BLE so a smartphone can receive and verify it.
* **Tasks:**
    * Initialize **NVS** (required for NimBLE bonding storage).
    * Derive a unique device name **Lab3-XXXX** from the BT MAC address.
    * Call `nimble_port_init()` (ESP-IDF v5.x unified init — handles BT controller, HCI, and host).
    * Register a **GATT service** (UUID `0xFFF0`) with a **Notify/Read characteristic** (UUID `0xFFF1`).
    * Configure **GAP advertising**: undirected, connectable, general discoverable, with service UUID in the advertisement payload.
    * Handle GAP events: `CONNECT`, `DISCONNECT`, `SUBSCRIBE`, `ADV_COMPLETE`.
* **Key Lesson:** In ESP-IDF v5.x, do NOT manually call `esp_bt_controller_init/enable` or `esp_nimble_hci_init` — `nimble_port_init()` handles all of this internally. Manual calls cause "controller init failed" crashes.

### Phase 8: BLE Packet Encoding (Lab 3)
* **Objective:** Encode sensor data into the Lab3-specified 33-byte binary packet.
* **Packet Layout:**

| Offset | Size | Field | Scaling |
|--------|------|-------|---------|
| 0 | 3 | Header | Constant `0xAA 0xAA 0xAA` |
| 3 | 4 | Sample counter | `uint32`, little-endian, increments per notify |
| 7 | 8 | Timestamp | `uint64`, ESP32 local µs, little-endian |
| 15 | 6 | Accel X/Y/Z | 3×`int16` LE raw; mg = raw × 0.122; m/s² = raw × 0.122 × 9.80665e-3 (±4 g FS) |
| 21 | 6 | Gyro X/Y/Z | 3×`int16` LE raw; mdps = raw × 35.0; dps = raw × 0.035 (±1000 dps FS) |
| 27 | 6 | Quat qx/qy/qz | 3×`uint16` LE, IEEE 754 half-float; qw = √(1 − qx² − qy² − qz²) |

* **Total:** 33 bytes, verified at compile time with `_Static_assert`.
* **Tasks:**
    * Store raw `int16` accelerometer and gyroscope register values alongside the float conversions in `imu_sample_t`.
    * Implement `float_to_half()` to convert float quaternion components to IEEE 754 binary16.
    * Build a `ble_imu_packet_t` (packed struct) in `ble_notify_task` from the latest circular-buffer sample.
    * Send via `ble_gatts_notify_custom()` at ~10 Hz when a central has subscribed.

### Phase 9: Verification with nRF Connect (Lab 3)
* **Objective:** Prove end-to-end data flow from sensor to smartphone.
* **Verification Checklist:**
    * Device appears as **Lab3-E42E** in nRF Connect scanner.
    * Service `0xFFF0` and characteristic `0xFFF1` are discoverable after connecting.
    * Enabling notifications on `0xFFF1` delivers continuous 33-byte packets.
    * First 3 bytes of every packet are `0xAA 0xAA 0xAA`.
    * Sample counter (bytes 3–6) increments by 1 each packet.
    * Timestamp (bytes 7–14) changes between packets.
    * Accelerometer/Gyroscope values are non-zero and physically reasonable.

---

# Firmware Integration Flow (Lab 2 + Lab 3)

```
+---------------------------------------------------------+
| Phase 1 & 2: System Initialization (app_main)           |
+---------------------------------------------------------+
| 1. Initialize I2C Master (400kHz | Pins 8 & 9)          |
| 2. Identity Check (WHO_AM_I == 0x70)                    |
| 3. Software Reset & BDU Enable                          |
| 4. ODR Config: Accel/Gyro/SFLP @ 120Hz                  |
+---------------------------+-----------------------------+
                            |
                            v
+---------------------------+-----------------------------+
| Phase 3: Hardware Interrupt (120Hz Pulse)               |
+---------------------------------------------------------+
| [IMU INT1 Pin] -> [ESP32 GPIO4 Rising Edge]             |
|          |                                              |
|          +--> [IRAM ISR]: Capture Timestamp (us)        |
|          +--> [Queue]: Push Timestamp                   |
+---------------------------+-----------------------------+
                            |
                            v
+---------------------------+-----------------------------+
| Phase 4 & 5: Data Acquisition & Math (imu_task)         |
+---------------------------------------------------------+
| 1. Wake up on Queue Receive                             |
| 2. Read Accel & Gyro Registers (raw int16 + float)      |
| 3. Read SFLP Quaternion from FIFO                       |
| 4. Normalize Quaternion (||q|| = 1)                     |
| 5. Convert to Euler (Roll, Pitch, Yaw)                  |
| 6. Update 120-Sample Circular Buffer                    |
+---------------------------+-----------------------------+
                            |
                            v
+---------------------------+-----------------------------+
| Phase 6: Buffered Display & Verification                |
+---------------------------------------------------------+
| IF (Samples % 120 == 0)                                 |
|    +--> PRINT: Accel/Gyro/Euler Data (1 Hz)             |
| Clear Interrupt Latch (all_sources_get)                  |
+---------------------------+-----------------------------+
                            |
                            v
+---------------------------+-----------------------------+
| Phase 7-9: BLE Data Transmission (Lab 3)                |
+---------------------------------------------------------+
| [ble_init] NVS -> nimble_port_init -> GATT/GAP setup    |
|          |                                              |
|          +--> [ble_advertise_start]: Lab3-XXXX visible  |
|          +--> [ble_gap_event]: CONNECT / DISCONNECT     |
|          +--> [ble_notify_task @ 10 Hz]:                |
|               Read latest sample from circular buffer   |
|               Encode 33-byte packet (AA AA AA header)   |
|               -> ble_gatts_notify_custom()              |
|               -> nRF Connect receives packet            |
+---------------------------------------------------------+
```

---

### Summary of AI Consultations ###

| Main Questions Asked | AI Guidance / Resolution |
| :--- | :--- |
| **Why is the I2C address not responding?** | Identified that the ST library uses 8-bit addresses (0xD5), while ESP-IDF requires 7-bit (**0x6B**). Corrected the handle to use `(LSM6DSV16X_I2C_ADD_H >> 1)`. |
| **How do I ensure microsecond-level timing?** | Advised moving the timestamp capture into the **ISR** rather than the task loop. This eliminated task-scheduling jitter and ensured the timestamp reflects the exact moment of physical data readiness. |
| **Why does the interrupt only fire once?** | Explained the "Latched Interrupt" behavior. Recommended calling `lsm6dsv16x_all_sources_get` after data retrieval to clear the latch, allowing the INT1 pin to return to its idle state for the next pulse. |
| **How do I fix the "All Zeros" sensor output?** | Diagnosed a configuration conflict between standard and High-Accuracy ODR modes. Resolved by implementing a proper software reset sequence and standardizing on the **120Hz ODR** constant. |
| **Why are Euler angles inaccurate/drifting?** | Identified a missing normalization step in the quaternion-to-Euler conversion. Provided a robust implementation with unit-length constraints and gimbal lock protection for the asinf function. |
| **How do I fix a "Detached HEAD" in Git?** | Guided the process of creating a temporary branch and merging it back into `main` to satisfy the university lab's version control requirements. |
| **How do I add BLE to my ESP32-S3 firmware?** | Guided use of the NimBLE stack: registering a GATT service (UUID 0xFFF0) with a Notify characteristic (UUID 0xFFF1), configuring GAP advertising, and handling connect/disconnect/subscribe events. |
| **Why does "controller init failed" crash?** | Identified that in ESP-IDF v5.x, `nimble_port_init()` internally handles BT controller + HCI init. Manually calling `esp_bt_controller_init/enable` + `esp_nimble_hci_init` causes a double-init conflict and LoadProhibited panic. Fix: use only `nimble_port_init()`. |
| **How should I encode sensor data in the BLE packet?** | Designed a 33-byte packed struct with 0xAA header, raw int16 accel/gyro, and IEEE 754 half-float quaternions. Documented scaling factors (0.122 mg/LSB for accel, 35 mdps/LSB for gyro). Verified size at compile time with `_Static_assert`. |
| **How do I convert float to half-float in C?** | Provided a bit-manipulation function extracting sign, exponent (rebased from bias-127 to bias-15), and truncated mantissa to produce a compliant binary16 uint16 value. |

---
### How did AI Tools Accelerate Development? ###
AI tools acted as a **Senior Firmware Consultant** throughout the lifecycle of this project, transforming a complex sensor integration into a structured, verifiable engineering process. The following areas highlight how AI accelerated the development:

### 1. Rapid Hardware-Software Bridge Construction
Interfacing with high-performance STMicroelectronics sensors typically requires parsing hundreds of pages of technical datasheets and register maps. AI tools significantly reduced this overhead by:
* **Automating the Platform Layer:** Generating the I2C "wrapper" functions (`platform_write`/`read`) needed to bridge the generic ST driver with the ESP-IDF specific I2C driver.
* **I2C Address Translation:** Instantly resolving the 8-bit vs. 7-bit addressing conflict (0xD5 vs. 0x6B), which is a common stumbling block in embedded systems.

### 2. High-Frequency Logic & Determinism
Ensuring data integrity at 120Hz requires an understanding of Real-Time Operating Systems (RTOS) and hardware constraints. AI support was critical in:
* **Interrupt Optimization:** Designing the **IRAM-optimized ISR** to ensure microsecond-precision timestamping, moving beyond a simple polling architecture to a professional interrupt-driven model.
* **Deadlock Resolution:** Diagnosing the "single-pulse" interrupt issue by identifying the need to clear the sensor's internal latching register (`all_sources_get`), which prevented hardware stalls.

### 3. Advanced Mathematical Implementation
Moving from raw sensor counts to human-readable orientation is mathematically intensive. AI tools accelerated this by:
* **Sensor Fusion Logic:** Implementing the complex SFLP FIFO decoding logic and the Q14 fixed-point to floating-point conversion.
* **Robust Euler Transformations:** Developing the ZYX Tait-Bryan conversion math with integrated **Gimbal Lock** protection and quaternion normalization, ensuring that the orientation data remained accurate even during rapid movement.

### 4. BLE Stack Integration & Debugging (Lab 3)
Adding wireless data transmission required understanding the full NimBLE stack lifecycle. AI tools were essential for:
* **API Version Awareness:** Identifying that ESP-IDF v5.x consolidated BT controller, HCI, and NimBLE host initialization into a single `nimble_port_init()` call — preventing the "controller init failed" crash caused by redundant manual init calls.
* **GATT Service Design:** Structuring the service/characteristic hierarchy (0xFFF0/0xFFF1) with correct flags (READ | NOTIFY) and CCCD handling for subscription management.
* **Binary Packet Engineering:** Designing the 33-byte packed struct with compile-time size verification, raw int16 sensor values for efficient transmission, and IEEE 754 half-float quaternion encoding to meet a compact 6-byte budget.
* **Crash Diagnosis:** Interpreting LoadProhibited panic backtraces to pinpoint the root cause (uninitialized NimBLE mutexes due to failed controller init) and implement the correct fix.

---

**Summary of Impact:**
By leveraging AI for low-level register configuration, high-level algorithmic math, and BLE stack integration, the development cycle was compressed from weeks of manual troubleshooting into a matter of days. This allowed for more time to be spent on the core project objective: **IoT sensor data transmission and verification.**

