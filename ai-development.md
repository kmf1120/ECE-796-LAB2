# AI-Assisted Development Plan: LSM6DSV16X Integration + BLE Transmission

## 1. System Implementation & Data Flow
The firmware is designed using a **Producer-Consumer** architecture to ensure the 120Hz sampling rate is maintained without data loss or significant timing jitter. BLE transmission runs as a separate consumer that reads from the shared circular buffer.

### Key Components:
* **Hardware Layer:** The LSM6DSV16X is configured in **High-Accuracy ODR (HAODR)** mode. The **SFLP (Sensor Fusion)** engine generates quaternions internally at 120Hz.
* **Interrupt Service Routine (ISR):** To meet the requirement for "precise sample timing," the ISR triggers on the rising edge of the INT1 pin and immediately captures the **ESP32-S3 timestamp** using `esp_timer_get_time()`.
* **FreeRTOS Queue:** This timestamp is passed via a thread-safe queue to a processing task.
* **Processing Task (`imu_task`):** Remains in a blocked state until a timestamp is received. Reads accel/gyro registers (storing both raw int16 and float conversions), reads SFLP quaternion from FIFO, converts to Euler angles, and updates the circular buffer.
* **BLE Notify Task (`ble_notify_task`):** At ~10 Hz, reads the latest completed sample from the circular buffer, encodes a 33-byte binary packet, and sends it via `ble_gatts_notify_custom()`.

---

## 2. BLE Implementation Details (Lab 3)

### Initialization Flow
1. **NVS Flash** — Required by NimBLE for bonding data storage.
2. **Device Naming** — Unique name `Lab3-XXXX` derived from BT MAC address last 2 bytes.
3. **`nimble_port_init()`** — Single call that initializes BT controller, HCI transport, and NimBLE host (ESP-IDF v5.x API).
4. **GATT Registration** — Service UUID `0xFFF0`, characteristic UUID `0xFFF1` with `READ | NOTIFY` flags.
5. **GAP Advertising** — Undirected connectable, general discoverable, includes service UUID in payload.

### 33-Byte Packet Format

| Offset | Size | Field | Scaling |
|--------|------|-------|---------|
| 0 | 3 | Header | Constant `0xAA 0xAA 0xAA` |
| 3 | 4 | Sample counter | `uint32` LE, increments per notify |
| 7 | 8 | Timestamp | `uint64` LE, ESP32 µs |
| 15 | 6 | Accel X/Y/Z | 3×`int16` LE raw; mg = raw × 0.122 (±4 g) |
| 21 | 6 | Gyro X/Y/Z | 3×`int16` LE raw; mdps = raw × 35.0 (±1000 dps) |
| 27 | 6 | Quat qx/qy/qz | 3×`uint16` LE, IEEE 754 half-float |

**Total: 33 bytes** — enforced with `_Static_assert(sizeof(ble_imu_packet_t) == 33)`.

### Key Source Code References
* `ble_init()` — NVS, nimble_port_init, GATT/GAP setup, starts host task + notify task.
* `ble_advertise_start()` — Configures adv fields (name, UUID, flags) and starts advertising.
* `ble_gap_event()` — Handles CONNECT, DISCONNECT, SUBSCRIBE, ADV_COMPLETE.
* `ble_notify_task()` — Reads latest `imu_sample_t`, builds `ble_imu_packet_t`, sends notification.
* `float_to_half()` — Converts float32 to IEEE 754 binary16 for quaternion encoding.

---

## 3. Summary of AI Consultations

| Main Questions Asked | AI Guidance / Resolution |
| :--- | :--- |
| **Why is the I2C address not responding?** | Identified that the ST library uses 8-bit addresses (0xD5), while ESP-IDF requires 7-bit (0x6B). Corrected the handle to use `(LSM6DSV16X_I2C_ADD_H >> 1)`. |
| **How do I ensure microsecond-level timing?** | Advised moving the timestamp capture into the **ISR** rather than the loop to eliminate task-scheduling jitter. |
| **Why does the interrupt only fire once?** | Explained the "Latched Interrupt" behavior. Recommended using `lsm6dsv16x_all_sources_get` to clear the latch and allow the INT pin to pulse again. |
| **How do I get Euler angles from the sensor?** | Provided the mathematical implementation of atan2f and asinf to translate the SFLP rotation vector into Roll, Pitch, and Yaw degrees. |
| **How do I fix a "Detached HEAD" in Git?** | Guided the process of creating a temporary branch and merging it back into `main` to satisfy lab branching requirements. |
| **How do I initialize BLE on ESP-IDF v5.x?** | Explained that `nimble_port_init()` is the unified init call for ESP-IDF v5.x. Manual `esp_bt_controller_init` + `esp_nimble_hci_init` causes double-init crashes. |
| **Why does "controller init failed" crash the board?** | The BT controller partially initialized but the code continued into `ble_svc_gap_init()` with uninitialized NimBLE mutexes, causing a LoadProhibited panic. Fix: use only `nimble_port_init()`. |
| **How to encode sensor data for BLE?** | Designed 33-byte packed struct with 0xAA header, raw int16 sensor values, and IEEE 754 half-float quaternions. Added `_Static_assert` for compile-time size check. |

---

## 4. Acceleration of Firmware Development
AI tools served as a **Senior Firmware Consultant**, accelerating the project in four primary ways:

1. **Complexity Reduction:** By providing specific register-level configurations for the LSM6DSV16X library, the AI removed the need to manually parse hundreds of pages of technical documentation for SFLP and FIFO batching.
2. **Rapid Debugging:** The AI interpreted terminal logs (GPIO levels, return codes, panic backtraces) to diagnose hardware-software synchronization issues and BLE stack initialization failures, saving hours of troubleshooting.
3. **Algorithmic Implementation:** The AI automated the coordinate transformation math (quaternion-to-Euler) and IEEE 754 half-float conversion, ensuring mathematically correct implementations.
4. **BLE Stack Navigation:** The AI identified the correct ESP-IDF v5.x NimBLE initialization pattern, designed the GATT service/characteristic hierarchy, and engineered the binary packet format to meet the Lab3 specification.

---

## 5. Technical Specifications
* **ODR:** 120Hz (Accel, Gyro, SFLP)
* **SFLP Engine:** Game Rotation Vector (6-axis, no magnetometer)
* **Bus Speed:** I2C Fast Mode (400kHz)
* **OS:** FreeRTOS (ESP-IDF v5.5.2)
* **BLE Stack:** NimBLE (ESP-IDF integrated)
* **BLE Service:** UUID `0xFFF0`, Characteristic UUID `0xFFF1` (Notify + Read)
* **BLE Packet:** 33 bytes, ~10 Hz notification rate
* **Device Name:** `Lab3-E42E` (derived from BT MAC 94:A9:90:0C:E4:2E)
