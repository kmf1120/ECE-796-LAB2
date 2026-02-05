# AI-assisted Development Plan ✅

## Overview
This document summarizes the plan and implementation steps taken to integrate the LSM6DSV16X driver into the ESP‑IDF project to enable IMU data acquisition over I²C, configure the sensor for HAODR operation with SFLP enabled, capture ESP32‑S3 timestamps on gyroscope data‑ready interrupts, and print acceleration, angular velocity, and Euler angles computed from quaternions.

# AI-Assisted Development Plan: LSM6DSV16X 120Hz IMU Integration

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
    * **Quaternion Decoding:** Convert Q14 fixed-point FIFO data into floating-point quaternions ($w, x, y, z$).
    * **Normalization:** Apply a unit-length constraint to the quaternion to prevent mathematical drift.
    * **Euler Conversion:** Implement the **ZYX Tait-Bryan convention** ($atan2f$ and $asinf$) to derive Roll, Pitch, and Yaw.
    * **Comparison Logic:** Calculate an accelerometer-only tilt estimate to validate SFLP performance.

### Phase 6: Buffer Management & Verification
* **Objective:** Manage high-speed data and display results at a readable rate.
* **Tasks:**
    * Maintain a **120-sample circular buffer** (storing 1 second of motion history).
    * Implement a **1Hz "gate"** to print detailed diagnostics to the terminal every 120 samples.
---

# LSM6DSV16X Firmware Integration Flow

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
| 2. Read Accel & Gyro Registers via I2C                  |
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
|    |                                                    |
|    +--> PRINT: Total Count & Timestamp                  |
|    +--> PRINT: Accel/Gyro/Euler Data                    |
|    +--> PRINT: Accel-Only Comparison                    |
|                                                         |
| 7. Clear Int (lsm6dsv16x_all_sources_get)               |
+---------------------------------------------------------+
---

### Summary of AI Consultations ###

| Main Questions Asked | AI Guidance / Resolution |
| :--- | :--- |
| **Why is the I2C address not responding?** | Identified that the ST library uses 8-bit addresses (0xD5), while ESP-IDF requires 7-bit (**0x6B**). Corrected the handle to use `(LSM6DSV16X_I2C_ADD_H >> 1)`. |
| **How do I ensure microsecond-level timing?** | Advised moving the timestamp capture into the **ISR** rather than the task loop. This eliminated task-scheduling jitter and ensured the timestamp reflects the exact moment of physical data readiness. |
| **Why does the interrupt only fire once?** | Explained the "Latched Interrupt" behavior. Recommended calling `lsm6dsv16x_all_sources_get` after data retrieval to clear the latch, allowing the INT1 pin to return to its idle state for the next pulse. |
| **How do I fix the "All Zeros" sensor output?** | Diagnosed a configuration conflict between standard and High-Accuracy ODR modes. Resolved by implementing a proper software reset sequence and standardizing on the **120Hz ODR** constant. |
| **Why are Euler angles inaccurate/drifting?** | Identified a missing normalization step in the quaternion-to-Euler conversion. Provided a robust implementation with unit-length constraints and gimbal lock protection for the $asinf$ function. |
| **How do I fix a "Detached HEAD" in Git?** | Guided the process of creating a temporary branch and merging it back into `main` to satisfy the university lab's version control requirements. |

---
### How did AI Tools Accelerate Development? ###
AI tools acted as a **Senior Firmware Consultant** throughout the lifecycle of this project, transforming a complex sensor integration into a structured, verifiable engineering process. The following three areas highlight how AI accelerated the development:

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



---

**Summary of Impact:**
By leveraging AI for low-level register configuration and high-level algorithmic math, the development cycle was compressed from weeks of manual troubleshooting into a matter of days. This allowed for more time to be spent on the core project objective: **IoT data integrity and comparative analysis.**

