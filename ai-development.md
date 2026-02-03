# AI-Assisted Development Plan: LSM6DSV16X Integration

## 1. System Implementation & Data Flow
The firmware is designed using a **Producer-Consumer** architecture to ensure the 120Hz sampling rate is maintained without data loss or significant timing jitter.



### Key Components:
* **Hardware Layer:** The LSM6DSV16X is configured in **High-Accuracy ODR (HAODR)** mode. The **SFLP (Sensor Fusion)** engine generates quaternions internally at 120Hz.
* **Interrupt Service Routine (ISR):** To meet the requirement for "precise sample timing," the ISR triggers on the falling edge of the INT1 pin and immediately captures the **ESP32-S3 timestamp** using `esp_timer_get_time()`.
* **FreeRTOS Queue:** This timestamp is passed via a thread-safe queue to a processing task.
* **Processing Task:** This task remains in a blocked state until a timestamp is received. It then clears the sensor interrupt, reads the FIFO, and performs the Quaternion-to-Euler conversion.



---

## 2. Summary of AI Consultations

| Main Questions Asked | AI Guidance / Resolution |
| :--- | :--- |
| **Why is the I2C address not responding?** | Identified that the ST library uses 8-bit addresses (0xD5), while ESP-IDF requires 7-bit (0x6B). Corrected the handle to use `(LSM6DSV16X_I2C_ADD_H >> 1)`. |
| **How do I ensure microsecond-level timing?** | Advised moving the timestamp capture into the **ISR** rather than the loop to eliminate task-scheduling jitter. |
| **Why does the interrupt only fire once?** | Explained the "Latched Interrupt" behavior. Recommended using `lsm6dsv16x_all_sources_get` to clear the latch and allow the INT pin to pulse again. |
| **How do I get Euler angles from the sensor?** | Provided the mathematical implementation of $atan2f$ and $asinf$ to translate the SFLP rotation vector into Roll, Pitch, and Yaw degrees. |
| **How do I fix a "Detached HEAD" in Git?** | Guided the process of creating a temporary branch and merging it back into `main` to satisfy lab branching requirements. |

---

## 3. Acceleration of Firmware Development
AI tools served as a **Senior Firmware Consultant**, accelerating the project in three primary ways:

1.  **Complexity Reduction:** By providing specific register-level configurations for the LSM6DSV16X library, the AI removed the need to manually parse hundreds of pages of technical documentation for SFLP and FIFO batching.
2.  **Rapid Debugging:** The AI interpreted terminal logs (e.g., GPIO levels and return codes) to diagnose hardware-software synchronization issues—specifically the interrupt deadlock—saving hours of troubleshooting with an oscilloscope.
3.  **Algorithmic Implementation:** The AI automated the coordinate transformation math required to convert raw quaternions into Euler angles, ensuring high-accuracy orientation data was ready for terminal verification.

---

## 4. Technical Specifications
* **ODR:** HA01 (125Hz)
* **SFLP Engine:** 120Hz Game Rotation Vector
* **Bus Speed:** I2C Fast Mode (400kHz)
* **OS:** FreeRTOS (ESP-IDF)