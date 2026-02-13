// need to know where the application layer and other layers reside.
// gatt- generic attribute profile, defines the structure of the data and services provided by the device
// uses concepts called services and characteristics to communicate data. Services are collections of characteristics, and characteristics are individual data points or attributes that can be read, written, or subscribed to for notifications.
// look at slides, need to condigure CCCD?


#include "sdkconfig.h"
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_bt.h"
#include "lsm6dsv16x_reg.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

static const char *TAG = "imu";

/* Platform context and helpers for ST driver */
static stmdev_ctx_t dev_ctx;
typedef struct {
    i2c_port_t i2c_num;
    uint8_t i2c_addr; /* 7-bit */
} lsm6dsv16x_handle_t;
static lsm6dsv16x_handle_t lsm_handle;

/* Data structure for IMU samples */
typedef struct {
    int64_t timestamp_us;     // ESP32 timestamp in microseconds
    float ax, ay, az;         // Acceleration in m/s²
    float gx, gy, gz;         // Angular velocity in dps
    float q0, q1, q2, q3;     // Quaternion (w, x, y, z)
    float roll, pitch, yaw;   // Euler angles in degrees
} imu_sample_t;

#define SAMPLE_BUFFER_SIZE 120  // Store 1 second of data at 120Hz
static imu_sample_t sample_buffer[SAMPLE_BUFFER_SIZE];
static volatile uint32_t sample_index = 0;
static volatile uint64_t total_samples = 0;

typedef struct {
    uint32_t sample_counter;
    uint64_t timestamp_us;
} ble_telemetry_t;

static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t ble_tx_chr_val_handle;
static uint8_t ble_addr_type;
static bool ble_notify_enabled = false;
static volatile uint32_t ble_notify_counter = 0;
static char ble_device_name[20] = "Lab3-IMU";

static QueueHandle_t drdy_queue;

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_advertise_start(void);

static int ble_chr_access_cb(uint16_t conn_handle,
                             uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    ble_telemetry_t packet = {
        .sample_counter = ble_notify_counter,
        .timestamp_us = (uint64_t)esp_timer_get_time(),
    };

    int rc = os_mbuf_append(ctxt->om, &packet, sizeof(packet));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFF0),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xFFF1),
                .access_cb = ble_chr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_tx_chr_val_handle,
            },
            {0},
        },
    },
    {0},
};

static void ble_notify_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE && ble_notify_enabled) {
            ble_telemetry_t packet = {
                .sample_counter = ++ble_notify_counter,
                .timestamp_us = (uint64_t)esp_timer_get_time(),
            };

            struct os_mbuf *om = ble_hs_mbuf_from_flat(&packet, sizeof(packet));
            if (om != NULL) {
                int rc = ble_gatts_notify_custom(ble_conn_handle, ble_tx_chr_val_handle, om);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Notify failed, rc=%d", rc);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void ble_on_sync(void)
{
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(ble_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_hs_id_copy_addr failed: %d", rc);
    } else {
        ESP_LOGI(TAG,
                 "BLE addr type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X name=%s",
                 ble_addr_type,
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0],
                 ble_device_name);
    }

    ble_advertise_start();
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "BLE connected, conn_handle=%d", ble_conn_handle);
            } else {
                ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
                ESP_LOGW(TAG, "BLE connect failed; status=%d", event->connect.status);
                ble_advertise_start();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
            ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_notify_enabled = false;
            ble_advertise_start();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == ble_tx_chr_val_handle) {
                ble_notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "BLE notify %s", ble_notify_enabled ? "enabled" : "disabled");
            }
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_advertise_start();
            return 0;

        default:
            return 0;
    }
}

static void ble_advertise_start(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *name = ble_svc_gap_device_name();
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    uint16_t svc_uuid = 0xFFF0;
    fields.uuids16 = (ble_uuid16_t[]){{ .u = { BLE_UUID_TYPE_16 }, .value = svc_uuid }};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE advertising started");
    }
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    uint8_t bt_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(bt_mac, ESP_MAC_BT));
    snprintf(ble_device_name, sizeof(ble_device_name),
             "Lab3-%02X%02X", bt_mac[4], bt_mac[5]);

    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %s", esp_err_to_name(err));
        return;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_nimble_hci_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_hci_init failed: %s", esp_err_to_name(err));
        return;
    }
    nimble_port_init();

    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(ble_device_name));

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK((rc == 0) ? ESP_OK : ESP_FAIL);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK((rc == 0) ? ESP_OK : ESP_FAIL);

    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(ble_notify_task, "ble_notify_task", 4096, NULL, 5, NULL);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    lsm6dsv16x_handle_t *h = (lsm6dsv16x_handle_t *)handle;
    uint8_t data[256];
    if (len + 1 > sizeof(data)) return -1;
    data[0] = reg;
    memcpy(&data[1], bufp, len);
    esp_err_t err = i2c_master_write_to_device(h->i2c_num, h->i2c_addr, data, len + 1, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    lsm6dsv16x_handle_t *h = (lsm6dsv16x_handle_t *)handle;
    esp_err_t err = i2c_master_write_read_device(h->i2c_num, h->i2c_addr, &reg, 1, bufp, len, pdMS_TO_TICKS(1000));
    return (err == ESP_OK) ? 0 : -1;
}

static void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// Helper: Decode SFLP rotation vector from FIFO (6 bytes -> quaternion)
static void decode_sflp_quaternion(const uint8_t data[6], float *q0, float *q1, float *q2, float *q3)
{
    // SFLP game rotation vector contains qx, qy, qz as int16_t
    int16_t qx = (int16_t)((data[1] << 8) | data[0]);
    int16_t qy = (int16_t)((data[3] << 8) | data[2]);
    int16_t qz = (int16_t)((data[5] << 8) | data[4]);
    
    // Scale factor for quaternion (typically 2^14 = 16384)
    const float Q_SCALE = 16384.0f;
    float x = qx / Q_SCALE;
    float y = qy / Q_SCALE;
    float z = qz / Q_SCALE;
    
    // Calculate w from normalization constraint: w² + x² + y² + z² = 1
    float w_sq = 1.0f - (x * x + y * y + z * z);
    float w = (w_sq > 0.0f) ? sqrtf(w_sq) : 0.0f;
    
    *q0 = w;  // Scalar part
    *q1 = x;  // Vector i
    *q2 = y;  // Vector j
    *q3 = z;  // Vector k
}

// Convert quaternion to Euler angles - FIXED version with proper normalization
static void quaternion_to_euler(float q0, float q1, float q2, float q3, 
                                float *roll, float *pitch, float *yaw)
{
    const float RAD_TO_DEG = 57.29577951308232f;
    
    // First, normalize the quaternion to ensure unit length
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0001f) {  // Avoid division by zero
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    } else {
        // Invalid quaternion, set to identity
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }
    
    // Roll (X-axis rotation) - rotation around X
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // Pitch (Y-axis rotation) - rotation around Y
    // Use atan2 instead of asin to avoid gimbal lock
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    
    // Clamp sinp to valid range for asin
    if (sinp >= 1.0f) {
        *pitch = 90.0f;  // Use 90 degrees for north pole
    } else if (sinp <= -1.0f) {
        *pitch = -90.0f;  // Use -90 degrees for south pole
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    // Yaw (Z-axis rotation) - rotation around Z
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

// Alternative: Direct accelerometer-based Euler (for comparison/debugging)
static void accel_to_euler(float ax, float ay, float az, float *roll, float *pitch)
{
    const float RAD_TO_DEG = 57.29577951308232f;
    
    // Calculate magnitude
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    
    if (mag < 0.01f) {
        // Too small, probably in freefall or no gravity
        *roll = 0.0f;
        *pitch = 0.0f;
        return;
    }
    
    // Normalize
    float ax_n = ax / mag;
    float ay_n = ay / mag;
    float az_n = az / mag;
    
    // Calculate roll and pitch from normalized acceleration
    *roll = atan2f(ay_n, az_n) * RAD_TO_DEG;
    *pitch = atan2f(-ax_n, sqrtf(ay_n * ay_n + az_n * az_n)) * RAD_TO_DEG;
}

// GPIO interrupt handler - triggers on gyro data ready
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int64_t ts = esp_timer_get_time(); // Capture timestamp immediately
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(drdy_queue, &ts, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// IMU task: reads data at 120Hz, prints at 1Hz
static void imu_task(void *arg) {
    int64_t ts;
    uint32_t print_counter = 0;
    uint32_t timeout_counter = 0;
    
    ESP_LOGI(TAG, "IMU task started - waiting for interrupts...\n");
    
    for (;;) {
        // Wait for interrupt with 2 second timeout for debugging
        if (xQueueReceive(drdy_queue, &ts, pdMS_TO_TICKS(2000)) == pdTRUE) {
            timeout_counter = 0; // Reset timeout counter
            
            uint32_t idx = sample_index;
            imu_sample_t *sample = &sample_buffer[idx];
            
            // Store timestamp
            sample->timestamp_us = ts;
            
            // Read accelerometer
            int16_t raw_acc[3] = {0};
            if (lsm6dsv16x_acceleration_raw_get(&dev_ctx, raw_acc) == 0) {
                float ax_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[0]);
                float ay_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[1]);
                float az_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[2]);
                const float MG_TO_MS2 = 9.80665e-3f;
                sample->ax = ax_mg * MG_TO_MS2;
                sample->ay = ay_mg * MG_TO_MS2;
                sample->az = az_mg * MG_TO_MS2;
            }
            
            // Read gyroscope
            int16_t raw_gyro[3] = {0};
            if (lsm6dsv16x_angular_rate_raw_get(&dev_ctx, raw_gyro) == 0) {
                float gx_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_gyro[0]);
                float gy_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_gyro[1]);
                float gz_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_gyro[2]);
                sample->gx = gx_mdps / 1000.0f;
                sample->gy = gy_mdps / 1000.0f;
                sample->gz = gz_mdps / 1000.0f;
            }
            
            // Try to read SFLP quaternion from FIFO
            lsm6dsv16x_fifo_out_raw_t fifo_raw;
            bool quaternion_valid = false;
            
            if (lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &fifo_raw) == 0) {
                if (fifo_raw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
                    decode_sflp_quaternion(fifo_raw.data, 
                                          &sample->q0, &sample->q1, 
                                          &sample->q2, &sample->q3);
                    quaternion_valid = true;
                }
            }
            
            // If no SFLP quaternion, create from accelerometer
            if (!quaternion_valid) {
                // Get direct roll/pitch from accelerometer first
                float acc_roll, acc_pitch;
                accel_to_euler(sample->ax, sample->ay, sample->az, &acc_roll, &acc_pitch);
                
                // Convert to radians for quaternion creation
                const float DEG_TO_RAD = 0.0174532925f;
                float roll_rad = acc_roll * DEG_TO_RAD;
                float pitch_rad = acc_pitch * DEG_TO_RAD;
                
                // Create quaternion from Euler angles (ZYX convention)
                float cy = cosf(0.0f * 0.5f);  // yaw = 0 (no magnetometer)
                float sy = sinf(0.0f * 0.5f);
                float cp = cosf(pitch_rad * 0.5f);
                float sp = sinf(pitch_rad * 0.5f);
                float cr = cosf(roll_rad * 0.5f);
                float sr = sinf(roll_rad * 0.5f);
                
                sample->q0 = cr * cp * cy + sr * sp * sy;  // w
                sample->q1 = sr * cp * cy - cr * sp * sy;  // x
                sample->q2 = cr * sp * cy + sr * cp * sy;  // y
                sample->q3 = cr * cp * sy - sr * sp * cy;  // z
            }
            
            // Convert quaternion to Euler angles
            quaternion_to_euler(sample->q0, sample->q1, sample->q2, sample->q3,
                              &sample->roll, &sample->pitch, &sample->yaw);
            
            // Update counters
            total_samples++;
            sample_index = (sample_index + 1) % SAMPLE_BUFFER_SIZE;
            print_counter++;
            
            // Print at 1Hz (every 120 samples)
            if (print_counter >= 120) {
                print_counter = 0;
                
                float acc_roll, acc_pitch;
                accel_to_euler(sample->ax, sample->ay, sample->az, &acc_roll, &acc_pitch);

                printf("\n--- Sample #%llu ---\n", total_samples);
                printf("Timestamp: %lld us\n", sample->timestamp_us);
                printf("Accel [m/s²]: X=%.3f, Y=%.3f, Z=%.3f\n", sample->ax, sample->ay, sample->az);
                printf("Gyro [dps]:   X=%.3f, Y=%.3f, Z=%.3f\n", sample->gx, sample->gy, sample->gz);
                printf("Quat [wxyz]:  w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", sample->q0, sample->q1, sample->q2, sample->q3);
                printf("Euler [deg]:  Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", sample->roll, sample->pitch, sample->yaw);
                // printf("Accel-Only:   Roll=%.2f, Pitch=%.2f\n", acc_roll, acc_pitch);
            }
            
            // Clear interrupt by reading all_sources register
            lsm6dsv16x_all_sources_t all_sources;
            lsm6dsv16x_all_sources_get(&dev_ctx, &all_sources);
            
        } else {
            // Timeout - no interrupt received
            timeout_counter++;
            ESP_LOGW(TAG, "No interrupt for 2 seconds (timeout #%u)", timeout_counter);
            
            // Check GPIO pin level
            int gpio_level = gpio_get_level(GPIO_NUM_4);
            ESP_LOGI(TAG, "  GPIO4 level: %d", gpio_level);
            
            // Check data ready flags
            lsm6dsv16x_all_sources_t status;
            if (lsm6dsv16x_all_sources_get(&dev_ctx, &status) == 0) {
                ESP_LOGI(TAG, "  DRDY status: xl=%d, gy=%d", status.drdy_xl, status.drdy_gy);
            }
            
            // Try reading data anyway (polling mode as fallback)
            ESP_LOGI(TAG, "  Attempting manual read...");
            int16_t raw_acc[3], raw_gyro[3];
            if (lsm6dsv16x_acceleration_raw_get(&dev_ctx, raw_acc) == 0 &&
                lsm6dsv16x_angular_rate_raw_get(&dev_ctx, raw_gyro) == 0) {
                
                float ax_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[0]);
                float ay_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[1]);
                float az_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[2]);
                const float MG_TO_MS2 = 9.80665e-3f;
                float ax = ax_mg * MG_TO_MS2;
                float ay = ay_mg * MG_TO_MS2;
                float az = az_mg * MG_TO_MS2;
                
                float acc_roll, acc_pitch;
                accel_to_euler(ax, ay, az, &acc_roll, &acc_pitch);
                
                ESP_LOGI(TAG, "  Manual: acc=[%.2f, %.2f, %.2f] m/s², roll=%.1f°, pitch=%.1f°",
                         ax, ay, az, acc_roll, acc_pitch);
            }
        }
    }
}

static void i2c_scan(i2c_port_t i2c_num)
{
    ESP_LOGI(TAG, "Starting I2C scan...");
    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ I2C device found at address 0x%02x", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s).\n", found);
}

void app_main(void) {
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   LSM6DSV16X IMU - 120Hz Collection, 1Hz Display     ║");
    ESP_LOGI(TAG, "║   with SFLP Quaternion Sensor Fusion                 ║");
    ESP_LOGI(TAG, "║   FIXED: Proper Euler Angle Conversion               ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════╝\n");

    ble_init();

    // I2C configuration
    const i2c_port_t I2C_NUM = I2C_NUM_0;
    const gpio_num_t SDA_PIN = GPIO_NUM_8;
    const gpio_num_t SCL_PIN = GPIO_NUM_9;
    const gpio_num_t INT_PIN = GPIO_NUM_4;  // INT1 pin

    ESP_LOGI(TAG, "Initializing I2C bus (SDA=GPIO%d, SCL=GPIO%d)...", SDA_PIN, SCL_PIN);
    
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 400000},
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully.\n");

    // Scan I2C bus
    i2c_scan(I2C_NUM);

    // Setup device context for ST driver
    lsm_handle.i2c_num = I2C_NUM;
    lsm_handle.i2c_addr = (uint8_t)(LSM6DSV16X_I2C_ADD_H >> 1); // 0x6B

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &lsm_handle;

    // Verify device identity
    ESP_LOGI(TAG, "Checking device identity...");
    uint8_t whoamI = 0;
    if (lsm6dsv16x_device_id_get(&dev_ctx, &whoamI) != 0) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register!");
        ESP_LOGE(TAG, "Check your I2C connections and pull-up resistors.");
        return;
    }
    
    if (whoamI != LSM6DSV16X_ID) {
        ESP_LOGE(TAG, "Wrong device ID! Expected 0x%02x, got 0x%02x", LSM6DSV16X_ID, whoamI);
        ESP_LOGE(TAG, "This may not be an LSM6DSV16X sensor.");
        return;
    }
    
    ESP_LOGI(TAG, "✓ LSM6DSV16X detected (WHO_AM_I = 0x%02x)\n", whoamI);

    // Perform software reset
    ESP_LOGI(TAG, "Performing software reset...");
    if (lsm6dsv16x_sw_reset(&dev_ctx) != 0) {
        ESP_LOGE(TAG, "Failed to reset device");
        return;
    }
    platform_delay(20);
    
    // Wait for reset to complete
    ESP_LOGI(TAG, "Waiting for reset to complete...");
    lsm6dsv16x_ctrl3_t ctrl3;
    int timeout = 100;
    do {
        if (lsm6dsv16x_read_reg(&dev_ctx, LSM6DSV16X_CTRL3, (uint8_t*)&ctrl3, 1) != 0) {
            ESP_LOGE(TAG, "Failed to read CTRL3 register");
            break;
        }
        platform_delay(1);
        timeout--;
    } while (ctrl3.sw_reset && timeout > 0);
    
    if (timeout == 0) {
        ESP_LOGW(TAG, "Reset timeout - continuing anyway");
    } else {
        ESP_LOGI(TAG, "✓ Reset complete\n");
    }

    // Configure sensor
    ESP_LOGI(TAG, "Configuring sensor...");
    int32_t rc;
    
    // Enable block data update
    rc = lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    ESP_LOGI(TAG, "✓ Block data update: rc=%d", (int)rc);
    
    // Set accelerometer full scale to ±4g
    rc = lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
    ESP_LOGI(TAG, "✓ Accel full scale ±4g: rc=%d", (int)rc);
    
    // Set gyroscope full scale to ±1000 dps
    rc = lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_1000dps);
    ESP_LOGI(TAG, "✓ Gyro full scale ±1000dps: rc=%d", (int)rc);
    
    // Set accelerometer ODR to 120 Hz
    rc = lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
    ESP_LOGI(TAG, "✓ Accel ODR 120Hz: rc=%d", (int)rc);
    
    // Set gyroscope ODR to 120 Hz
    rc = lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
    ESP_LOGI(TAG, "✓ Gyro ODR 120Hz: rc=%d", (int)rc);
    
    // Enable SFLP game rotation (quaternion without magnetometer)
    rc = lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, 1);
    ESP_LOGI(TAG, "✓ SFLP game rotation enabled: rc=%d", (int)rc);
    
    // Set SFLP data rate to 120 Hz
    rc = lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_120Hz);
    ESP_LOGI(TAG, "✓ SFLP ODR 120Hz: rc=%d", (int)rc);
    
    // Configure FIFO to output SFLP data
    lsm6dsv16x_fifo_sflp_raw_t sflp_batch = {
        .game_rotation = 1,  // Enable game rotation vector in FIFO
        .gravity = 0,
        .gbias = 0
    };
    rc = lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, sflp_batch);
    ESP_LOGI(TAG, "✓ FIFO SFLP batch configured: rc=%d", (int)rc);
    
    // Set FIFO to continuous mode
    rc = lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);
    ESP_LOGI(TAG, "✓ FIFO stream mode: rc=%d", (int)rc);
    
    // Setup GPIO interrupt BEFORE routing interrupts
    ESP_LOGI(TAG, "Configuring INT1 interrupt (GPIO%d)...", INT_PIN);
    drdy_queue = xQueueCreate(20, sizeof(int64_t)); // Larger queue
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // INT1 is active high
        .pin_bit_mask = (1ULL << INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull down when inactive
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_PIN, gpio_isr_handler, NULL);
    ESP_LOGI(TAG, "✓ GPIO interrupt configured");
    
    // Route gyro data-ready to INT1 pin
    lsm6dsv16x_pin_int_route_t int_route;
    memset(&int_route, 0, sizeof(int_route));
    int_route.drdy_g = 1;  // Gyro data ready on INT1
    int_route.drdy_xl = 1; // Also route accel
    rc = lsm6dsv16x_pin_int1_route_set(&dev_ctx, &int_route);
    ESP_LOGI(TAG, "✓ INT1 route configured: rc=%d", (int)rc);
    
    ESP_LOGI(TAG, "Configuration complete!\n");
    
    // Give sensor time to stabilize
    ESP_LOGI(TAG, "Waiting for sensor to stabilize...");
    platform_delay(200);
    ESP_LOGI(TAG, "✓ Sensor ready\n");
    
    ESP_LOGI(TAG, "Starting 120Hz interrupt-driven data acquisition...");
    ESP_LOGI(TAG, "Printing every 1 second (120 samples)\n");
    
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);
}