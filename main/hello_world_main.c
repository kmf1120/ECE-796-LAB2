#include "sdkconfig.h" // <--- Add this first!
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lsm6dsv16x_reg.h" // ST driver header included from components/lsm6dsv16x
#include <string.h>
#include <math.h>

/* Platform context and helpers for ST driver */
static stmdev_ctx_t dev_ctx;
typedef struct {
    i2c_port_t i2c_num;
    uint8_t i2c_addr; /* 7-bit */
} lsm6dsv16x_handle_t;
static lsm6dsv16x_handle_t lsm_handle;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    lsm6dsv16x_handle_t *h = (lsm6dsv16x_handle_t *)handle;
    uint8_t data[256];
    if (len + 1 > sizeof(data)) return -1;
    data[0] = reg;
    memcpy(&data[1], bufp, len);
    // Use pdMS_TO_TICKS directly
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

// Helper: parse SFLP rotation vector from FIFO 6 bytes -> qx,qy,qz (assumes 3x int16)
static void decode_sflp_rotation_vector(const uint8_t data[6], float *q0, float *q1, float *q2, float *q3)
{
    int16_t qx = (int16_t)((data[1] << 8) | data[0]);
    int16_t qy = (int16_t)((data[3] << 8) | data[2]);
    int16_t qz = (int16_t)((data[5] << 8) | data[4]);
    /* Assumption: raw values are signed Q14/Q15 style — use a conservative scale. Adjust if you know exact scale. */
    const float Q_SCALE = 16384.0f; /* typical Q14 */
    float x = qx / Q_SCALE;
    float y = qy / Q_SCALE;
    float z = qz / Q_SCALE;
    float w_sq = 1.0f - (x * x + y * y + z * z);
    float w = (w_sq > 0.0f) ? sqrtf(w_sq) : 0.0f;
    *q0 = w; *q1 = x; *q2 = y; *q3 = z;
}

// --- existing code continues ---

static const char *TAG = "imu";
static QueueHandle_t drdy_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int64_t ts = esp_timer_get_time(); // microseconds
    xQueueSendFromISR(drdy_queue, &ts, NULL);
}

static void imu_task(void *arg) {
    int64_t ts;
    uint64_t sample_cnt = 0;
    for (;;) {
        if (xQueueReceive(drdy_queue, &ts, portMAX_DELAY) == pdTRUE) {
            sample_cnt++;
            // Read accelerometer and gyroscope via library
            int16_t raw_acc[3] = {0};
            int16_t raw_g[3] = {0};
            float ax=0, ay=0, az=0;
            float gx=0, gy=0, gz=0;
            float q0=1, q1=0, q2=0, q3=0;

            // accel (raw -> mg -> m/s^2)
            if (lsm6dsv16x_acceleration_raw_get(&dev_ctx, raw_acc) == 0) {
                float ax_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[0]);
                float ay_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[1]);
                float az_mg = lsm6dsv16x_from_fs4_to_mg(raw_acc[2]);
                const float MG_TO_MS2 = 9.80665e-3f;
                ax = ax_mg * MG_TO_MS2;
                ay = ay_mg * MG_TO_MS2;
                az = az_mg * MG_TO_MS2;
            }

            // gyro (raw -> mdps -> dps)
            if (lsm6dsv16x_angular_rate_raw_get(&dev_ctx, raw_g) == 0) {
                float gx_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_g[0]);
                float gy_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_g[1]);
                float gz_mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_g[2]);
                gx = gx_mdps / 1000.0f;
                gy = gy_mdps / 1000.0f;
                gz = gz_mdps / 1000.0f;
            }

            // Try to read SFLP rotation vector from FIFO (tagged 6-byte item: qx,qy,qz)
            lsm6dsv16x_fifo_out_raw_t fifo_raw;
            if (lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &fifo_raw) == 0 && fifo_raw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
                decode_sflp_rotation_vector(fifo_raw.data, &q0, &q1, &q2, &q3);
            } else {
                // Fallback: read AH_QVAR (single value) if available, otherwise keep identity quaternion
                int16_t qraw;
                if (lsm6dsv16x_ah_qvar_raw_get(&dev_ctx, &qraw) == 0) {
                    float qx = qraw / 16384.0f;
                    q1 = qx; q2 = 0.0f; q3 = 0.0f;
                    float w_sq = 1.0f - (q1*q1 + q2*q2 + q3*q3);
                    q0 = (w_sq > 0.0f) ? sqrtf(w_sq) : 0.0f;
                }
            }

            // Convert quaternion to Euler (rad -> deg)
            float roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
            float pitch = asinf(2*(q0*q2 - q3*q1));
            float yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
            const float RAD_TO_DEG = 57.29577951308232f;

            ESP_LOGI(TAG, "#%llu ts=%lld us, accel=[%.3f,%.3f,%.3f] m/s^2, gyro=[%.3f,%.3f,%.3f] dps, euler=[%.2f,%.2f,%.2f]",
                     sample_cnt, ts,
                     ax, ay, az, gx, gy, gz,
                     roll*RAD_TO_DEG, pitch*RAD_TO_DEG, yaw*RAD_TO_DEG);
        }
    }
}

void app_main(void) {
    // I2C configuration - adjust pins to your board wiring if needed
    const i2c_port_t I2C_NUM = I2C_NUM_0;
    const gpio_num_t SDA_PIN = GPIO_NUM_8;
    const gpio_num_t SCL_PIN = GPIO_NUM_9;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 400000},
    };
    i2c_param_config(I2C_NUM, &i2c_conf);
    i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);

    // prepare device context for ST driver
    lsm_handle.i2c_num = I2C_NUM;
    lsm_handle.i2c_addr = (uint8_t)(LSM6DSV16X_I2C_ADD_L >> 1); // convert 8-bit to 7-bit

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &lsm_handle;

    uint8_t whoamI = 0;
    if (lsm6dsv16x_device_id_get(&dev_ctx, &whoamI) != 0 || whoamI != LSM6DSV16X_ID) {
        ESP_LOGE(TAG, "LSM6DSV16X not found (whoami=0x%02x)", whoamI);
    } else {
        ESP_LOGI(TAG, "Found LSM6DSV16X (0x%02x)", whoamI);
    }

    // Basic configuration: enable block data update and set scales/ODR.
    lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_1000dps);

    // Use HAODR variant - HA mode options are available; use HA01 ~125Hz for high accuracy (~120Hz requirement)
    lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_HA01_AT_125Hz);
    lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_HA01_AT_125Hz);

    // Enable SFLP (game rotation vector) and set its ODR to 120Hz
    lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, 1);
    lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_120Hz);

    // Ask FIFO to batch SFLP game rotation values (so we can read rotation vector from FIFO)
    lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, (lsm6dsv16x_fifo_sflp_raw_t){.game_rotation = 1, .gravity = 0, .gbias = 0});

    // Route gyroscope data-ready to INT1 pin so HW interrupt fires when new sample is ready
    lsm6dsv16x_pin_int_route_t int_route;
    memset(&int_route, 0, sizeof(int_route));
    int_route.drdy_g = 1; // route gyro DRDY to INT1
    lsm6dsv16x_pin_int1_route_set(&dev_ctx, &int_route);

    // Latch embedded interrupt output so it stays asserted until driver clears/status read
    lsm6dsv16x_embedded_int_cfg_set(&dev_ctx, LSM6DSV16X_INT_LATCH_ENABLE);

    // Setup ESP GPIO for INT pin and queue to capture timestamps in ISR
    drdy_queue = xQueueCreate(10, sizeof(int64_t));
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // <--- Fixed name (removed _PIN_)
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_4, gpio_isr_handler, NULL);

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}