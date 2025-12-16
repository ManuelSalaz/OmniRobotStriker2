#include "bno055.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BNO055";

static esp_err_t bno055_ensure_bus(i2c_port_t port, gpio_num_t scl, gpio_num_t sda);
static esp_err_t bno055_write_bytes(BNO055_t *bno055, uint8_t reg, const uint8_t *data, size_t len);
static esp_err_t bno055_read_bytes(BNO055_t *bno055, uint8_t reg, uint8_t *data, size_t len);
static void bno055_convert_accel(const BNO055_t *bno055, const uint8_t *data, float *x, float *y, float *z);
static void bno055_convert_gyro(const BNO055_t *bno055, const uint8_t *data, float *x, float *y, float *z);
static void bno055_convert_euler(const BNO055_t *bno055, const uint8_t *data, float *yaw, float *pitch, float *roll);
static void bno055_convert_mag(const uint8_t *data, float *x, float *y, float *z);

int8_t BNO055_Init(BNO055_t *bno055, i2c_port_t port, gpio_num_t scl, gpio_num_t sda, gpio_num_t rst_gpio)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_Init");
        return BNO055_ERROR;
    }

    memset(bno055, 0, sizeof(*bno055));
    bno055->i2c_port = port;
    bno055->i2c_addr = BNO055_SENSOR_ADDR;
    bno055->operation_mode = INIT;
    bno055->power_mode = SUSPEND;
    bno055->unit_settings.accel_unit = BNO055_ACCEL_UNIT_MSQ;
    bno055->unit_settings.gyro_unit = BNO055_GYRO_UNIT_DPS;
    bno055->unit_settings.euler_unit = BNO055_EULER_UNIT_RAD;
    bno055->unit_settings.temp_unit = BNO055_TEMP_UNIT_CELSIUS;
    bno055->unit_settings.ori_unit = BNO055_ANDROID_ORIENTATION;

    if (rst_gpio >= 0 && rst_gpio < GPIO_NUM_MAX) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << rst_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&io_conf) == ESP_OK) {
            bno055->has_reset_gpio = true;
            bno055->rst_gpio = rst_gpio;
            gpio_set_level(rst_gpio, 1);
        } else {
            ESP_LOGW(TAG, "Failed to configure reset GPIO %d", rst_gpio);
        }
    }

    esp_err_t err = bno055_ensure_bus(port, scl, sda);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure I2C configuration: %s", esp_err_to_name(err));
        return BNO055_ERROR;
    }

    uint8_t probe_id = 0;
    if (bno055_read_bytes(bno055, BNO055_CHIP_ID_ADDR, &probe_id, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to read CHIP_ID (addr=0x%02X)", bno055->i2c_addr);
        return BNO055_ERROR;
    }
    ESP_LOGI(TAG, "BNO055 detected with CHIP_ID=0x%02X", probe_id);

    vTaskDelay(pdMS_TO_TICKS(650)); // Allow sensor boot time

    uint8_t page = BNO055_PAGE_ZERO;
    if (BN055_Write(bno055, BNO055_PAGE_ID_ADDR, &page, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Unable to select page 0");
        return BNO055_ERROR;
    }

    if (BNO055_SetOperationMode(bno055, CONFIGMODE) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enter CONFIGMODE");
        return BNO055_ERROR;
    }

    if (BNO055_SetUnit(bno055,
                       bno055->unit_settings.accel_unit,
                       bno055->unit_settings.gyro_unit,
                       bno055->unit_settings.euler_unit,
                       bno055->unit_settings.temp_unit,
                       bno055->unit_settings.ori_unit) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set unit selection register");
        return BNO055_ERROR;
    }

    if (BNO055_SetPowerMode(bno055, NORMAL) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set power mode");
        return BNO055_ERROR;
    }

    if (BNO055_SetOperationMode(bno055, NDOF) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enter NDOF mode");
        return BNO055_ERROR;
    }

    if (BNO055_GetInfo(bno055) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read sensor info");
        return BNO055_ERROR;
    }

    ESP_LOGI(TAG, "BNO055 initialized. Chip ID: 0x%02X, SW: %02X%02X, OpMode: 0x%02X",
             bno055->chip_id, bno055->sw_rev_id[0], bno055->sw_rev_id[1], bno055->operation_mode);
    return BNO055_SUCCESS;
}

void BNO055_Reset(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_Reset");
        return;
    }
    if (!bno055->has_reset_gpio) {
        ESP_LOGW(TAG, "Reset GPIO not configured; skipping hardware reset");
        return;
    }

    gpio_set_level(bno055->rst_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(bno055->rst_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int8_t BNO055_GetCalibrationStatus(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetCalibrationStatus");
        return BNO055_ERROR;
    }

    uint8_t calib_status = 0;
    if (BNO055_Read(bno055, BNO055_CALIB_STAT_ADDR, &calib_status, 1) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read calibration status");
        return BNO055_ERROR;
    }

    bno055->calib_stat = calib_status;
    bno055->is_calibrated = (calib_status == BNO055_CALIB_STAT_OK);

    ESP_LOGD(TAG, "Calibration status: 0x%02X", calib_status);
    return BNO055_SUCCESS;
}

int8_t BNO055_GetInfo(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetInfo");
        return BNO055_ERROR;
    }

    uint8_t data[8] = {0};
    if (BNO055_Read(bno055, BNO055_CHIP_ID_ADDR, data, sizeof(data)) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read BNO055 info block");
        return BNO055_ERROR;
    }

    bno055->chip_id = data[0];
    bno055->accel_rev_id = data[1];
    bno055->mag_rev_id = data[2];
    bno055->gyro_rev_id = data[3];
    bno055->sw_rev_id[0] = data[4];
    bno055->sw_rev_id[1] = data[5];
    bno055->bl_rev_id = data[6];
    bno055->page_id = data[7];

    return BNO055_SUCCESS;
}

int8_t BNO055_SetOperationMode(BNO055_t *bno055, BNO055_OperationMode mode)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_SetOperationMode");
        return BNO055_ERROR;
    }

    if (mode == bno055->operation_mode) {
        return BNO055_SUCCESS;
    }

    uint8_t data = (uint8_t)mode;
    if (BN055_Write(bno055, BNO055_OPR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to write operation mode 0x%02X", mode);
        return BNO055_ERROR;
    }

    bno055->operation_mode = mode;
    vTaskDelay(pdMS_TO_TICKS((mode == CONFIGMODE) ? 25 : 35));
    return BNO055_SUCCESS;
}

int8_t BNO055_SetPowerMode(BNO055_t *bno055, BNO055_PowerMode mode)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_SetPowerMode");
        return BNO055_ERROR;
    }

    if (mode == bno055->power_mode) {
        return BNO055_SUCCESS;
    }

    uint8_t data = (uint8_t)mode;
    if (BN055_Write(bno055, BNO055_PWR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set power mode 0x%02X", mode);
        return BNO055_ERROR;
    }

    bno055->power_mode = mode;
    vTaskDelay(pdMS_TO_TICKS(20));
    return BNO055_SUCCESS;
}

int8_t BNO055_SetUnit(BNO055_t *bno055,
                      uint8_t accel_unit,
                      uint8_t gyro_unit,
                      uint8_t euler_unit,
                      uint8_t temp_unit,
                      uint8_t ori_unit)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_SetUnit");
        return BNO055_ERROR;
    }

    uint8_t value = 0;
    value |= (accel_unit & 0x01) << 0;
    value |= (gyro_unit & 0x01) << 1;
    value |= (euler_unit & 0x01) << 2;
    value |= (temp_unit & 0x01) << 4;
    value |= (ori_unit & 0x01) << 7;

    if (BN055_Write(bno055, BNO055_UNIT_SEL_ADDR, &value, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to write UNIT_SEL register");
        return BNO055_ERROR;
    }

    bno055->unit_settings.accel_unit = accel_unit & 0x01;
    bno055->unit_settings.gyro_unit = gyro_unit & 0x01;
    bno055->unit_settings.euler_unit = euler_unit & 0x01;
    bno055->unit_settings.temp_unit = temp_unit & 0x01;
    bno055->unit_settings.ori_unit = ori_unit & 0x01;

    return BNO055_SUCCESS;
}

void BNO055_GetEulerAngles(BNO055_t *bno055, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetEulerAngles");
        return;
    }
    if (yaw == NULL || pitch == NULL || roll == NULL) {
        ESP_LOGE(TAG, "Null output pointer passed to BNO055_GetEulerAngles");
        return;
    }

    uint8_t data[6] = {0};
    if (BNO055_Read(bno055, BNO055_EULER_H_LSB_ADDR, data, sizeof(data)) != BNO055_SUCCESS) {
        *yaw = bno055->yaw;
        *pitch = bno055->pitch;
        *roll = bno055->roll;
        return;
    }

    bno055_convert_euler(bno055, data, yaw, pitch, roll);

    bno055->yaw = *yaw;
    bno055->pitch = *pitch;
    bno055->roll = *roll;
}

void BNO055_GetAcceleration(BNO055_t *bno055, float *x, float *y, float *z)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetAcceleration");
        return;
    }
    if (x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "Null output pointer passed to BNO055_GetAcceleration");
        return;
    }

    uint8_t data[6] = {0};
    if (BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, sizeof(data)) != BNO055_SUCCESS) {
        *x = bno055->ax;
        *y = bno055->ay;
        *z = bno055->az;
        return;
    }

    bno055_convert_accel(bno055, data, x, y, z);
    bno055->ax = *x;
    bno055->ay = *y;
    bno055->az = *z;
}

void BNO055_GetGyro(BNO055_t *bno055, float *gx, float *gy, float *gz)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetGyro");
        return;
    }
    if (gx == NULL || gy == NULL || gz == NULL) {
        ESP_LOGE(TAG, "Null output pointer passed to BNO055_GetGyro");
        return;
    }

    uint8_t data[6] = {0};
    if (BNO055_Read(bno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, sizeof(data)) != BNO055_SUCCESS) {
        *gx = bno055->gx;
        *gy = bno055->gy;
        *gz = bno055->gz;
        return;
    }

    bno055_convert_gyro(bno055, data, gx, gy, gz);
    bno055->gx = *gx;
    bno055->gy = *gy;
    bno055->gz = *gz;
}

void BNO055_GetMagnetometer(BNO055_t *bno055, float *mx, float *my, float *mz)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_GetMagnetometer");
        return;
    }
    if (mx == NULL || my == NULL || mz == NULL) {
        ESP_LOGE(TAG, "Null output pointer passed to BNO055_GetMagnetometer");
        return;
    }

    uint8_t data[6] = {0};
    if (BNO055_Read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, data, sizeof(data)) != BNO055_SUCCESS) {
        *mx = bno055->mx;
        *my = bno055->my;
        *mz = bno055->mz;
        return;
    }

    bno055_convert_mag(data, mx, my, mz);
    bno055->mx = *mx;
    bno055->my = *my;
    bno055->mz = *mz;
}

int8_t BNO055_ReadAll(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_ReadAll");
        return BNO055_ERROR;
    }

    uint8_t raw[24] = {0};
    if (BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, raw, sizeof(raw)) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Bulk read failed");
        return BNO055_ERROR;
    }

    bno055_convert_accel(bno055, &raw[0], &bno055->ax, &bno055->ay, &bno055->az);
    bno055_convert_mag(&raw[6], &bno055->mx, &bno055->my, &bno055->mz);
    bno055_convert_gyro(bno055, &raw[12], &bno055->gx, &bno055->gy, &bno055->gz);
    bno055_convert_euler(bno055, &raw[18], &bno055->yaw, &bno055->pitch, &bno055->roll);

    bno055->accel = sqrtf((bno055->ax * bno055->ax) + (bno055->ay * bno055->ay));
    bno055->dir = atan2f(bno055->ay, bno055->ax);

    return BNO055_SUCCESS;
}

int8_t BNO055_ReadAll_Lineal(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null handle passed to BNO055_ReadAll_Lineal");
        return BNO055_ERROR;
    }

    uint8_t raw[24] = {0};
    if (BNO055_Read(bno055, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, raw, 6) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read linear acceleration block");
        return BNO055_ERROR;
    }
    if (BNO055_Read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, raw + 6, 18) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read remainder of sensor block");
        return BNO055_ERROR;
    }

    bno055_convert_accel(bno055, &raw[0], &bno055->ax, &bno055->ay, &bno055->az);
    bno055_convert_mag(&raw[6], &bno055->mx, &bno055->my, &bno055->mz);
    bno055_convert_gyro(bno055, &raw[12], &bno055->gx, &bno055->gy, &bno055->gz);
    bno055_convert_euler(bno055, &raw[18], &bno055->yaw, &bno055->pitch, &bno055->roll);

    bno055->accel = sqrtf((bno055->ax * bno055->ax) + (bno055->ay * bno055->ay));
    bno055->dir = atan2f(bno055->ay, bno055->ax);

    return BNO055_SUCCESS;
}

int8_t BN055_Write(BNO055_t *bno055, uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGE(TAG, "Invalid argument to BN055_Write");
        return BNO055_ERROR;
    }

    if (bno055_write_bytes(bno055, reg, data, len) != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed (reg=0x%02X)", reg);
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGE(TAG, "Invalid argument to BNO055_Read");
        return BNO055_ERROR;
    }

    if (bno055_read_bytes(bno055, reg, data, len) != ESP_OK) {
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

int8_t BNO055_CheckAck(const uint8_t *data)
{
    if (data == NULL) {
        return BNO055_ERROR;
    }
    if (data[0] == BNO055_ACK_VALUE) {
        return (data[1] == BNO055_WRITE_SUCCESS) ? BNO055_SUCCESS : BNO055_ERROR;
    }
    return BNO055_ERROR;
}

int8_t BNO055_GetCalibrationProfile(BNO055_t *bno055, BNO055_CalibProfile_t *calib_data)
{
    if (bno055 == NULL || calib_data == NULL) {
        ESP_LOGE(TAG, "Invalid argument to BNO055_GetCalibrationProfile");
        return BNO055_ERROR;
    }

    BNO055_OperationMode previous_mode = bno055->operation_mode;
    if (BNO055_SetOperationMode(bno055, CONFIGMODE) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Unable to switch to CONFIGMODE for calibration dump");
        return BNO055_ERROR;
    }

    uint8_t raw[22] = {0};
    if (BNO055_Read(bno055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, raw, sizeof(raw)) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read calibration data block");
        BNO055_SetOperationMode(bno055, previous_mode);
        return BNO055_ERROR;
    }

    calib_data->accel_offset_x = (int16_t)((raw[1] << 8) | raw[0]);
    calib_data->accel_offset_y = (int16_t)((raw[3] << 8) | raw[2]);
    calib_data->accel_offset_z = (int16_t)((raw[5] << 8) | raw[4]);

    calib_data->mag_offset_x = (int16_t)((raw[7] << 8) | raw[6]);
    calib_data->mag_offset_y = (int16_t)((raw[9] << 8) | raw[8]);
    calib_data->mag_offset_z = (int16_t)((raw[11] << 8) | raw[10]);

    calib_data->gyro_offset_x = (int16_t)((raw[13] << 8) | raw[12]);
    calib_data->gyro_offset_y = (int16_t)((raw[15] << 8) | raw[14]);
    calib_data->gyro_offset_z = (int16_t)((raw[17] << 8) | raw[16]);

    calib_data->accel_radius = (int16_t)((raw[19] << 8) | raw[18]);
    calib_data->mag_radius = (int16_t)((raw[21] << 8) | raw[20]);

    if (BNO055_SetOperationMode(bno055, previous_mode) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to restore operation mode after calibration dump");
    }

    ESP_LOGI(TAG, "Calibration profile: "
                  "Accel offsets [%d, %d, %d], Mag offsets [%d, %d, %d], "
                  "Gyro offsets [%d, %d, %d], Radii [accel=%d, mag=%d]",
             (int16_t)calib_data->accel_offset_x,
             (int16_t)calib_data->accel_offset_y,
             (int16_t)calib_data->accel_offset_z,
             (int16_t)calib_data->mag_offset_x,
             (int16_t)calib_data->mag_offset_y,
             (int16_t)calib_data->mag_offset_z,
             (int16_t)calib_data->gyro_offset_x,
             (int16_t)calib_data->gyro_offset_y,
             (int16_t)calib_data->gyro_offset_z,
             (int16_t)calib_data->accel_radius,
             (int16_t)calib_data->mag_radius);

    return BNO055_SUCCESS;
}

// ==== Static helpers =======================================================

static esp_err_t bno055_ensure_bus(i2c_port_t port, gpio_num_t scl, gpio_num_t sda)
{
    if (scl == GPIO_NUM_NC || sda == GPIO_NUM_NC) {
        return ESP_OK;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BNO055_I2C_DEFAULT_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(port, &cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    return err;
}

static esp_err_t bno055_write_bytes(BNO055_t *bno055, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buffer[32];
    if (len + 1 > sizeof(buffer)) {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    return i2c_master_write_to_device(bno055->i2c_port, bno055->i2c_addr, buffer, len + 1, pdMS_TO_TICKS(50));
}

static esp_err_t bno055_read_bytes(BNO055_t *bno055, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(bno055->i2c_port,
                                        bno055->i2c_addr,
                                        &reg,
                                        1,
                                        data,
                                        len,
                                        pdMS_TO_TICKS(50));
}

static void bno055_convert_accel(const BNO055_t *bno055, const uint8_t *data, float *x, float *y, float *z)
{
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    const float divisor = (bno055->unit_settings.accel_unit == BNO055_ACCEL_UNIT_MSQ) ? 100.0f : 1.0f;

    *x = raw_x / divisor;
    *y = raw_y / divisor;
    *z = raw_z / divisor;
}

static void bno055_convert_gyro(const BNO055_t *bno055, const uint8_t *data, float *x, float *y, float *z)
{
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    const float divisor = (bno055->unit_settings.gyro_unit == BNO055_GYRO_UNIT_DPS) ? 16.0f : 900.0f;

    *x = raw_x / divisor;
    *y = raw_y / divisor;
    *z = raw_z / divisor;
}

static void bno055_convert_euler(const BNO055_t *bno055, const uint8_t *data, float *yaw, float *pitch, float *roll)
{
    int16_t raw_yaw = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_pitch = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_roll = (int16_t)((data[5] << 8) | data[4]);

    if (bno055->unit_settings.euler_unit == BNO055_EULER_UNIT_DEG) {
        *yaw = raw_yaw / 16.0f;
        *pitch = raw_pitch / 16.0f;
        *roll = raw_roll / 16.0f;
    } else {
        *yaw = raw_yaw / 900.0f;
        *pitch = raw_pitch / 900.0f;
        *roll = raw_roll / 900.0f;
    }
}

static void bno055_convert_mag(const uint8_t *data, float *x, float *y, float *z)
{
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    *x = raw_x / 16.0f; // 1 LSB = 1/16 uT
    *y = raw_y / 16.0f;
    *z = raw_z / 16.0f;
}
