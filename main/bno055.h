#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

// I2C defaults
#define BNO055_I2C_DEFAULT_FREQ_HZ   (100000)
#define BNO055_SENSOR_ADDR_PRIMARY   (0x28)
#define BNO055_SENSOR_ADDR_SECONDARY (0x29)
#define BNO055_SENSOR_ADDR           (BNO055_SENSOR_ADDR_PRIMARY)

// Return codes
#define BNO055_SUCCESS  (0)
#define BNO055_ERROR    (-1)

// Page / register addresses
#define BNO055_PAGE_ID_ADDR                  (0x07)
#define BNO055_CHIP_ID_ADDR                  (0x00)
#define BNO055_ACCEL_DATA_X_LSB_ADDR         (0x08)
#define BNO055_MAG_DATA_X_LSB_ADDR           (0x0E)
#define BNO055_GYRO_DATA_X_LSB_ADDR          (0x14)
#define BNO055_EULER_H_LSB_ADDR              (0x1A)
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  (0x28)
#define BNO055_CALIB_STAT_ADDR               (0x35)
#define BNO055_UNIT_SEL_ADDR                 (0x3B)
#define BNO055_OPR_MODE_ADDR                 (0x3D)
#define BNO055_PWR_MODE_ADDR                 (0x3E)

// Calibration profile registers
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR       (0x55)

// General constants
#define BNO055_PAGE_ZERO                     (0x00)
#define BNO055_GEN_READ_WRITE_LENGTH         (1)

// UART protocol constants (for completeness)
#define BNO055_ACK_VALUE                     (0xEE)
#define BNO055_WRITE_SUCCESS                 (0x01)
#define BNO055_WRITE_FAIL                    (0x03)
#define BNO055_READ_SUCCESS                  (0xBB)

// Unit selection bits
#define BNO055_ACCEL_UNIT_MSQ                (0x00)
#define BNO055_ACCEL_UNIT_MG                 (0x01)
#define BNO055_GYRO_UNIT_DPS                 (0x00)
#define BNO055_GYRO_UNIT_RPS                 (0x01)
#define BNO055_EULER_UNIT_DEG                (0x00)
#define BNO055_EULER_UNIT_RAD                (0x01)
#define BNO055_TEMP_UNIT_CELSIUS             (0x00)
#define BNO055_TEMP_UNIT_FAHRENHEIT          (0x01)
#define BNO055_WINDOWS_ORIENTATION           (0x00)
#define BNO055_ANDROID_ORIENTATION           (0x01)

#define BNO055_CALIB_STAT_OK                 (0xFF)

typedef enum {
    CONFIGMODE      = 0x00,
    ACCONLY         = 0x01,
    MAGONLY         = 0x02,
    GYROONLY        = 0x03,
    ACCMAG          = 0x04,
    ACCGYRO         = 0x05,
    MAGGYRO         = 0x06,
    AMG             = 0x07,
    IMU             = 0x08,
    COMPASS         = 0x09,
    M4G             = 0x0A,
    NDOF_FMC_OFF    = 0x0B,
    NDOF            = 0x0C,
    INIT            = 0x0D
} BNO055_OperationMode;

typedef enum {
    NORMAL  = 0x00,
    LOWPOWER= 0x01,
    SUSPEND = 0x02
} BNO055_PowerMode;

typedef struct {
    uint8_t accel_unit;
    uint8_t gyro_unit;
    uint8_t euler_unit;
    uint8_t temp_unit;
    uint8_t ori_unit;
} BNO055_UnitSettings_t;

typedef struct {
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t accel_radius;
    uint16_t mag_radius;
} BNO055_CalibProfile_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t rst_gpio;
    bool has_reset_gpio;

    BNO055_OperationMode operation_mode;
    BNO055_PowerMode power_mode;
    BNO055_UnitSettings_t unit_settings;

    uint8_t chip_id;
    uint8_t sw_rev_id[2];
    uint8_t page_id;
    uint8_t accel_rev_id;
    uint8_t mag_rev_id;
    uint8_t gyro_rev_id;
    uint8_t bl_rev_id;

    uint8_t calib_stat;
    uint8_t test_stat;

    float yaw;
    float pitch;
    float roll;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

    float accel;
    float dir;
    bool is_calibrated;
} BNO055_t;

int8_t BNO055_Init(BNO055_t *bno055, i2c_port_t port, gpio_num_t scl, gpio_num_t sda, gpio_num_t rst_gpio);
void BNO055_Reset(BNO055_t *bno055);

int8_t BNO055_GetCalibrationStatus(BNO055_t *bno055);
int8_t BNO055_GetInfo(BNO055_t *bno055);

int8_t BNO055_SetOperationMode(BNO055_t *bno055, BNO055_OperationMode mode);
int8_t BNO055_SetPowerMode(BNO055_t *bno055, BNO055_PowerMode mode);
int8_t BNO055_SetUnit(BNO055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit);

void BNO055_GetEulerAngles(BNO055_t *bno055, float *yaw, float *pitch, float *roll);
void BNO055_GetAcceleration(BNO055_t *bno055, float *x, float *y, float *z);
void BNO055_GetGyro(BNO055_t *bno055, float *gx, float *gy, float *gz);
void BNO055_GetMagnetometer(BNO055_t *bno055, float *mx, float *my, float *mz);

int8_t BNO055_ReadAll(BNO055_t *bno055);
int8_t BNO055_ReadAll_Lineal(BNO055_t *bno055);

int8_t BN055_Write(BNO055_t *bno055, uint8_t reg, const uint8_t *data, uint8_t len);
int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

int8_t BNO055_CheckAck(const uint8_t *data);
int8_t BNO055_GetCalibrationProfile(BNO055_t *bno055, BNO055_CalibProfile_t *calib_data);

#ifdef __cplusplus
}
#endif
