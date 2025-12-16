#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

// ============ Dirección I2C ============
#ifndef AS5600_ADDR
#define AS5600_ADDR 0x36  // 7-bit
#endif

// ============ Registros clave ==========
#define AS5600_REG_ZPOS_H       0x01
#define AS5600_REG_MPOS_H       0x03
#define AS5600_REG_MANG_H       0x05
#define AS5600_REG_CONF_H       0x07
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE_H  0x1B
#define AS5600_REG_BURN         0xFF

// ============ STATUS bits ==============
#define AS5600_STATUS_MD   0x20  // Magnet detected
#define AS5600_STATUS_MH   0x08  // Too strong
#define AS5600_STATUS_ML   0x10  // Too weak

// ============ Estructuras ==============

typedef struct {
    i2c_port_t i2c_num;
    int gpio_scl;
    int gpio_sda;
} as5600_i2c_handle_t;

typedef struct {
    as5600_i2c_handle_t i2c_handle;
    int out_gpio;    // pin OUT si usas analógico/PWM, -1 si no se usa
} AS5600_t;

// Configuración (CONF). Para evitar problemas de bitfields/endianness en distintas toolchains,
// usa el campo WORD directamente. Los enums te ayudan a construir el WORD.
typedef union {
    uint16_t WORD;
    struct __attribute__((packed)) {
        uint16_t PM   : 2; // power mode
        uint16_t HYST : 2; // hysteresis
        uint16_t OUTS : 2; // output stage (solo pin OUT)
        uint16_t PWMF : 2; // PWM freq (solo pin OUT)
        uint16_t SF   : 2; // slow filter
        uint16_t FTH  : 3; // fast filter threshold
        uint16_t WD   : 1; // watchdog
        uint16_t RESERVED : 2;
    };
} AS5600_config_t;

// --- Valores típicos (ajusta a tu gusto) ---
typedef enum { // Power mode
    AS5600_POWER_MODE_NOM   = 0,
    AS5600_POWER_MODE_LPM1  = 1,
    AS5600_POWER_MODE_LPM2  = 2,
    AS5600_POWER_MODE_LPM3  = 3,
} AS5600_PM_t;

typedef enum { // Hysteresis
    AS5600_HYSTERESIS_OFF  = 0,
    AS5600_HYSTERESIS_1LSB = 1,
    AS5600_HYSTERESIS_2LSB = 2,
    AS5600_HYSTERESIS_3LSB = 3,
} AS5600_HYST_t;

typedef enum { // Output stage (pin OUT)
    AS5600_OUTPUT_STAGE_ANALOG_RR = 0, // 10%..90% Vcc (reduced range)
    AS5600_OUTPUT_STAGE_ANALOG_FR = 1, // 0..100% Vcc (full range)
    AS5600_OUTPUT_STAGE_PWM       = 2, // PWM
} AS5600_OUTS_t;

typedef enum { // PWM freq (pin OUT en modo PWM)
    AS5600_PWM_FREQUENCY_115HZ = 0,
    AS5600_PWM_FREQUENCY_230HZ = 1,
    AS5600_PWM_FREQUENCY_460HZ = 2,
    AS5600_PWM_FREQUENCY_920HZ = 3,
} AS5600_PWMF_t;

typedef enum { // Slow filter
    AS5600_SLOW_FILTER_16X = 0,
    AS5600_SLOW_FILTER_8X  = 1,
    AS5600_SLOW_FILTER_4X  = 2,
    AS5600_SLOW_FILTER_2X  = 3,
} AS5600_SF_t;

typedef enum { // Fast filter threshold
    AS5600_FF_THRESHOLD_OFF  = 0,
    AS5600_FF_THRESHOLD_6LSB = 1,
    AS5600_FF_THRESHOLD_7LSB = 2,
    AS5600_FF_THRESHOLD_9LSB = 3,
    AS5600_FF_THRESHOLD_18LSB= 4,
    AS5600_FF_THRESHOLD_21LSB= 5,
    AS5600_FF_THRESHOLD_24LSB= 6,
    AS5600_FF_THRESHOLD_10LSB= 7,
} AS5600_FTH_t;

typedef enum { // Watchdog
    AS5600_WATCHDOG_OFF = 0,
    AS5600_WATCHDOG_ON  = 1,
} AS5600_WD_t;

// ============ API pública ==============

// Bus I2C
bool as5600_init_bus(AS5600_t *dev, i2c_port_t i2c_num, int scl, int sda, int hz);
void as5600_deinit_bus(AS5600_t *dev);

// Lecturas básicas
bool as5600_get_status(AS5600_t *dev, uint8_t *status);
bool as5600_get_agc(AS5600_t *dev, uint8_t *agc);
bool as5600_get_magnitude(AS5600_t *dev, uint16_t *mag12);
bool as5600_get_raw_angle(AS5600_t *dev, uint16_t *raw12);   // 0..4095
bool as5600_get_angle_deg(AS5600_t *dev, float *deg);        // 0..360

// Config y ventana
bool as5600_set_conf_word(AS5600_t *dev, uint16_t conf_word);
bool as5600_get_conf_word(AS5600_t *dev, uint16_t *conf_word);
bool as5600_set_conf(AS5600_t *dev, AS5600_config_t conf);   // wrapper
bool as5600_get_conf(AS5600_t *dev, AS5600_config_t *conf);  // wrapper

bool as5600_set_zpos12(AS5600_t *dev, uint16_t zpos12);
bool as5600_get_zpos12(AS5600_t *dev, uint16_t *zpos12);
bool as5600_set_mpos12(AS5600_t *dev, uint16_t mpos12);
bool as5600_get_mpos12(AS5600_t *dev, uint16_t *mpos12);
bool as5600_set_max_angle12(AS5600_t *dev, uint16_t mang12);
bool as5600_get_max_angle12(AS5600_t *dev, uint16_t *mang12);

// Calibraciones (sin burn; volátiles)
bool as5600_calibrate_full_range(AS5600_t *dev);  // ZPOS=0, MPOS=4095
bool AS5600_Calibrate(AS5600_t *dev, AS5600_config_t conf, uint16_t start_position, uint16_t stop_position);
bool as5600_zero_here(AS5600_t *dev);             // Ajusta ZPOS=ANGLE actual, MPOS=ZPOS+4095 (full range)

// Utilidades
static inline uint16_t as5600_wrap12(int32_t x) {
    int32_t m = x % 4096;
    return (uint16_t)((m < 0) ? (m + 4096) : m);
}

#ifdef __cplusplus
}
#endif
