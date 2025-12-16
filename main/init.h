#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "as5600_lib.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// ==== PINES I2C (ajusta a tu hardware) ====
#define AS5600_I2C_PORT      1   // equivalente a I2C_NUM_1
#define AS5600_I2C_SCL_PIN   5   // GPIO5 (ESP32-S3: pin libre, no strapping)
#define AS5600_I2C_SDA_PIN   4   // GPIO4 (ESP32-S3: pin libre, no strapping)
#define AS5600_I2C_FREQ_HZ   400000

// ==== Encoders analógicos (salida OUT) ====
#define AS5600_ANALOG_COUNT      2
#define AS5600_ANALOG0_GPIO      GPIO_NUM_2   // Ajusta según tu conexión física
#define AS5600_ANALOG0_CHANNEL   ADC1_CHANNEL_1
#define AS5600_ANALOG1_GPIO      GPIO_NUM_3
#define AS5600_ANALOG1_CHANNEL   ADC1_CHANNEL_2

typedef struct {
    gpio_num_t gpio;
    adc1_channel_t channel;
    float last_deg;
} AS5600_AnalogEncoder_t;

// Init “alto nivel” del sensor
bool init_sensors(void);
AS5600_t *get_as5600_dev(void);

bool as5600_init_analog_encoders(void);
bool as5600_read_analog_deg(size_t index, float *deg_out);
const AS5600_AnalogEncoder_t *as5600_get_analog_encoder(size_t index);
