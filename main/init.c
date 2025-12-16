#include "init.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAGI = "INIT";
static AS5600_t g_dev;
static AS5600_AnalogEncoder_t g_analog_encoders[AS5600_ANALOG_COUNT] = {
    {
        .gpio = AS5600_ANALOG0_GPIO,
        .channel = AS5600_ANALOG0_CHANNEL,
        .last_deg = 0.0f,
    },
    {
        .gpio = AS5600_ANALOG1_GPIO,
        .channel = AS5600_ANALOG1_CHANNEL,
        .last_deg = 0.0f,
    },
};

bool init_sensors(void)
{
    // Iniciar bus I2C
    if (!as5600_init_bus(&g_dev, AS5600_I2C_PORT, AS5600_I2C_SCL_PIN, AS5600_I2C_SDA_PIN, AS5600_I2C_FREQ_HZ)) {
        ESP_LOGE(TAGI, "I2C init failed");
        return false;
    }

    // Calibración volátil: 0..4095
    if (!as5600_calibrate_full_range(&g_dev)) {
        ESP_LOGE(TAGI, "AS5600 calibration failed");
        // deja el bus iniciado por si quieres seguir leyendo para debug
    }
    return true;
}

// Exponer el dev si lo necesitas en otros módulos
AS5600_t* get_as5600_dev(void) { return &g_dev; }

bool as5600_init_analog_encoders(void)
{
    static bool s_adc_configured = false;
    esp_err_t err = ESP_OK;

    if (!s_adc_configured) {
        err = adc1_config_width(ADC_WIDTH_BIT_12);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAGI, "adc1_config_width failed: %s", esp_err_to_name(err));
            return false;
        }
        s_adc_configured = true;
    }

    for (size_t i = 0; i < AS5600_ANALOG_COUNT; ++i) {
        err = adc1_config_channel_atten(g_analog_encoders[i].channel, ADC_ATTEN_DB_11);
        if (err != ESP_OK) {
            ESP_LOGE(TAGI, "adc1_config_channel_atten failed on idx %zu: %s", i, esp_err_to_name(err));
            return false;
        }
        g_analog_encoders[i].last_deg = 0.0f;
    }

    return true;
}

bool as5600_read_analog_deg(size_t index, float *deg_out)
{
    if (deg_out == NULL || index >= AS5600_ANALOG_COUNT) {
        return false;
    }

    const AS5600_AnalogEncoder_t *enc = &g_analog_encoders[index];
    int raw = adc1_get_raw(enc->channel);
    if (raw < 0) {
        return false;
    }

    const float ratio = (float)raw / 4095.0f;
    const float deg = ratio * 360.0f;
    g_analog_encoders[index].last_deg = deg;
    *deg_out = deg;
    return true;
}

const AS5600_AnalogEncoder_t *as5600_get_analog_encoder(size_t index)
{
    if (index >= AS5600_ANALOG_COUNT) {
        return NULL;
    }
    return &g_analog_encoders[index];
}
