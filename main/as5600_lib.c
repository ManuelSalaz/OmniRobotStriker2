#include "as5600_lib.h"
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "AS5600";

// ============ Helpers I2C (ESP-IDF) ============

static esp_err_t i2c_write_u8_u8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t i2c_read_u8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t i2c_read_u16_be(i2c_port_t port, uint8_t addr, uint8_t reg_hi, uint16_t *val)
{
    uint8_t hi=0, lo=0;
    esp_err_t r = i2c_read_u8(port, addr, reg_hi, &hi);
    if (r != ESP_OK) return r;
    r = i2c_read_u8(port, addr, reg_hi + 1, &lo);
    if (r != ESP_OK) return r;
    *val = ((uint16_t)hi << 8) | lo; // registros AS5600 big-endian
    return ESP_OK;
}

static esp_err_t i2c_write_u16_be(i2c_port_t port, uint8_t addr, uint8_t reg_hi, uint16_t val)
{
    // envío: reg_hi, MSB, LSB
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_hi, true);
    i2c_master_write_byte(cmd, (val >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd,  val       & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

// ============ Bus I2C ============

bool as5600_init_bus(AS5600_t *dev, i2c_port_t i2c_num, int scl, int sda, int hz)
{
    dev->i2c_handle.i2c_num = i2c_num;
    dev->i2c_handle.gpio_scl = scl;
    dev->i2c_handle.gpio_sda = sda;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = hz,
    };
    esp_err_t r = i2c_param_config(i2c_num, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(r));
        return false;
    }
    r = i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(r));
        return false;
    }
    return true;
}

void as5600_deinit_bus(AS5600_t *dev)
{
    i2c_driver_delete(dev->i2c_handle.i2c_num);
}

// ============ Lecturas básicas ============

bool as5600_get_status(AS5600_t *dev, uint8_t *status)
{
    return i2c_read_u8(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_STATUS, status) == ESP_OK;
}

bool as5600_get_agc(AS5600_t *dev, uint8_t *agc)
{
    return i2c_read_u8(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_AGC, agc) == ESP_OK;
}

bool as5600_get_magnitude(AS5600_t *dev, uint16_t *mag12)
{
    uint16_t v=0;
    if (i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_MAGNITUDE_H, &v) != ESP_OK) return false;
    *mag12 = v & 0x0FFF;
    return true;
}

bool as5600_get_raw_angle(AS5600_t *dev, uint16_t *raw12)
{
    uint16_t v=0;
    if (i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_RAW_ANGLE_H, &v) != ESP_OK) return false;
    *raw12 = v & 0x0FFF;
    return true;
}

bool as5600_get_angle_deg(AS5600_t *dev, float *deg)
{
    uint16_t raw=0;
    if (!as5600_get_raw_angle(dev, &raw)) return false;
    *deg = (raw * 360.0f) / 4096.0f;
    return true;
}

// ============ Config y ventana ============

bool as5600_set_conf_word(AS5600_t *dev, uint16_t conf_word)
{
    return i2c_write_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_CONF_H, conf_word) == ESP_OK;
}

bool as5600_get_conf_word(AS5600_t *dev, uint16_t *conf_word)
{
    return i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_CONF_H, conf_word) == ESP_OK;
}

bool as5600_set_conf(AS5600_t *dev, AS5600_config_t conf)
{
    return as5600_set_conf_word(dev, conf.WORD);
}

bool as5600_get_conf(AS5600_t *dev, AS5600_config_t *conf)
{
    return as5600_get_conf_word(dev, &conf->WORD);
}

bool as5600_set_zpos12(AS5600_t *dev, uint16_t zpos12)
{
    zpos12 &= 0x0FFF;
    return i2c_write_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_ZPOS_H, zpos12) == ESP_OK;
}

bool as5600_get_zpos12(AS5600_t *dev, uint16_t *zpos12)
{
    if (i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_ZPOS_H, zpos12) != ESP_OK) return false;
    *zpos12 &= 0x0FFF;
    return true;
}

bool as5600_set_mpos12(AS5600_t *dev, uint16_t mpos12)
{
    mpos12 &= 0x0FFF;
    return i2c_write_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_MPOS_H, mpos12) == ESP_OK;
}

bool as5600_get_mpos12(AS5600_t *dev, uint16_t *mpos12)
{
    if (i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_MPOS_H, mpos12) != ESP_OK) return false;
    *mpos12 &= 0x0FFF;
    return true;
}

bool as5600_set_max_angle12(AS5600_t *dev, uint16_t mang12)
{
    mang12 &= 0x0FFF;
    return i2c_write_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_MANG_H, mang12) == ESP_OK;
}

bool as5600_get_max_angle12(AS5600_t *dev, uint16_t *mang12)
{
    if (i2c_read_u16_be(dev->i2c_handle.i2c_num, AS5600_ADDR, AS5600_REG_MANG_H, mang12) != ESP_OK) return false;
    *mang12 &= 0x0FFF;
    return true;
}

// ============ Calibraciones (sin burn) ============

bool as5600_calibrate_full_range(AS5600_t *dev)
{
    // 1) magnet detect
    uint8_t st=0;
    if (!as5600_get_status(dev, &st) || (st & AS5600_STATUS_MD) == 0) {
        ESP_LOGE(TAG, "Magnet not detected (STATUS=0x%02X)", st);
        return false;
    }

    // 2) ZPOS=0, MPOS=4095
    if (!as5600_set_zpos12(dev, 0x0000)) return false;
    if (!as5600_set_mpos12(dev, 0x0FFF)) return false;

    // 3) verificación
    uint16_t z=0, m=0;
    if (!as5600_get_zpos12(dev, &z)) return false;
    if (!as5600_get_mpos12(dev, &m)) return false;

    bool ok = (z == 0x0000) && (m == 0x0FFF);
    ESP_LOGI(TAG, "Calib(full) ZPOS=0x%03X MPOS=0x%03X => %s", z, m, ok?"OK":"FAIL");
    return ok;
}

// API compatible con tu versión previa: escribe CONF + ventana start/stop y verifica.
// NO realiza burn. Requiere que el bus esté ya inicializado.
bool AS5600_Calibrate(AS5600_t *dev, AS5600_config_t conf, uint16_t start_position, uint16_t stop_position)
{
    if (start_position > 4095 || stop_position > 4095 || start_position >= stop_position) {
        ESP_LOGE(TAG, "Invalid window: start=%u stop=%u", start_position, stop_position);
        return false;
    }

    uint8_t st = 0;
    if (!as5600_get_status(dev, &st) || (st & AS5600_STATUS_MD) == 0) {
        ESP_LOGE(TAG, "Magnet not detected (STATUS=0x%02X)", st);
        return false;
    }

    bool ok = true;

    // 1) Config (opcional para I2C, pero tu API la pide)
    ok &= as5600_set_conf(dev, conf);

    // 2) Ventana
    ok &= as5600_set_zpos12(dev, start_position);
    ok &= as5600_set_mpos12(dev, stop_position);

    // 3) Verificación
    uint16_t rd_conf=0, z=0, m=0;
    ok &= as5600_get_conf_word(dev, &rd_conf);
    ok &= as5600_get_zpos12(dev, &z);
    ok &= as5600_get_mpos12(dev, &m);

    ESP_LOGI(TAG, "CONF wrote:0x%04X read:0x%04X | ZPOS:0x%03X MPOS:0x%03X",
             conf.WORD, rd_conf, z, m);

    if (!ok || rd_conf != conf.WORD || z != start_position || m != stop_position) {
        return false;
    }
    return true;
}

// Ajusta el cero a la posición actual (volatile, sin burn):
// pone ZPOS = ANGLE_ACTUAL y MPOS = ZPOS + 4095 (ventana completa)
bool as5600_zero_here(AS5600_t *dev)
{
    uint8_t st=0;
    if (!as5600_get_status(dev, &st) || (st & AS5600_STATUS_MD) == 0) {
        ESP_LOGE(TAG, "Magnet not detected (STATUS=0x%02X)", st);
        return false;
    }

    uint16_t raw=0;
    if (!as5600_get_raw_angle(dev, &raw)) return false;

    uint16_t z = raw & 0x0FFF;
    uint16_t m = (z + 0x0FFF) & 0x0FFF; // wrap 12 bits

    if (!as5600_set_zpos12(dev, z)) return false;
    if (!as5600_set_mpos12(dev, m)) return false;

    uint16_t zr=0, mr=0;
    if (!as5600_get_zpos12(dev, &zr)) return false;
    if (!as5600_get_mpos12(dev, &mr)) return false;

    bool ok = (zr == z) && (mr == m);
    ESP_LOGI(TAG, "Zero-here @raw=0x%03X -> ZPOS=0x%03X MPOS=0x%03X => %s", raw, zr, mr, ok?"OK":"FAIL");
    return ok;
}
