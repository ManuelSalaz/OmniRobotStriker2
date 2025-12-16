#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "as5600_lib.h"
#include "init.h"
#include "bldc_pwm.h"
#include "bno055.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"


typedef struct {
    float angle_deg;
    float rpm;
    uint8_t status;
    uint8_t agc;
    uint16_t mag;
} encoder_data_t;

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} pi_ctrl_t;

typedef struct {
    float vx;
    float vy;
    float wz;
    int64_t end_time_us;
    bool active;
} motion_cmd_t;

static motion_cmd_t motion_cmd = {
    .vx = 0.0f,
    .vy = 0.0f,
    .wz = 0.0f,
    .end_time_us = 0,
    .active = false
};

static portMUX_TYPE motion_mux = portMUX_INITIALIZER_UNLOCKED;

static void apply_motion_command(float speed, float dir_deg, uint32_t time_ms);



static encoder_data_t encoder_data;
static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

static float prev_ref[3] = {0};

static const char *TAG = "MAIN";

static float ana_rpm_valid[AS5600_ANALOG_COUNT] = {0};

static float yaw_target = 0.0f;
static bool heading_hold_enabled = true;



static bldc_pwm_motor_t motor1;
static bldc_pwm_motor_t motor2;
static bldc_pwm_motor_t motor3;

static pi_ctrl_t pi_m1 = { .kp=0.35f, .ki=0.35f, .out_min=8.0f, .out_max=100.0f };
static pi_ctrl_t pi_m2 = { .kp=0.28f, .ki=0.35f, .out_min=8.0f, .out_max=100.0f };
static pi_ctrl_t pi_m3 = { .kp=0.22f, .ki=0.25f, .out_min=10.0f, .out_max=95.0f};


static float rpm_ref[3] = {0.0f, 0.0f, 0.0f};

static portMUX_TYPE ana_mux = portMUX_INITIALIZER_UNLOCKED;

static float ana_angle[AS5600_ANALOG_COUNT] = {0};
static float ana_rpm[AS5600_ANALOG_COUNT]   = {0};

static float vx_cmd = 0.0f;   // adelante (+) / atrás (-)
static float vy_cmd = 0.0f;   // izquierda (+) / derecha (-)
static float wz_cmd = 0.0f;   // giro CCW (+)

static float wheel_gain[3] = {
    1.00f,  // M1
    1.00f,  // M2
    1.10f   // M3
};

#define RPM_ALPHA 0.1f
#define M1_PWM_GPIO   16
#define M1_REV_GPIO   17

#define M2_PWM_GPIO   18
#define M2_REV_GPIO   8

#define M3_PWM_GPIO   20
#define M3_REV_GPIO   21

#define MOTOR_PWM_FREQ_HZ      50        // ESC tipico
#define MOTOR_PWM_RES_HZ       1000000   // resolucion de 1 MHz -> 1 tick = 1 us
#define MOTOR_PWM_BOTTOM_DUTY  55        // ~= 1100us -> minimo del ESC 
#define MOTOR_PWM_TOP_DUTY     97        // ~= 1940us -> maximo del ESC 
#define MOTOR_DEMO_DEFAULT_DUTY 20.0f    // Duty base % para la demo de motores
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

static const float theta[3] = {
    DEG2RAD(120.0f),   // M1
    DEG2RAD(0.0f),     // M2
    DEG2RAD(240.0f)    // M3
};
#define WHEEL_RADIUS   0.03f   // 3 cm
#define ROBOT_RADIUS   0.12f   // centro a rueda


static float motor_pattern_duty_percent = MOTOR_DEMO_DEFAULT_DUTY;  // Ajuste global del duty para la demo

static void motor_demo_set_duty_percent(float percent)
{
    if (percent < 0.0f) {
        percent = 0.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }
    motor_pattern_duty_percent = percent;
}

#define BNO055_I2C_PORT     I2C_NUM_0
#define BNO055_SCL_GPIO     GPIO_NUM_10
#define BNO055_SDA_GPIO     GPIO_NUM_11
#define BNO055_RST_GPIO     GPIO_NUM_NC

typedef struct {
    float angle;
    float rpm;
} encoder_simple_t;

typedef struct {
    float x;   // estado estimado (RPM)
    float P;   // covarianza del estado
    float Q;   // ruido del proceso
    float R;   // ruido de medición
} kalman_1d_t;

typedef struct {
    float vx;
    float vy;
    float wz_rad_s;
    float yaw_rad;
} state_est_t;

static state_est_t est = {0};
static portMUX_TYPE est_mux = portMUX_INITIALIZER_UNLOCKED;

static inline float rpm_to_rad_s(float rpm)
{
    return rpm * (2.0f * M_PI / 60.0f);
}

static void omni_forward_kinematics(float rpm1, float rpm2, float rpm3,
                                    float *vx, float *vy, float *wz)
{
    float w[3] = {
        rpm_to_rad_s(rpm1),
        rpm_to_rad_s(rpm2),
        rpm_to_rad_s(rpm3)
    };

    float b[3] = {
        w[0] * WHEEL_RADIUS,
        w[1] * WHEEL_RADIUS,
        w[2] * WHEEL_RADIUS
    };

    float A[3][3] = {
        { -sinf(theta[0]),  cosf(theta[0]), ROBOT_RADIUS },
        { -sinf(theta[1]),  cosf(theta[1]), ROBOT_RADIUS },
        { -sinf(theta[2]),  cosf(theta[2]), ROBOT_RADIUS }
    };

    float det =
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabsf(det) < 1e-6f) {
        *vx = *vy = *wz = 0;
        return;
    }

    float inv_det = 1.0f / det;

    float invA[3][3] = {
        {  (A[1][1]*A[2][2]-A[1][2]*A[2][1])*inv_det,
          -(A[0][1]*A[2][2]-A[0][2]*A[2][1])*inv_det,
           (A[0][1]*A[1][2]-A[0][2]*A[1][1])*inv_det },

        { -(A[1][0]*A[2][2]-A[1][2]*A[2][0])*inv_det,
           (A[0][0]*A[2][2]-A[0][2]*A[2][0])*inv_det,
          -(A[0][0]*A[1][2]-A[0][2]*A[1][0])*inv_det },

        {  (A[1][0]*A[2][1]-A[1][1]*A[2][0])*inv_det,
          -(A[0][0]*A[2][1]-A[0][1]*A[2][0])*inv_det,
           (A[0][0]*A[1][1]-A[0][1]*A[1][0])*inv_det }
    };

    float vx_e = invA[0][0]*b[0] + invA[0][1]*b[1] + invA[0][2]*b[2];
    float vy_e = invA[1][0]*b[0] + invA[1][1]*b[1] + invA[1][2]*b[2];
    float wz_e = invA[2][0]*b[0] + invA[2][1]*b[1] + invA[2][2]*b[2];

    // coherente con tu inversa (vx invertido)
    *vx = -vx_e;
    *vy =  vy_e;
    *wz =  wz_e;
}

static float fuse_wz(float wz_enc, float wz_gyro)
{
    const float alpha = 0.85f;
    return alpha * wz_gyro + (1.0f - alpha) * wz_enc;
}

static float wrap_pi(float a)
{
    while (a >  M_PI) a -= 2.0f*M_PI;
    while (a < -M_PI) a += 2.0f*M_PI;
    return a;
}

static float fuse_yaw(float yaw_prev, float wz_gyro, float yaw_meas, float dt)
{
    const float beta = 0.98f;

    float yaw_pred = wrap_pi(yaw_prev + wz_gyro * dt);
    float err = wrap_pi(yaw_meas - yaw_pred);

    return wrap_pi(yaw_pred + (1.0f - beta) * err);
}

static float yaw_controller(float yaw_target, float yaw_meas)
{
    const float Kp_yaw = 2.5f;   // AJUSTABLE (rad/s por rad)
    float err = wrap_pi(yaw_target - yaw_meas);
    return Kp_yaw * err;
}


static void sensor_fusion_task(void *arg)
{
    BNO055_t *imu = (BNO055_t *)arg;

    int64_t t_prev = esp_timer_get_time();
    float yaw_est = 0.0f;

    while (1) {
        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt <= 0 || dt > 0.05f) dt = 0.01f;

        // RPM medidas
        encoder_data_t enc;
        portENTER_CRITICAL(&encoder_mux);
        enc = encoder_data;
        portEXIT_CRITICAL(&encoder_mux);

        float rpm2, rpm3;
        portENTER_CRITICAL(&ana_mux);
        rpm2 = ana_rpm[0];
        rpm3 = ana_rpm[1];
        portEXIT_CRITICAL(&ana_mux);

        // Cinemática directa
        float vx_e, vy_e, wz_enc;
        omni_forward_kinematics(enc.rpm, rpm2, rpm3, &vx_e, &vy_e, &wz_enc);

        // IMU
        float yaw_deg, pitch, roll;
        float gx, gy, gz;
        BNO055_GetEulerAngles(imu, &yaw_deg, &pitch, &roll);
        BNO055_GetGyro(imu, &gx, &gy, &gz);

        float yaw_meas = DEG2RAD(yaw_deg);
        float wz_gyro  = DEG2RAD(gz);

        // Fusión
        float wz = fuse_wz(wz_enc, wz_gyro);
        yaw_est = fuse_yaw(yaw_est, wz_gyro, yaw_meas, dt);

        // Publicar estado
        portENTER_CRITICAL(&est_mux);
        est.vx = vx_e;
        est.vy = vy_e;
        est.wz_rad_s = wz;
        est.yaw_rad = yaw_est;
        portEXIT_CRITICAL(&est_mux);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}


static kalman_1d_t kalman_m1 = {
    .x = 0.0f,
    .P = 10.0f,
    .Q = 1.0f,     // qué tanto esperas que cambie la RPM real
    .R = 25.0f     // ruido del encoder
};

static kalman_1d_t kalman_m2 = {
    .x = 0.0f,
    .P = 10.0f,
    .Q = 3.0f,
    .R = 20.0f
};

static kalman_1d_t kalman_m3 = {
    .x = 0.0f,
    .P = 10.0f,
    .Q = 3.0f,
    .R = 20.0f
};

static encoder_simple_t encoders[AS5600_ANALOG_COUNT];

static void motor_apply_duty(bldc_pwm_motor_t *motor, float duty, const char *name)
{
    esp_err_t err = bldc_set_duty_motor(motor, duty);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo aplicar duty %.1f%% a %s (err=0x%x)", duty, name, err);
    }
}

typedef struct {
    float scale_m1;
    float scale_m2;
    float scale_m3;
    const char *label;
} motor_pattern_cmd_t;

static float kalman_update(kalman_1d_t *k, float z)
{
    // Predicción
    k->P += k->Q;

    // Ganancia de Kalman
    float K = k->P / (k->P + k->R);

    // Corrección
    k->x += K * (z - k->x);
    k->P *= (1.0f - K);

    return k->x;
}

static float pi_update(pi_ctrl_t *pi, float ref, float meas, float dt)
{
    float error = ref - meas;

    pi->integral += error * dt;

    float out = pi->kp * error + pi->ki * pi->integral;

    if (out > pi->out_max) {
        out = pi->out_max;
        pi->integral -= error * dt;  // anti-windup
    }
    else if (out < pi->out_min) {
        out = pi->out_min;
        pi->integral -= error * dt;
    }

    return out;
}


static void omni_inverse_kinematics(float vx, float vy, float wz,
                                    float *rpm1, float *rpm2, float *rpm3)
{
    vx = -vx;   // ✅ corrige convención: +vx ahora será “adelante” en tu robot

    float w[3];
    for (int i = 0; i < 3; i++) {
        w[i] =
            (-sinf(theta[i]) * vx +
              cosf(theta[i]) * vy +
              ROBOT_RADIUS * wz)
            / WHEEL_RADIUS;
    }

    const float RADS_TO_RPM = 60.0f / (2.0f * M_PI);
    *rpm1 = w[0] * RADS_TO_RPM;
    *rpm2 = w[1] * RADS_TO_RPM;
    *rpm3 = w[2] * RADS_TO_RPM;
}

#define CMD_PORT 9000

// ==============================
//  WiFi SoftAP + HTTP simple
// ==============================
static void wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    const char *ssid = "robot-snowx";
    const char *pass = "robot123";

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(ssid);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
    if (strlen(pass) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP inicializado SSID:%s password:%s canal:%d", ssid, pass, wifi_config.ap.channel);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char resp[] =
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Robot SnowX</title>"
        "<style>body{font-family:Arial;padding:16px;max-width:420px;margin:auto;}label{display:block;margin:8px 0 4px;}input{width:100%;padding:8px;font-size:16px;}button{width:100%;padding:10px;margin-top:12px;font-size:16px;}</style>"
        "</head><body>"
        "<h2>Control de movimiento</h2>"
        "<label>Velocidad (m/s)</label><input id='speed' type='number' step='0.05' value='0.20'>"
        "<label>Dirección (grados)</label><input id='dir' type='number' step='1' value='0'>"
        "<label>Tiempo (ms)</label><input id='time' type='number' step='50' value='2000'>"
        "<button onclick='send()'>Enviar</button>"
        "<pre id='status'></pre>"
        "<script>"
        "async function send(){"
        "const s=document.getElementById('speed').value;"
        "const d=document.getElementById('dir').value;"
        "const t=document.getElementById('time').value;"
        "const r=await fetch(`/cmd?speed=${s}&dir=${d}&time_ms=${t}`);"
        "document.getElementById('status').textContent=await r.text();"
        "}"
        "</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_get_handler(httpd_req_t *req)
{
    char query[128] = {0};
    float speed = 0.0f;
    float dir = 0.0f;
    int time_ms = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(query, "speed", param, sizeof(param)) == ESP_OK) {
            speed = strtof(param, NULL);
        }
        if (httpd_query_key_value(query, "dir", param, sizeof(param)) == ESP_OK) {
            dir = strtof(param, NULL);
        }
        if (httpd_query_key_value(query, "time_ms", param, sizeof(param)) == ESP_OK) {
            time_ms = atoi(param);
        }
    }

    apply_motion_command(speed, dir, (uint32_t)time_ms);

    char resp[96];
    snprintf(resp, sizeof(resp),
             "OK: V=%.2f m/s, Dir=%.1f deg, T=%d ms\n", speed, dir, time_ms);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_uri_t cmd_uri = {
            .uri = "/cmd",
            .method = HTTP_GET,
            .handler = cmd_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &cmd_uri);
        ESP_LOGI(TAG, "Servidor HTTP levantado en puerto %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "No se pudo iniciar el servidor HTTP");
    }
    return server;
}

static void tcp_cmd_server_task(void *arg)
{
    char rx_buffer[128];
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE("TCP", "Error creando socket");
        vTaskDelete(NULL);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(CMD_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(listen_sock, 1);

    ESP_LOGI("TCP", "Servidor escuchando en puerto %d", CMD_PORT);

    while (1) {
        int sock = accept(listen_sock,
                          (struct sockaddr *)&client_addr,
                          &addr_len);

        if (sock < 0) continue;

        while (1) {
            int len = recv(sock, rx_buffer,
                           sizeof(rx_buffer) - 1, 0);

            if (len <= 0) break;

            rx_buffer[len] = 0;

            float V, D;
            int T;

            if (sscanf(rx_buffer, "V=%f;D=%f;T=%d", &V, &D, &T) == 3) {
                apply_motion_command(V, D, T);
            } else {
                ESP_LOGW("TCP", "Comando inválido: %s", rx_buffer);
            }
        }

        close(sock);
    }
}


static void motor_control_task(void *arg)
{
    int64_t t_prev = esp_timer_get_time();

    while (1) {

        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) * 1e-6f;   // segundos
        t_prev = t_now;

        // Clamp de seguridad
        if (dt <= 0.0f || dt > 0.05f) {
            dt = 0.01f;   // fallback razonable
        }

        // ==============================
        // 0. COMANDO DE MOVIMIENTO DESDE WEB
        // ==============================
        int64_t now_us = esp_timer_get_time();

        portENTER_CRITICAL(&motion_mux);

        if (motion_cmd.active) {
            if (now_us < motion_cmd.end_time_us) {
                vx_cmd = motion_cmd.vx;
                vy_cmd = motion_cmd.vy;

                // --- HEADING HOLD ---
                if (heading_hold_enabled &&
                    (fabsf(vx_cmd) > 0.01f || fabsf(vy_cmd) > 0.01f)) {

                    state_est_t s;
                    portENTER_CRITICAL(&est_mux);
                    s = est;
                    portEXIT_CRITICAL(&est_mux);

                    float wz_ref = yaw_controller(yaw_target, s.yaw_rad);

                    const float Kp_wz = 0.8f;   // ajuste fino
                    float wz_err = wz_ref - s.wz_rad_s;

                    wz_cmd = wz_ref + Kp_wz * wz_err;
                    const float WZ_MAX = 2.0f;   // rad/s (~115°/s)
                    if (wz_cmd >  WZ_MAX) wz_cmd =  WZ_MAX;
                    if (wz_cmd < -WZ_MAX) wz_cmd = -WZ_MAX;
                } else {
                    wz_cmd = motion_cmd.wz;
                }
            } else {
                // Tiempo vencido → STOP automático
                vx_cmd = 0.0f;
                vy_cmd = 0.0f;
                wz_cmd = 0.0f;
                motion_cmd.active = false;
                portENTER_CRITICAL(&est_mux);
                yaw_target = est.yaw_rad;
                portEXIT_CRITICAL(&est_mux);
            }
        } else {
            vx_cmd = 0.0f;
            vy_cmd = 0.0f;
            wz_cmd = 0.0f;
        }

        portEXIT_CRITICAL(&motion_mux);


        // ==============================
        // 1. CINEMÁTICA INVERSA  ← AQUÍ VA
        // ==============================
        float rpm1_ref, rpm2_ref, rpm3_ref;

        omni_inverse_kinematics(
            vx_cmd,
            vy_cmd,
            wz_cmd,
            &rpm1_ref,
            &rpm2_ref,
            &rpm3_ref
        );

        rpm1_ref *= wheel_gain[0];
        rpm2_ref *= wheel_gain[1];
        rpm3_ref *= wheel_gain[2];

        // límite físico
        const float MAX_RPM = 300.0f;

        rpm1_ref = fmaxf(fminf(rpm1_ref, MAX_RPM), -MAX_RPM);
        rpm2_ref = fmaxf(fminf(rpm2_ref, MAX_RPM), -MAX_RPM);
        rpm3_ref = fmaxf(fminf(rpm3_ref, MAX_RPM), -MAX_RPM);

        // escribir referencias GLOBALES
        rpm_ref[0] = rpm1_ref;
        rpm_ref[1] = rpm2_ref;
        rpm_ref[2] = rpm3_ref;


        // ==============================
        // 1. Leer RPM medidas
        // ==============================
        encoder_data_t enc_i2c;
        portENTER_CRITICAL(&encoder_mux);
        enc_i2c = encoder_data;
        portEXIT_CRITICAL(&encoder_mux);

        float m2_rpm, m3_rpm;
        portENTER_CRITICAL(&ana_mux);
        m2_rpm = ana_rpm[0];
        m3_rpm = ana_rpm[1];
        portEXIT_CRITICAL(&ana_mux);

        float rpm_meas[3] = {
            enc_i2c.rpm,
            m2_rpm,
            m3_rpm
        };

        // ==============================
        // 2. Control PI por rueda
        // ==============================
        float ref_abs[3]  = { fabsf(rpm_ref[0]), fabsf(rpm_ref[1]), fabsf(rpm_ref[2]) };
        float meas_abs[3] = { fabsf(rpm_meas[0]), fabsf(rpm_meas[1]), fabsf(rpm_meas[2]) };

        float pwm1 = (ref_abs[0] < 5.0f) ? 0.0f : pi_update(&pi_m1, ref_abs[0], meas_abs[0], dt);
        float pwm2 = (ref_abs[1] < 5.0f) ? 0.0f : pi_update(&pi_m2, ref_abs[1], meas_abs[1], dt);
        float pwm3 = (ref_abs[2] < 5.0f) ? 0.0f : pi_update(&pi_m3, ref_abs[2], meas_abs[2], dt);

        // aplicar signo de la referencia DESPUÉS del PI
        if (rpm_ref[0] < 0) pwm1 = -pwm1;
        if (rpm_ref[1] < 0) pwm2 = -pwm2;
        if (rpm_ref[2] < 0) pwm3 = -pwm3;

        // ==============================
        // 4. Aplicar PWM
        // ==============================
        bldc_set_duty_motor(&motor1, pwm1);
        bldc_set_duty_motor(&motor2, pwm2);
        bldc_set_duty_motor(&motor3, pwm3);

        // ==============================vx_cmdvx_cmd
        // 5. Periodo de control
        // ==============================
        vTaskDelay(pdMS_TO_TICKS(10));   // objetivo: ~100 Hz
    }
}

static void motor_demo_step(void)
{
    static const motor_pattern_cmd_t pattern[] = {
        {  1.0f,   0.0f, 0.0f, "Todos girando" }
    };
    static size_t idx = 0;

    const motor_pattern_cmd_t *cmd = &pattern[idx];
    float duty_m1 = cmd->scale_m1 * motor_pattern_duty_percent;
    float duty_m2 = cmd->scale_m2 * motor_pattern_duty_percent;
    float duty_m3 = cmd->scale_m3 * motor_pattern_duty_percent;

    motor_apply_duty(&motor1, duty_m1, "motor1");
    motor_apply_duty(&motor2, duty_m2, "motor2");
    motor_apply_duty(&motor3, duty_m3, "motor3");

    ESP_LOGI(TAG, "Patron motores: %s -> duty [%.1f, %.1f, %.1f]",
             cmd->label, duty_m1, duty_m2, duty_m3);

    idx = (idx + 1) % (sizeof(pattern) / sizeof(pattern[0]));
}

static void encoder_task(void *arg)
{
    AS5600_t *enc = (AS5600_t *)arg;

    float prev_angle = 0.0f;
    int64_t prev_time = esp_timer_get_time();

    while (1) {
        float angle;
        uint8_t status, agc;
        uint16_t mag;

        bool ok = true;
        ok &= as5600_get_angle_deg(enc, &angle);
        ok &= as5600_get_status(enc, &status);
        ok &= as5600_get_agc(enc, &agc);
        ok &= as5600_get_magnitude(enc, &mag);

        if (ok) {
            int64_t now = esp_timer_get_time();
            float dt = (now - prev_time) * 1e-6f;

            if (dt > 0.0f) {
                float dtheta = angle - prev_angle;

                if (dtheta > 180.0f) dtheta -= 360.0f;
                if (dtheta < -180.0f) dtheta += 360.0f;

                float rpm = (dtheta / dt) * (60.0f / 360.0f);


                portENTER_CRITICAL(&encoder_mux);
                encoder_data.angle_deg = angle;
                encoder_data.rpm = kalman_update(&kalman_m1, rpm);
                encoder_data.status    = status;
                encoder_data.agc       = agc;
                encoder_data.mag       = mag;
                portEXIT_CRITICAL(&encoder_mux);

                prev_angle = angle;
                prev_time  = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

static void analog_encoders_task(void *arg)


{
    (void)arg;

    float prev_angle[AS5600_ANALOG_COUNT] = {0};
    int64_t prev_time = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - prev_time) * 1e-6f;
        if (dt <= 0.0f) dt = 0.01f;

        for (int i = 0; i < AS5600_ANALOG_COUNT; i++) {
            

    float angle;
    if (!as5600_read_analog_deg(i, &angle)) {
        continue;   // esto sí está bien (sensor no respondió)
    }

    float dtheta = angle - prev_angle[i];

    if (dtheta > 180.0f)  dtheta -= 360.0f;
    if (dtheta < -180.0f) dtheta += 360.0f;

    float rpm_raw = (dtheta / dt) * (60.0f / 360.0f);

    if (i == 1) {
        rpm_raw = -rpm_raw;   // 🔥 invertir M3
    }

    // -------------------------------
    // CLAMP FÍSICO (NO DESCARTA)
    // -------------------------------
    const float RPM_MAX = 600.0f;

    if (rpm_raw >  RPM_MAX) rpm_raw =  RPM_MAX;
    if (rpm_raw < -RPM_MAX) rpm_raw = -RPM_MAX;

    // -------------------------------
    // SUAVIZADO PREVIO (anti-picos)
    // -------------------------------
    ana_rpm_valid[i] = 0.7f * ana_rpm_valid[i] + 0.3f * rpm_raw;

    // -------------------------------
    // KALMAN CONTINUO
    // -------------------------------
    portENTER_CRITICAL(&ana_mux);

    ana_angle[i] = angle;

    if (i == 0) {
        ana_rpm[i] = kalman_update(&kalman_m2, ana_rpm_valid[i]);
    } else if (i == 1) {
        ana_rpm[i] = kalman_update(&kalman_m3, ana_rpm_valid[i]);
    }

    portEXIT_CRITICAL(&ana_mux);


    prev_angle[i] = angle;
}


        prev_time = now;
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

static void apply_motion_command(float speed, float dir_deg, uint32_t time_ms)
{
    float dir_rad = dir_deg * M_PI / 180.0f;

    portENTER_CRITICAL(&motion_mux);

    motion_cmd.vx = speed * cosf(dir_rad);
    motion_cmd.vy = speed * sinf(dir_rad);
    motion_cmd.wz = 0.0f;

    // fijar rumbo objetivo al iniciar movimiento
    portENTER_CRITICAL(&est_mux);
    yaw_target = est.yaw_rad;
    portEXIT_CRITICAL(&est_mux);

    motion_cmd.end_time_us =
        esp_timer_get_time() + (int64_t)time_ms * 1000;

    motion_cmd.active = true;

    portEXIT_CRITICAL(&motion_mux);

    ESP_LOGI("CMD",
        "Movimiento: V=%.2f m/s | Dir=%.1f deg | T=%d ms",
        speed, dir_deg, time_ms
    );
}


void app_main(void)
{
    ESP_LOGI(TAG, "==== AS5600 DEMO START ====");

    AS5600_t encoder;
    BNO055_t imu;

    // --- Inicializar bus I2C ---
    if (!as5600_init_bus(&encoder, AS5600_I2C_PORT, AS5600_I2C_SCL_PIN, AS5600_I2C_SDA_PIN, AS5600_I2C_FREQ_HZ)) {
        ESP_LOGE(TAG, "Fallo al iniciar bus I2C");
        return;
    }
    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");

    // Red + servidor web para comandos
    wifi_init_softap();
    start_webserver();
    xTaskCreatePinnedToCore(tcp_cmd_server_task, "tcp_cmd", 4096, NULL, 4, NULL, 1);

    // --- Inicializar ADC para encoders analógicos ---
    bool analog_available = as5600_init_analog_encoders();
    if (!analog_available) {
        ESP_LOGW(TAG, "No se pudo inicializar los encoders analógicos (ADC)");
    } else {
        ESP_LOGI(TAG, "Encoders analógicos inicializados");
    }

    xTaskCreatePinnedToCore(analog_encoders_task,"analog_enc",4096,NULL,5,NULL,1 );

    // --- Inicializar IMU BNO055 en bus independiente ---
    bool imu_available = (BNO055_Init(&imu, BNO055_I2C_PORT, BNO055_SCL_GPIO, BNO055_SDA_GPIO, BNO055_RST_GPIO) == BNO055_SUCCESS);
    ESP_LOGI(TAG, "IMU status: %s", imu_available ? "OK" : "FALLÓ");

    if (imu_available) {
    xTaskCreatePinnedToCore(
        sensor_fusion_task,
        "sensor_fusion",
        4096,
        &imu,
        6,
        NULL,
        1
    );
}


    // --- Calibración volátil completa (0..4095) ---
    if (!as5600_calibrate_full_range(&encoder)) {
        ESP_LOGW(TAG, "No se pudo calibrar el rango completo, revisa el imán o conexiones");
    } else {
        ESP_LOGI(TAG, "Calibración completada (ventana 0..4095)");
    }

    // --- Configurar parámetros opcionales (CONF) ---
    AS5600_config_t conf = { .WORD = 0 };
    conf.PM   = AS5600_POWER_MODE_NOM;
    conf.HYST = AS5600_HYSTERESIS_2LSB;
    conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_FR;
    conf.PWMF = AS5600_PWM_FREQUENCY_115HZ;
    conf.SF   = AS5600_SLOW_FILTER_8X;
    conf.FTH  = AS5600_FF_THRESHOLD_6LSB;
    conf.WD   = AS5600_WATCHDOG_OFF;

    

    if (AS5600_Calibrate(&encoder, conf, 0x0000, 0x0FFF)) {
        ESP_LOGI(TAG, "Configuración CONF escrita y verificada correctamente");
    } else {
        ESP_LOGW(TAG, "Error al aplicar configuración CONF");
    }

    xTaskCreatePinnedToCore(encoder_task,"encoder_task",4096,&encoder,5,NULL,1);

    // --- Inicializar PWM de los tres ESC ---
    ESP_LOGI(TAG, "Inicializando motores...");

    if (bldc_init(&motor1, M1_PWM_GPIO, M1_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 1");
        return;
    }

    if (bldc_init(&motor2, M2_PWM_GPIO, M2_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 2");
        return;
    }

    if (bldc_init(&motor3, M3_PWM_GPIO, M3_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 3");
        return;
    }

    if (bldc_enable(&motor1) != ESP_OK ||
        bldc_enable(&motor2) != ESP_OK ||
        bldc_enable(&motor3) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo habilitar MCPWM para todos los motores");
        return;
    }

    bldc_set_duty(&motor1, MOTOR_PWM_BOTTOM_DUTY);
    bldc_set_duty(&motor2, MOTOR_PWM_BOTTOM_DUTY);
    bldc_set_duty(&motor3, MOTOR_PWM_BOTTOM_DUTY);
    vTaskDelay(pdMS_TO_TICKS(3000));  // esperar a que el ESC arme


    motor_demo_set_duty_percent(MOTOR_DEMO_DEFAULT_DUTY);  // Ajusta aquí la intensidad global del patrón

    xTaskCreatePinnedToCore(motor_control_task,"motor_ctrl",4096,NULL,6,NULL,1);


    // --- Loop principal ---
    float analog_deg[AS5600_ANALOG_COUNT] = {0};
    int64_t prev_time = esp_timer_get_time();

    while (1)
    {
        float yaw = 0, pitch = 0, roll = 0;
        encoder_data_t enc_copy;

        portENTER_CRITICAL(&encoder_mux);
        enc_copy = encoder_data;
        portEXIT_CRITICAL(&encoder_mux);

        bool ok = true;


        if (!ok) {
            ESP_LOGE(TAG, "Error leyendo el AS5600 por I2C");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        for (size_t i = 0; analog_available && i < AS5600_ANALOG_COUNT; ++i) {
            float adc_deg = analog_deg[i];
            if (as5600_read_analog_deg(i, &adc_deg)) {
                analog_deg[i] = adc_deg;
            } else {
            ESP_LOGW(TAG, "No se pudo leer encoder analógico %zu", i);
            }
        }

        int64_t now = esp_timer_get_time();
        float dt = (now - prev_time) * 1e-6f;


        prev_time = now;

        float m2a, m3a, a2, a3;
        portENTER_CRITICAL(&ana_mux);
        a2 = ana_angle[0]; m2a = ana_rpm[0];
        a3 = ana_angle[1]; m3a = ana_rpm[1];
        portEXIT_CRITICAL(&ana_mux);

    
        printf(
        "REF[%.1f %.1f %.1f] | "
        "M1(I2C) RPM=%.2f | M2(ANA0) RPM=%.2f | M3(ANA1) RPM=%.2f\n",
        rpm_ref[0], rpm_ref[1], rpm_ref[2],
        enc_copy.rpm, m2a, m3a
        );

        printf("\n");

        state_est_t s;
        portENTER_CRITICAL(&est_mux);
        s = est;
        portEXIT_CRITICAL(&est_mux);

        printf("EST vx=%.2f vy=%.2f wz=%.2f yaw=%.2f deg\n",
            s.vx, s.vy, s.wz_rad_s, s.yaw_rad * 180.0f / M_PI);

        vTaskDelay(pdMS_TO_TICKS(500));  // 2 Hz
    }

    // Nunca llega aquí, pero si quisieras limpiar:
    // as5600_deinit_bus(&encoder);
}
