Robot SnowX
===========

Firmware para una plataforma omnidireccional de 3 ruedas con ESP-IDF. Integra encoders AS5600 (I2C y analógicos), IMU BNO055, control de motores BLDC por PWM, servidor HTTP + TCP para comandos y lazo de control PI con fusión de sensores para mantener el rumbo.

Características principales
--------------------------
- SoftAP integrado (`SSID: robot-snowx`, `pass: robot123`) con página mínima para enviar comandos de movimiento.
- Servidor TCP en el puerto 9000 (`V=0.20;D=0;T=2000`) como alternativa al HTTP.
- Tareas FreeRTOS separadas para: fusión de sensores (IMU+encoders), lectura de encoders (I2C y analógicos), control de motores y servidor TCP.
- Cinemática directa e inversa para chasis omnidireccional; control PI por rueda con anti-windup y límites de seguridad.
- “Heading hold”: mientras hay comando de traslación mantiene el yaw objetivo usando la IMU.

Requisitos
----------
- ESP-IDF 5.x (probado con toolchain de ESP32).
- Placa ESP32 con 3 ESC controlados por PWM.
- Sensores: AS5600 (I2C) para la rueda 1, dos AS5600 analógicos para ruedas 2 y 3, IMU BNO055 en bus I2C independiente.

Asignación de pines
-------------------
- Motor 1: PWM `GPIO16`, REV `GPIO17`
- Motor 2: PWM `GPIO18`, REV `GPIO8`
- Motor 3: PWM `GPIO20`, REV `GPIO21`
- IMU BNO055 I2C0: SCL `GPIO10`, SDA `GPIO11`, RST `NC`
- AS5600 I2C (rueda 1): usa `AS5600_I2C_PORT`, `AS5600_I2C_SCL_PIN`, `AS5600_I2C_SDA_PIN` definidos en `as5600_lib.h`
- Encoders analógicos (ruedas 2 y 3): ver `as5600_lib.c` para canales ADC

Estructura rápida
-----------------
- `main/main.c`  lógica de aplicación: servidores, cinemática, control PI, fusión de sensores, comandos.
- `main/as5600_lib.*`  drivers I2C/ADC para AS5600 y calibración.
- `main/bldc_pwm.*`    wrapper MCPWM para los ESC.
- `main/bno055.*`      driver básico BNO055.
- `main/init.*`        utilidades de arranque.
- `CMakeLists.txt`     proyecto ESP-IDF.


Uso rápido
----------
1) Alimentar la placa y espera a que los ESC armen (3 s).  
2) Conectarse al WiFi SoftAP `robot-snowx` (`robot123`).  
3) Abre `http://192.168.4.1/` y envía comandos: velocidad (m/s), dirección (deg) y tiempo (ms).  
4) Es posible abrir un socket TCP al puerto 9000 y enviar `V=0.25;D=90;T=3000`.  
5) Los logs de control y estimación se muestran en el monitor serial.

Parámetros clave (editable en `main.c`)
---------------------------------------
- Ganancias PI por rueda: `pi_m1`, `pi_m2`, `pi_m3`.
- Ganancia de heading: `Kp_yaw`, `Kp_wz`, límite `WZ_MAX`.
- Ganancias de rueda: `wheel_gain[]` para compensar desbalances.
- Límites: `MAX_RPM`, clamps en encoders analógicos (`RPM_MAX`), rangos de PWM `MOTOR_PWM_*`.
- SSID/clave del SoftAP (`wifi_init_softap`).

Flujo de tareas (resumen)
-------------------------
- `encoder_task`: AS5600 I2C (rueda 1) con filtro Kalman.
- `analog_encoders_task`: AS5600 analógicos (ruedas 2 y 3) con suavizado + Kalman.
- `sensor_fusion_task`: combina cinemática (encoders) con IMU (yaw, gyro) → estado `est`.
- `motor_control_task`: aplica cinemática inversa, heading hold, PI y escribe PWM a los 3 motores.
- `tcp_cmd_server_task` y HTTP server: reciben comandos y activan `apply_motion_command`.

Notas y tips
------------
- El `heading_hold` fija el yaw al iniciar un comando; si quieres giros explícitos ajusta `motion_cmd.wz` o desactiva el modo.
- El BNO055 se inicializa en un I2C dedicado; si falla se loguea pero el sistema sigue operando sin fusión.
- La calibración AS5600 completa (`as5600_calibrate_full_range`) se ejecuta al arranque; revisa imán y alineación si falla.
- Ajusta `MOTOR_PWM_BOTTOM_DUTY` y `MOTOR_PWM_TOP_DUTY` a las especificaciones de tus ESC para evitar arm/desarme errático.

