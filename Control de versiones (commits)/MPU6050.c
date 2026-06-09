#include "mpu6050.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
 

// PRIMITIVAS I2C INTERNAS (No expuestas en la cabecera)

 
/**
 * @brief  Escribe un byte en un registro del MPU-6050.
 * @param  cfg    Puntero a la configuración del driver.
 * @param  reg    Dirección del registro destino.
 * @param  value  Valor a escribir.
 * @return MPU6050_OK o MPU6050_ERR_I2C_WRITE.
 */
static mpu6050_err_t _write_register(const mpu6050_config_t *cfg,
                                      uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
 
    // i2c_write_blocking retorna el número de bytes escritos o PICO_ERROR_GENERIC
    int result = i2c_write_blocking(cfg->i2c_port, cfg->i2c_addr, buf, 2, false);
 
    return (result == 2) ? MPU6050_OK : MPU6050_ERR_I2C_WRITE;
}
 
/**
 * @brief  Lee uno o más bytes consecutivos desde un registro del MPU-6050.
 * @param  cfg    Puntero a la configuración del driver.
 * @param  reg    Dirección del registro inicial.
 * @param  buf    Buffer de destino.
 * @param  len    Número de bytes a leer.
 * @return MPU6050_OK o MPU6050_ERR_I2C_READ.
 */
static mpu6050_err_t _read_registers(const mpu6050_config_t *cfg,
                                      uint8_t reg, uint8_t *buf, size_t len) {
    int result;
 
    // Fase 1: Escritura del puntero de registro (sin STOP para mantener el bus)
    result = i2c_write_blocking(cfg->i2c_port, cfg->i2c_addr, &reg, 1, true);
    if (result != 1) {
        return MPU6050_ERR_I2C_WRITE;
    }
 
    // Fase 2: Lectura en ráfaga con STOP al final
    result = i2c_read_blocking(cfg->i2c_port, cfg->i2c_addr, buf, len, false);
    if (result != (int)len) {
        return MPU6050_ERR_I2C_READ;
    }
 
    return MPU6050_OK;
}
 

// INTERFAZ PÚBLICA

 
mpu6050_config_t mpu6050_get_default_config(void) {
    mpu6050_config_t cfg = {
        .i2c_port   = i2c_default,          // i2c0 por defecto (SDA=GP4, SCL=GP5)
        .i2c_addr   = MPU6050_ADDR,          // 0x68 (AD0 = GND)
        .accel_fs   = MPU6050_ACCEL_FS_2G,   // ±2g: resolución máxima para movimientos finos del guante
        .gyro_fs    = MPU6050_GYRO_FS_250DPS, // ±250°/s: suficiente para gestos de mano
        .dlpf_bw    = MPU6050_DLPF_BW_42HZ,  // 42 Hz DLPF: filtra vibraciones > 42 Hz del guante
        .smplrt_div = 0x00                    // Sin división: frecuencia de muestreo = giroscopio / (1+0)
    };
    return cfg;
}
 
mpu6050_err_t mpu6050_init(const mpu6050_config_t *cfg) {
    if (cfg == NULL || cfg->i2c_port == NULL) {
        return MPU6050_ERR_NULL_PTR;
    }
 
    mpu6050_err_t err;
 
    // PASO 1: Verificar identidad del dispositivo en el bus antes de cualquier configuración
    err = mpu6050_check_identity(cfg);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 2: Reset completo del dispositivo (equivale a un power-on reset por software)
    // Esto garantiza un estado inicial determinista independiente del estado previo
    err = _write_register(cfg, MPU6050_REG_PWR_MGMT_1, MPU6050_RESET_BIT);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 3: Esperar el tiempo mínimo de estabilización post-reset (datasheet: 100ms)
    sleep_ms(100);
 
    // PASO 4: Despertar el dispositivo y configurar fuente de reloj PLL del giroscopio X
    // El reloj PLL es más estable y preciso que el oscilador interno de 8 MHz
    err = _write_register(cfg, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 5: Configurar el filtro digital de paso bajo (DLPF)
    // Reduce ruido de alta frecuencia sin afectar la respuesta a gestos de mano
    err = _write_register(cfg, MPU6050_REG_CONFIG, (uint8_t)cfg->dlpf_bw);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 6: Configurar divisor de frecuencia de muestreo
    // Frecuencia_muestreo = Frecuencia_giroscopio / (1 + SMPLRT_DIV)
    err = _write_register(cfg, MPU6050_REG_SMPLRT_DIV, cfg->smplrt_div);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 7: Configurar escala del giroscopio
    err = _write_register(cfg, MPU6050_REG_GYRO_CONFIG, (uint8_t)cfg->gyro_fs);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // PASO 8: Configurar escala del acelerómetro
    err = _write_register(cfg, MPU6050_REG_ACCEL_CONFIG, (uint8_t)cfg->accel_fs);
    if (err != MPU6050_OK) {
        return err;
    }
 
    return MPU6050_OK;
}
 
mpu6050_err_t mpu6050_check_identity(const mpu6050_config_t *cfg) {
    if (cfg == NULL) {
        return MPU6050_ERR_NULL_PTR;
    }
 
    uint8_t who_am_i = 0;
    mpu6050_err_t err = _read_registers(cfg, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
 
    if (err != MPU6050_OK) {
        return err;
    }
 
    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        return MPU6050_ERR_WHO_AM_I;
    }
 
    return MPU6050_OK;
}
 
mpu6050_err_t mpu6050_read_all(const mpu6050_config_t *cfg, mpu6050_data_t *data) {
    if (cfg == NULL || data == NULL) {
        return MPU6050_ERR_NULL_PTR;
    }
 
    // Buffer de 14 bytes: 6 de acelerómetro + 2 de temperatura + 6 de giroscopio
    // La temperatura (bytes [6] y [7]) se descarta pero debe leerse para mantener
    // la ráfaga continua desde ACCEL_XOUT_H hasta GYRO_ZOUT_L en una sola transacción
    uint8_t raw[14];
 
    mpu6050_err_t err = _read_registers(cfg, MPU6050_REG_ACCEL_XOUT_H, raw, 14);
    if (err != MPU6050_OK) {
        return err;
    }
 
    // Reconstrucción de los valores de 16 bits (big-endian, complemento a 2)
    // El MPU-6050 envía primero el byte alto (H) y luego el bajo (L)
    int16_t raw_ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4]  << 8) | raw[5]);
    // raw[6] y raw[7] corresponden a temperatura → descartados
    int16_t raw_gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);
 
    // Conversión a unidades físicas usando el factor de escala configurado
    // Para el proyecto echo se usa ±2g y ±250°/s como valores por defecto
    data->accel_x = (float)raw_ax / MPU6050_ACCEL_SCALE_2G;
    data->accel_y = (float)raw_ay / MPU6050_ACCEL_SCALE_2G;
    data->accel_z = (float)raw_az / MPU6050_ACCEL_SCALE_2G;
    data->gyro_x  = (float)raw_gx / MPU6050_GYRO_SCALE_250;
    data->gyro_y  = (float)raw_gy / MPU6050_GYRO_SCALE_250;
    data->gyro_z  = (float)raw_gz / MPU6050_GYRO_SCALE_250;
 
    return MPU6050_OK;
}
 
mpu6050_err_t mpu6050_read_to_vector(const mpu6050_config_t *cfg, float *output) {
    if (cfg == NULL || output == NULL) {
        return MPU6050_ERR_NULL_PTR;
    }
 
    mpu6050_data_t data;
    mpu6050_err_t err = mpu6050_read_all(cfg, &data);
 
    if (err != MPU6050_OK) {
        return err;
    }
 
    // Mapeo directo al formato del super_vector[14..19] del proyecto echo
    // Orden estricto: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
    output[0] = data.accel_x;
    output[1] = data.accel_y;
    output[2] = data.accel_z;
    output[3] = data.gyro_x;
    output[4] = data.gyro_y;
    output[5] = data.gyro_z;
 
    return MPU6050_OK;
}