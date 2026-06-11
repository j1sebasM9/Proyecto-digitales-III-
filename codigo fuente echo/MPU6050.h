/**
 * @file    mpu6050.h
 * @brief   Driver I2C para el sensor inercial MPU-6050 - Proyecto "echo"
 * @details Módulo de abstracción de hardware para lectura determinista de los
 *          6 ejes inerciales (acelerómetro + giroscopio) del MPU-6050 via I2C.
 *          Diseñado para operar en el Core 0 del RP2040 bajo arquitectura IRQ + Polling.
 *
 * @author  Proyecto echo
 * @version 1.0.0
 */
 
#ifndef MPU6050_H
#define MPU6050_H
 
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
 

// DIRECCIÓN I2C DEL DISPOSITIVO

 
/**
 * @brief Dirección I2C del MPU-6050.
 * @note  AD0 = GND → 0x68 (default)
 *        AD0 = VCC → 0x69 (alternativa si hay dos dispositivos en el bus)
 */
#define MPU6050_ADDR            0x68
 

// MAPA DE REGISTROS DEL MPU-6050 (Datasheet Rev 3.4)

 
#define MPU6050_REG_PWR_MGMT_1  0x6B  ///< Gestión de energía 1 (reset, sleep, clock source)
#define MPU6050_REG_PWR_MGMT_2  0x6C  ///< Gestión de energía 2 (standby individual por eje)
#define MPU6050_REG_SMPLRT_DIV  0x19  ///< Divisor de frecuencia de muestreo
#define MPU6050_REG_CONFIG      0x1A  ///< Configuración del filtro DLPF y FSYNC
#define MPU6050_REG_GYRO_CONFIG 0x1B  ///< Configuración de escala del giroscopio
#define MPU6050_REG_ACCEL_CONFIG 0x1C ///< Configuración de escala del acelerómetro
#define MPU6050_REG_ACCEL_XOUT_H 0x3B ///< Byte alto de aceleración en X (inicio del bloque de 14 bytes)
#define MPU6050_REG_WHO_AM_I    0x75  ///< Registro de identificación del dispositivo (debe retornar 0x68)
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68 ///< Reset de la cadena de señal analógica
 

// VALORES DE CONFIGURACIÓN

 
#define MPU6050_WHO_AM_I_VAL    0x68  ///< Valor esperado del registro WHO_AM_I
#define MPU6050_RESET_BIT       0x80  ///< Bit de reset del dispositivo en PWR_MGMT_1
#define MPU6050_SLEEP_BIT       0x40  ///< Bit de modo sleep en PWR_MGMT_1
#define MPU6050_CLOCK_PLL_XGYRO 0x01  ///< Fuente de reloj: PLL con referencia del giroscopio X (recomendado)
 
/** @brief Escala completa del acelerómetro */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,  ///< ±2g  → LSB/g = 16384
    MPU6050_ACCEL_FS_4G  = 0x08,  ///< ±4g  → LSB/g = 8192
    MPU6050_ACCEL_FS_8G  = 0x10,  ///< ±8g  → LSB/g = 4096
    MPU6050_ACCEL_FS_16G = 0x18   ///< ±16g → LSB/g = 2048
} mpu6050_accel_fs_t;
 
/** @brief Escala completa del giroscopio */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0x00,  ///< ±250°/s  → LSB/(°/s) = 131.0
    MPU6050_GYRO_FS_500DPS  = 0x08,  ///< ±500°/s  → LSB/(°/s) = 65.5
    MPU6050_GYRO_FS_1000DPS = 0x10,  ///< ±1000°/s → LSB/(°/s) = 32.8
    MPU6050_GYRO_FS_2000DPS = 0x18   ///< ±2000°/s → LSB/(°/s) = 16.4
} mpu6050_gyro_fs_t;
 
/** @brief Ancho de banda del filtro digital de paso bajo (DLPF) */
typedef enum {
    MPU6050_DLPF_BW_256HZ = 0x00,  ///< Sin filtro activo (ruido máximo)
    MPU6050_DLPF_BW_188HZ = 0x01,
    MPU6050_DLPF_BW_98HZ  = 0x02,
    MPU6050_DLPF_BW_42HZ  = 0x03,  ///< Recomendado para gestos de mano
    MPU6050_DLPF_BW_20HZ  = 0x04,
    MPU6050_DLPF_BW_10HZ  = 0x05,
    MPU6050_DLPF_BW_5HZ   = 0x06
} mpu6050_dlpf_t;
 

// FACTORES DE ESCALA (Conversión de LSB a unidades físicas)

 
#define MPU6050_ACCEL_SCALE_2G   16384.0f  ///< LSB por g para ±2g
#define MPU6050_GYRO_SCALE_250   131.0f    ///< LSB por °/s para ±250°/s
 

// CÓDIGOS DE RETORNO

 
/** @brief Códigos de error del driver */
typedef enum {
    MPU6050_OK              =  0,   ///< Operación exitosa
    MPU6050_ERR_I2C_WRITE   = -1,   ///< Falla en transmisión I2C
    MPU6050_ERR_I2C_READ    = -2,   ///< Falla en recepción I2C
    MPU6050_ERR_WHO_AM_I    = -3,   ///< Dispositivo no reconocido en el bus
    MPU6050_ERR_TIMEOUT     = -4,   ///< Timeout esperando respuesta del dispositivo
    MPU6050_ERR_NULL_PTR    = -5    ///< Puntero nulo en argumento
} mpu6050_err_t;
 

// ESTRUCTURA DE CONFIGURACIÓN

 
/**
 * @brief Estructura de configuración del driver MPU-6050.
 * @note  Inicializar con mpu6050_get_default_config() antes de usar.
 */
typedef struct {
    i2c_inst_t          *i2c_port;   ///< Puerto I2C del RP2040 (i2c0 o i2c1)
    uint8_t              i2c_addr;   ///< Dirección I2C del dispositivo
    mpu6050_accel_fs_t   accel_fs;   ///< Escala del acelerómetro
    mpu6050_gyro_fs_t    gyro_fs;    ///< Escala del giroscopio
    mpu6050_dlpf_t       dlpf_bw;   ///< Ancho de banda del filtro DLPF
    uint8_t              smplrt_div; ///< Divisor de muestreo (0 = máxima frecuencia)
} mpu6050_config_t;
 

// ESTRUCTURA DE DATOS DE SALIDA

 
/**
 * @brief Vector de datos inerciales de 6 ejes en unidades físicas.
 * @note  Los 6 valores de esta estructura corresponden directamente a
 *        super_vector[14..19] del proyecto echo.
 */
typedef struct {
    float accel_x;  ///< Aceleración en X [g]
    float accel_y;  ///< Aceleración en Y [g]
    float accel_z;  ///< Aceleración en Z [g]
    float gyro_x;   ///< Velocidad angular en X [°/s]
    float gyro_y;   ///< Velocidad angular en Y [°/s]
    float gyro_z;   ///< Velocidad angular en Z [°/s]
} mpu6050_data_t;
 

// INTERFAZ PÚBLICA DEL DRIVER

 
/**
 * @brief  Retorna una configuración por defecto segura para el proyecto echo.
 * @return Estructura mpu6050_config_t con valores predeterminados.
 */
mpu6050_config_t mpu6050_get_default_config(void);
 
/**
 * @brief  Inicializa el MPU-6050: verifica presencia, reset y aplica configuración.
 * @param  cfg  Puntero a la configuración del dispositivo.
 * @return MPU6050_OK si exitoso, código de error en caso contrario.
 */
mpu6050_err_t mpu6050_init(const mpu6050_config_t *cfg);
 
/**
 * @brief  Lee los 6 ejes inerciales en una sola transacción I2C de 14 bytes.
 * @param  cfg   Puntero a la configuración del dispositivo.
 * @param  data  Puntero a la estructura de destino de los datos.
 * @return MPU6050_OK si exitoso, código de error en caso contrario.
 */
mpu6050_err_t mpu6050_read_all(const mpu6050_config_t *cfg, mpu6050_data_t *data);
 
/**
 * @brief  Escribe los 6 ejes como floats en un arreglo plano (para super_vector).
 * @param  cfg     Puntero a la configuración del dispositivo.
 * @param  output  Arreglo de destino de al menos 6 elementos float.
 * @return MPU6050_OK si exitoso, código de error en caso contrario.
 * @note   Orden: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
 *         Compatible directo con super_vector[14..19] del Core 0.
 */
mpu6050_err_t mpu6050_read_to_vector(const mpu6050_config_t *cfg, float *output);
 
/**
 * @brief  Verifica que el dispositivo responda correctamente en el bus I2C.
 * @param  cfg  Puntero a la configuración del dispositivo.
 * @return MPU6050_OK si el WHO_AM_I es correcto, MPU6050_ERR_WHO_AM_I si no.
 */
mpu6050_err_t mpu6050_check_identity(const mpu6050_config_t *cfg);
 
#endif // MPU6050_H