/**
 * @file    core0_main.c
 * @brief   Subsistema CORE 0 - Proyecto "echo" (Versión de Producción Final)
 * @details Arquitectura: IRQ + Polling, FSM, Pipeline PIO+DMA autónomo.
 * Define las variables globales compartidas con Core 1 (common.h).
 */
 
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/address_mapped.h"
 
#include "mux_adc.pio.h"
#include "mpu6050.h"
#include "common.h"
#include "core1_ai.h"
 
// ============================================================
// DEFINICIÓN DE VARIABLES GLOBALES COMPARTIDAS
// ============================================================
volatile float super_vector[28];
volatile bool  FLAG_ADC_COMPLETE = false;
volatile bool  FLAG_DATA_READY   = false;
 
// ============================================================
// PINES Y CONSTANTES
// ============================================================
#define MUX_S0_PIN       2
#define ADC_MUX_SIG_PIN  26
#define I2C_SDA_PIN      6
#define I2C_SCL_PIN      7
#define UART0_TX_PIN     0
#define UART1_TX_PIN     8
 
#define NUM_SENSORS      14
#define ADC_DATA_DMA_CH  0
#define ADC_TRIG_DMA_CH  1
 
// ============================================================
// CONSTANTES DE CALIBRACIÓN NEUTRA (FILTRO ESPACIAL DE LA IMU)
// ============================================================
#define CAL_NEUTRO_AX   (-0.300729f)
#define CAL_NEUTRO_AY   (-0.548062f)
#define CAL_NEUTRO_AZ   (0.577612f)
#define CAL_NEUTRO_GX   (-19.279268f)
#define CAL_NEUTRO_GY   (1.395114f)
#define CAL_NEUTRO_GZ   (2.734779f)

static uint16_t hall_samples[NUM_SENSORS];
static mpu6050_config_t g_imu_cfg;
 
typedef enum {
    ESTADO_INIT,
    ESTADO_IDLE
} core0_state_t;
 
core0_state_t current_state = ESTADO_INIT;

// ============================================================
// VARIABLES DE CONTROL: FILTRO DE ESTABILIDAD
// ============================================================
static int  g_ultimo_voto = -1;
static int  g_conteo_consecutivo = 0;
static int  g_gesto_estable_actual = -1;
static bool g_audio_reproducido = false;
 
// ============================================================
// FUNCIONES DE CONTROL DFPLAYER MINI (UART0)
// ============================================================
/**
 * @brief Envía una trama de 10 bytes al DFPlayer Mini.
 */
static void dfplayer_send_cmd(uint8_t cmd, uint16_t param) {
    uint8_t buffer[10];
    buffer[0] = 0x7E; // Byte de Inicio
    buffer[1] = 0xFF; // Versión
    buffer[2] = 0x06; // Longitud de datos
    buffer[3] = cmd;  // Comando principal
    buffer[4] = 0x00; // Feedback (0 = No requiere confirmación)
    buffer[5] = (uint8_t)(param >> 8);   // Parámetro MSB (Byte alto)
    buffer[6] = (uint8_t)(param & 0xFF); // Parámetro LSB (Byte bajo)
    
    // Cálculo del Checksum (0 - Suma de los bytes 1 al 6)
    uint16_t checksum = 0 - (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6]);
    buffer[7] = (uint8_t)(checksum >> 8);
    buffer[8] = (uint8_t)(checksum & 0xFF);
    buffer[9] = 0xEF; // Byte de Finalización

    // Transmitir por hardware UART0
    for (int i = 0; i < 10; i++) {
        uart_putc(uart0, buffer[i]);
    }
    sleep_ms(20); // Pequeña pausa para que el módulo de audio procese
}

/**
 * @brief Ajusta el volumen (0 a 30)
 */
static void dfplayer_set_volume(uint8_t volume) {
    if(volume > 30) volume = 30;
    dfplayer_send_cmd(0x06, volume); 
}

/**
 * @brief Reproduce la pista especificada dentro de la carpeta "mp3"
 */
static void dfplayer_play_mp3(uint16_t track_number) {
    dfplayer_send_cmd(0x12, track_number);
}

// ============================================================
// ISR — SEGUNDO PLANO
// ============================================================
bool timer_50hz_isr(struct repeating_timer *t) {
    pio_sm_put(pio0, 0, 1);
    return true;
}
 
void dma_data_isr(void) {
    dma_hw->ints0 = 1u << ADC_DATA_DMA_CH;
    FLAG_ADC_COMPLETE = true;
}
 
// ============================================================
// CONFIGURACIÓN DEL PIPELINE DE HARDWARE
// ============================================================
static void config_pipeline_hardware(void) {
    uint offset = pio_add_program(pio0, &mux_adc_program);
    pio_sm_config c_pio = mux_adc_program_get_default_config(offset);
 
    for (int i = 0; i < 4; i++) {
        pio_gpio_init(pio0, MUX_S0_PIN + i);
    }
    pio_sm_set_consecutive_pindirs(pio0, 0, MUX_S0_PIN, 4, true);
    sm_config_set_out_pins(&c_pio, MUX_S0_PIN, 4);
 
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    sm_config_set_clkdiv(&c_pio, div);
    pio_sm_init(pio0, 0, offset, &c_pio);
    pio_sm_set_enabled(pio0, 0, true);
 
    adc_init();
    adc_gpio_init(ADC_MUX_SIG_PIN);
    adc_select_input(0);
    adc_fifo_setup(true, true, 1, true, false);
 
    dma_channel_config c_data = dma_channel_get_default_config(ADC_DATA_DMA_CH);
    channel_config_set_transfer_data_size(&c_data, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data, false);
    channel_config_set_write_increment(&c_data, true);
    channel_config_set_dreq(&c_data, DREQ_ADC);
 
    dma_channel_configure(
        ADC_DATA_DMA_CH, &c_data,
        hall_samples,       
        &adc_hw->fifo,
        NUM_SENSORS,
        false
    );
 
    dma_channel_set_irq0_enabled(ADC_DATA_DMA_CH, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_data_isr);
    irq_set_enabled(DMA_IRQ_0, true);
 
    dma_channel_config c_trig = dma_channel_get_default_config(ADC_TRIG_DMA_CH);
    channel_config_set_transfer_data_size(&c_trig, DMA_SIZE_32);
    channel_config_set_read_increment(&c_trig, false);
    channel_config_set_write_increment(&c_trig, false);
    channel_config_set_dreq(&c_trig, DREQ_PIO0_RX0);
 
    dma_channel_configure(
        ADC_TRIG_DMA_CH, &c_trig,
        hw_set_alias(&adc_hw->cs),
        &pio0->rxf[0],
        NUM_SENSORS,
        true
    );
 
    dma_channel_start(ADC_DATA_DMA_CH);
 
    static struct repeating_timer timer;
    add_repeating_timer_ms(20, timer_50hz_isr, NULL, &timer);
}
 
// ============================================================
// INICIALIZACIÓN DEL DRIVER IMU
// ============================================================
static bool init_imu(void) {
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
 
    g_imu_cfg = mpu6050_get_default_config();
    g_imu_cfg.i2c_port = i2c1;
 
    mpu6050_err_t err = mpu6050_init(&g_imu_cfg);
    if (err != MPU6050_OK) {
        printf("[ERROR] MPU-6050 no responde en I2C1.\n");
        return false;
    }
    return true;
}
 
// ============================================================
// CORE 0: LAZO PRINCIPAL (FSM)
// ============================================================
int main(void) {
    float imu_data[6];
 
    while (true) {
        switch (current_state) {
 
            // --------------------------------------------------
            case ESTADO_INIT:
            // --------------------------------------------------
                stdio_init_all();
                sleep_ms(2000); 
                printf("\n[SISTEMA ECHO] Inicializando...\n"); 
                fflush(stdout);
 
                uart_init(uart0, 9600);
                uart_init(uart1, 115200);
                gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
                gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
                
                // Inicializar volumen del DFPlayer a 25 (de 30)
                dfplayer_set_volume(25);
                printf("[AUDIO] DFPlayer configurado a volumen 25.\n");
 
                init_imu();
                config_pipeline_hardware();
 
                // Evitamos recalibrar los 5 segundos ya que tenemos tus valores estáticos guardados
                // Si algún día cambias de guante, puedes volver a habilitar el ciclo for de 250 muestras.
                printf("[FILTRO] Offset de IMU Neutro fijado estaticamente.\n"); 
                fflush(stdout);
 
                printf("[CORE 1] Lanzando IA (MLP)...\n"); 
                multicore_launch_core1(core1_main);
 
                current_state = ESTADO_IDLE;
                printf("[ESTADO] Entrando en ESTADO_IDLE. Listo para procesar gestos.\n"); 
                fflush(stdout);
                break;
 
            // --------------------------------------------------
            case ESTADO_IDLE:
            // --------------------------------------------------
                if (multicore_fifo_rvalid()) {
                    uint32_t gesto_id = multicore_fifo_pop_blocking();
                    
                    if ((int)gesto_id == g_ultimo_voto) {
                        g_conteo_consecutivo++;
                    } else {
                        g_ultimo_voto = (int)gesto_id;
                        g_conteo_consecutivo = 1; 
                    }
                    
                    if (g_conteo_consecutivo >= 13) {
                        if (g_gesto_estable_actual != (int)gesto_id) {
                            g_gesto_estable_actual = (int)gesto_id;
                            g_audio_reproducido = false; 
                        }
                        
                        if (!g_audio_reproducido) {
                            switch (g_gesto_estable_actual) {
                                
                                case 1:
                                    dfplayer_play_mp3(1); // Llama al archivo 0001.mp3 en la carpeta mp3
                                    printf("\nGesto detectado: A\n");
                                    g_audio_reproducido = true;
                                    break;
                                    
                                case 2:
                                    dfplayer_play_mp3(2); // Llama al archivo 0002.mp3
                                    printf("\nGesto detectado: B\n");
                                    g_audio_reproducido = true;
                                    break;
                                    
                                case 4:
                                    dfplayer_play_mp3(4); // Llama al archivo 0004.mp3
                                    printf("\nGesto detectado: 4 (Gracias)\n");
                                    g_audio_reproducido = true;
                                    break;
                                    
                                default:
                                    // Estado neutro u otra clase: No hace sonido pero frena el spam serial
                                    g_audio_reproducido = true;
                                    break;
                            }
                        }
                    }
                }
 
                if (FLAG_ADC_COMPLETE) {
                    mpu6050_err_t imu_err = mpu6050_read_to_vector(&g_imu_cfg, imu_data);
                    if (imu_err != MPU6050_OK) {
                        for (int i = 0; i < 6; i++) imu_data[i] = 0.0f;
                    }
 
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        uint16_t raw = hall_samples[NUM_SENSORS - 1 - i];
                        if (raw & (1 << 15)) {
                            super_vector[i] = 0.0f;
                        } else {
                            super_vector[i] = (float)(raw & 0x0FFF) * (3.3f / 4096.0f);
                        }
                    }
 
                    // === FILTRO ESPACIAL DINÁMICO ACTIVO ===
                    super_vector[14] = imu_data[0] - CAL_NEUTRO_AX; 
                    super_vector[15] = imu_data[1] - CAL_NEUTRO_AY; 
                    super_vector[16] = imu_data[2] - CAL_NEUTRO_AZ; 
                    super_vector[17] = imu_data[3] - CAL_NEUTRO_GX; 
                    super_vector[18] = imu_data[4] - CAL_NEUTRO_GY; 
                    super_vector[19] = imu_data[5] - CAL_NEUTRO_GZ; 
                    
                    for (int i = 20; i < 28; i++) {
                        super_vector[i] = 0.0f;
                    }
 
                    FLAG_DATA_READY   = true;
                    FLAG_ADC_COMPLETE = false;
 
                    dma_channel_set_trans_count(ADC_DATA_DMA_CH, NUM_SENSORS, false);
                    dma_channel_set_write_addr(ADC_DATA_DMA_CH, hall_samples, true);
                    dma_channel_set_trans_count(ADC_TRIG_DMA_CH, NUM_SENSORS, true);
                }
                break;
        }
    }
    return 0;
}