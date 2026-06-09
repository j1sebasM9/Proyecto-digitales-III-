/**
 * @file    core0_main.c
 * @brief   Subsistema CORE 0 - Proyecto "echo" (Versión de Producción Final)
 * @details Integra el driver real del MPU-6050 sustituyendo el mock funcional.
 * Arquitectura: IRQ + Polling, FSM, Pipeline PIO+DMA autónomo.
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
#include "mpu6050.h"          // Driver real del sensor inercial
 

// PINES Y CONSTANTES

 
#define MUX_S0_PIN       2
#define ADC_MUX_SIG_PIN  26
#define I2C_SDA_PIN      6
#define I2C_SCL_PIN      7
#define UART0_TX_PIN     0
#define UART1_TX_PIN     8
 
#define NUM_SENSORS      14
#define ADC_DATA_DMA_CH  0
#define ADC_TRIG_DMA_CH  1
 

// BANDERAS ATÓMICAS DE SINCRONIZACIÓN

 
volatile bool FLAG_ADC_COMPLETE = false;
volatile bool FLAG_DATA_READY   = false;
 

// VECTORES DE TRABAJO

 
uint16_t hall_samples[NUM_SENSORS];
float    super_vector[20];           // [0-13] Hall | [14-19] IMU
 

// INSTANCIA DE CONFIGURACIÓN DEL DRIVER IMU

 
static mpu6050_config_t g_imu_cfg;
 

// FSM

 
typedef enum {
    ESTADO_INIT,
    ESTADO_IDLE
} core0_state_t;
 
core0_state_t current_state = ESTADO_INIT;
 

// ISR — SEGUNDO PLANO

 
bool timer_50hz_isr(struct repeating_timer *t) {
    pio_sm_put(pio0, 0, 1);  // Token único al PIO, no bloqueante
    return true;
}
 
void dma_data_isr(void) {
    dma_hw->ints0 = 1u << ADC_DATA_DMA_CH;
    FLAG_ADC_COMPLETE = true;
}
 

// CONFIGURACIÓN DEL PIPELINE DE HARDWARE

 
static void config_pipeline_hardware(void) {
    // --- PIO ---
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
 
    // --- ADC ---
    adc_init();
    adc_gpio_init(ADC_MUX_SIG_PIN);
    adc_select_input(0);
    adc_fifo_setup(true, true, 1, true, false);
 
    // --- DMA Canal 0: Datos (ADC FIFO → SRAM) ---
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
 
    // --- DMA Canal 1: Disparo síncrono (PIO FIFO → ADC CS SET ALIAS) ---
    dma_channel_config c_trig = dma_channel_get_default_config(ADC_TRIG_DMA_CH);
    channel_config_set_transfer_data_size(&c_trig, DMA_SIZE_32);
    channel_config_set_read_increment(&c_trig, false);
    channel_config_set_write_increment(&c_trig, false);
    channel_config_set_dreq(&c_trig, DREQ_PIO0_RX0);
 
    dma_channel_configure(
        ADC_TRIG_DMA_CH, &c_trig,
        hw_set_alias(&adc_hw->cs),  // OR atómico: activa EN y START_ONCE sin destruir AINSEL
        &pio0->rxf[0],
        NUM_SENSORS,
        true
    );
 
    dma_channel_start(ADC_DATA_DMA_CH);
 
    // --- Timer 50 Hz ---
    static struct repeating_timer timer;
    add_repeating_timer_ms(20, timer_50hz_isr, NULL, &timer);
}
 

// INICIALIZACIÓN DEL DRIVER IMU (CORREGIDA PARA I2C1)

 
/**
 * @brief   Inicializa el bus I2C1 y el driver del MPU-6050 en GP6/GP7.
 * @return  true si exitoso, false si el dispositivo no responde.
 */
static bool init_imu(void) {
    // 1. Inicializar el bloque específico I2C1 a Fast Mode (400 kHz)
    i2c_init(i2c1, 400 * 1000); 

    // Asignar físicamente los pines GP6 y GP7 al periférico de hardware I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
 
    // Obtener configuración por defecto del driver y apuntar explícitamente a I2C1
    g_imu_cfg = mpu6050_get_default_config();
    g_imu_cfg.i2c_port = i2c1; 
 
    // Inicializar el sensor con verificación de identidad y reset por software
    mpu6050_err_t err = mpu6050_init(&g_imu_cfg);
 
    if (err != MPU6050_OK) {
        printf("[ERROR] MPU-6050 no responde en I2C1. Codigo: %d\n", err);
        return false;
    }
 
    printf("[OK] MPU-6050 inicializado correctamente en I2C1.\n");
    return true;
}
 

// CORE 0: LAZO PRINCIPAL (FSM)

 
int main(void) {
    float imu_data[6];
 
    while (true) {
        switch (current_state) {
 
            case ESTADO_INIT:
                stdio_init_all();
                
                // Inicializar módulos UART físicos
                uart_init(uart0, 9600);            
                uart_init(uart1, 115200);          
                
                //Conectar físicamente los pines GP0 y GP8 a los periféricos UART
                gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
                gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
                
                // Inicializar la IMU de forma segura en I2C1
                init_imu();
                
                // Configurar toda la infraestructura PIO+ADC+DMA+Timer
                config_pipeline_hardware();

                // Descomentar cuando core1_main esté definido
                // multicore_launch_core1(core1_main);
                
                current_state = ESTADO_IDLE;
                break;
 
   
            case ESTADO_IDLE:

 
                if (FLAG_ADC_COMPLETE) {
                    // --- RAMA A: Adquisición completa, empaquetar super_vector ---
 
                    // Leer IMU: transacción I2C de 14 bytes atómica usando la estructura global correcta
                    mpu6050_err_t imu_err = mpu6050_read_to_vector(&g_imu_cfg, imu_data);
 
                    if (imu_err != MPU6050_OK) {
                        // Política de falla tolerante: datos IMU en cero, pipeline continúa
                        for (int i = 0; i < 6; i++) imu_data[i] = 0.0f;
                    }
 
                    // Empaquetar sensores Hall con compensación del barrido regresivo del PIO
                    // El PIO barre de C2 (x=13,~x=0010) a C15 (x=0,~x=1111)
                    // La inversión de índice reconstruye el orden anatómico: C15→C2
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        uint16_t raw = hall_samples[NUM_SENSORS - 1 - i];
 
                        if (raw & (1 << 15)) {
                            // Bit de error del ADC activo: muestra descartada por seguridad
                            super_vector[i] = 0.0f;
                        } else {
                            // Conversión directa a voltaje (referencia 3.3V regulada desde LiPo)
                            super_vector[i] = (float)(raw & 0x0FFF) * (3.3f / 4096.0f);
                        }
                    }
 
                    // Inyectar datos IMU al final del super_vector [14..19]
                    for (int i = 0; i < 6; i++) {
                        super_vector[NUM_SENSORS + i] = imu_data[i];
                    }
 

                    //Este ciclo for es de prueba 
                    /*for(int i = 0; i < 20; i++) {
                        printf("%.3f", super_vector[i]);
                        if (i < 19) {
                            printf(","); // Imprime coma entre valores
                        }
                    }*/
            
                    //printf("\n");
                    
                    // Sincronización inter-núcleo: despertar al Core 1
                    FLAG_DATA_READY   = true;
                    FLAG_ADC_COMPLETE = false;  // Consumir bandera
 
                    // Rearmar pipeline DMA (PIO está en pull block: ventana de carrera cerrada)
                    dma_channel_set_trans_count(ADC_DATA_DMA_CH, NUM_SENSORS, false);
                    dma_channel_set_write_addr(ADC_DATA_DMA_CH, hall_samples, true);
                    dma_channel_set_trans_count(ADC_TRIG_DMA_CH, NUM_SENSORS, true);
                }
                else if (multicore_fifo_rvalid()) {
                    //  RAMA B: Gesto clasificado por Core 1 disponible en SIO FIFO 
                    uint32_t gesto_id = multicore_fifo_pop_blocking();
 
                    uart_putc(uart0, (char)gesto_id);                        // Enviar al DFPlayer Mini
                    printf("Gesto detectado por Core 1: %lu\n", gesto_id);  // Monitor de la PC
                }
                else {
                    // Sin eventos pendientes: suspender CPU de forma segura hasta próxima IRQ
                    __wfi();
                }
                break;
        }
    }
 
    return 0;
}