/**
 * @file    core1_ai.c
 * @brief   Subsistema CORE 1 - Inferencia de IA - Proyecto "echo"
 * @details Implementa la FSM de 4 estados del Core 1 según la arquitectura
 *          de software del documento técnico del proyecto echo.
 *
 *          FSM:
 *            ESTADO_WAIT_DATA   → Polling de FLAG_DATA_READY
 *            ESTADO_DSP_FILTER  → Filtro de media móvil sobre sensores Hall
 *            ESTADO_VAR_GATE    → Varianza IMU → decide MLP estática o dinámica
 *            ESTADO_CONFIRMATION→ Modo diagnóstico: imprime RAW, NORM y predicción
 *
 *          Patrón: IRQ + Polling (polling activo de FLAG_DATA_READY)
 *          Núcleo: Core 1 del RP2040, ejecuta en paralelo con Core 0
 */

#include "core1_ai.h"
#include "common.h"
#include "mlp_weights.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#define MOVING_AVG_WIN      4
#define VAR_GATE_THRESHOLD  0.5f

// ============================================================
// DIAGNÓSTICO: número de muestras a imprimir antes de silenciar
// Cambia este valor a 0 para desactivar el diagnóstico
// ============================================================
#define DIAG_NUM_MUESTRAS   1

static inline void relu(float *v, int len) {
    for (int i = 0; i < len; i++) {
        if (v[i] < 0.0f) v[i] = 0.0f;
    }
}

static void softmax(float *v, int len) {
    float max_val = v[0];
    for (int i = 1; i < len; i++) {
        if (v[i] > max_val) max_val = v[i];
    }
    float suma = 0.0f;
    for (int i = 0; i < len; i++) {
        v[i] = expf(v[i] - max_val);
        suma += v[i];
    }
    for (int i = 0; i < len; i++) {
        v[i] /= suma;
    }
}

static void mlp_forward(const float *input, float *probs_out) {
    float layer0_out[L0_OUT];
    float layer1_out[L1_OUT];

    for (int j = 0; j < L0_OUT; j++) {
        float suma = BIAS_L0[j];
        for (int i = 0; i < L0_IN; i++) {
            suma += input[i] * WEIGHTS_L0[i][j];
        }
        layer0_out[j] = suma;
    }
    relu(layer0_out, L0_OUT);

    for (int j = 0; j < L1_OUT; j++) {
        float suma = BIAS_L1[j];
        for (int i = 0; i < L1_IN; i++) {
            suma += layer0_out[i] * WEIGHTS_L1[i][j];
        }
        layer1_out[j] = suma;
    }
    relu(layer1_out, L1_OUT);

    for (int j = 0; j < L2_OUT; j++) {
        float suma = BIAS_L2[j];
        for (int i = 0; i < L2_IN; i++) {
            suma += layer1_out[i] * WEIGHTS_L2[i][j];
        }
        probs_out[j] = suma;
    }

    softmax(probs_out, NUM_CLASSES);
}

// ============================================================
// PUNTO DE ENTRADA DEL CORE 1
// ============================================================

void core1_main(void) {

    float local_hall[NUM_FEATURES];
    float local_imu[6];
    float hall_filtered[NUM_FEATURES];
    float hall_normalized[NUM_FEATURES];
    float probs[NUM_CLASSES];

    float moving_avg_buf[NUM_FEATURES][MOVING_AVG_WIN] = {{0}};
    int   moving_avg_idx = 0;

    uint32_t ultimo_gesto         = 999;
    int      contador_consecutivo = 0;

    // Contador para el bloque de diagnóstico
    // Se imprime DIAG_NUM_MUESTRAS veces y luego se silencia automáticamente
    int diag_contador = 0;

    while (true) {

        // ======================================================
        // ESTADO 1: WAIT_DATA
        // ======================================================
        if (!FLAG_DATA_READY) {
            continue;
        }

        // ======================================================
        // ESTADO 2: DSP_FILTER
        // ======================================================

        // Copia del super_vector ANTES de bajar la bandera
        for (int i = 0; i < NUM_FEATURES; i++) {
            local_hall[i] = super_vector[i];
        }
        for (int i = 0; i < 6; i++) {
            local_imu[i] = super_vector[NUM_FEATURES + i];
        }

        FLAG_DATA_READY = false;

        // ============================================================
        // BLOQUE DE DIAGNÓSTICO — Se ejecuta solo DIAG_NUM_MUESTRAS veces
        // Imprime los valores RAW y NORM para verificar que el mapeo
        // de sensores y la normalización son correctos antes de inferir
        // ============================================================
        if (diag_contador < DIAG_NUM_MUESTRAS) {
            diag_contador++;

            printf("\n===== DIAGNOSTICO MUESTRA %d/%d =====\n",
                   diag_contador, DIAG_NUM_MUESTRAS);

            // --- Valores RAW del guante ---
            printf("RAW  [H0..H13]: ");
            for (int i = 0; i < NUM_FEATURES; i++) {
                printf("%.3f ", local_hall[i]);
            }
            printf("\n");

            // --- Valores del SCALER_MEAN de referencia ---
            printf("MEAN [H0..H13]: ");
            for (int i = 0; i < NUM_FEATURES; i++) {
                printf("%.3f ", SCALER_MEAN[i]);
            }
            printf("\n");

            // --- Valores normalizados (z-score) ---
            printf("NORM [H0..H13]: ");
            for (int i = 0; i < NUM_FEATURES; i++) {
                float norm = (local_hall[i] - SCALER_MEAN[i]) / SCALER_STD[i];
                printf("%.2f ", norm);
            }
            printf("\n");

            // --- Valores IMU ---
            printf("IMU  [Ax Ay Az Gx Gy Gz]: %.3f %.3f %.3f %.3f %.3f %.3f\n",
                   local_imu[0], local_imu[1], local_imu[2],
                   local_imu[3], local_imu[4], local_imu[5]);

            printf("=====================================\n\n");

            // Pausa para leer cómodamente en el monitor serial
            sleep_ms(1500);
            continue; // No inferir durante el diagnóstico
        }

        // ======================================================
        // Filtro de media móvil (activo solo después del diagnóstico)
        // ======================================================
        for (int i = 0; i < NUM_FEATURES; i++) {
            moving_avg_buf[i][moving_avg_idx] = local_hall[i];
            float suma = 0.0f;
            for (int w = 0; w < MOVING_AVG_WIN; w++) {
                suma += moving_avg_buf[i][w];
            }
            hall_filtered[i] = suma / MOVING_AVG_WIN;
        }
        moving_avg_idx = (moving_avg_idx + 1) % MOVING_AVG_WIN;

        // ======================================================
        // ESTADO 3: VAR_GATE
        // ======================================================
        float media_gyro = 0.0f;
        for (int i = 3; i < 6; i++) {
            media_gyro += local_imu[i];
        }
        media_gyro /= 3.0f;

        float varianza_gyro = 0.0f;
        for (int i = 3; i < 6; i++) {
            float diff = local_imu[i] - media_gyro;
            varianza_gyro += diff * diff;
        }
        varianza_gyro /= 3.0f;

        for (int i = 0; i < NUM_FEATURES; i++) {
            hall_normalized[i] = (hall_filtered[i] - SCALER_MEAN[i]) / SCALER_STD[i];
        }

        if (varianza_gyro <= VAR_GATE_THRESHOLD) {
            mlp_forward(hall_normalized, probs);
        } else {
            mlp_forward(hall_normalized, probs);
        }

        // ===================================================
        // ESTADO 4: CONFIRMATION (modo diagnóstico activo)
        // ===================================================
        int   clase_ganadora = 0;
        float prob_max       = probs[0];

        for (int i = 1; i < NUM_CLASSES; i++) {
            if (probs[i] > prob_max) {
                prob_max      = probs[i];
                clase_ganadora = i;
            }
        }

        uint32_t caracter_ascii = (uint32_t)CLASS_MAP[clase_ganadora];

        printf("[Core 1 IA] Prediccion: %c | Certeza: %.2f%%\n",
               (char)caracter_ascii, prob_max * 100.0f);

        if (multicore_fifo_wready()) {
            multicore_fifo_push_blocking(caracter_ascii);
        }

        sleep_ms(250);

    } // fin while(true)
}