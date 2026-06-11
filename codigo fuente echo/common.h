/**
 * @file    common.h
 * @brief   Variables y constantes compartidas entre Core 0 y Core 1 - Proyecto "echo"
 * @details Define el contrato de interfaz entre los dos subsistemas del RP2040.
 *          Todas las variables declaradas aquí son definidas en core0_main.c
 *          y accedidas por core1_ai.c mediante enlace externo.
 *
 *          Sincronización inter-núcleo:
 *            Core 0 (Productor) → escribe super_vector y levanta FLAG_DATA_READY
 *            Core 1 (Consumidor) → lee super_vector y baja FLAG_DATA_READY
 */
 
#ifndef COMMON_H
#define COMMON_H
 
#include <stdbool.h>
#include <stdint.h>
 
// ============================================================
// DIMENSIONES DE LA RED NEURONAL
// Arquitectura: 14 → 32 → 16 → 9
// Compatible con los pesos exportados por el pipeline TinyML
// ============================================================
#define NUM_FEATURES  14   // Entradas: 14 sensores Hall (mano estática)
#define L0_IN         14   // Capa oculta 0: entradas
#define L0_OUT        32   // Capa oculta 0: neuronas
#define L1_IN         32   // Capa oculta 1: entradas
#define L1_OUT        16   // Capa oculta 1: neuronas
#define L2_IN         16   // Capa de salida: entradas
#define L2_OUT        9    // Capa de salida: clases (1,2,3,4,5,A,B,C,D)
#define NUM_CLASSES   9    // Total de clases clasificables
 
// ============================================================
// UMBRAL DE CERTEZA PARA CONFIRMACIÓN DE GESTO
// El Core 1 solo despacha un gesto si Softmax supera este valor
// durante ANTIRREBOTE_CICLOS ciclos consecutivos (doc: 90%)
// ============================================================
#define SOFTMAX_THRESHOLD     0.65f
#define ANTIRREBOTE_CICLOS    1
 
// ============================================================
// VECTOR DE DATOS COMPARTIDO (Productor: Core 0 / Consumidor: Core 1)
// [0-13]  → Voltajes de los 14 sensores Hall en orden anatómico
// [14-19] → 6 ejes IMU: [Ax, Ay, Az, Gx, Gy, Gz]
// volatile garantiza que el compilador no optimice los accesos
// desde los dos núcleos ejecutando en paralelo
// ============================================================
extern volatile float super_vector[28];
 
// ============================================================
// BANDERAS ATÓMICAS DE SINCRONIZACIÓN
// Escritas por ISRs y leídas por lazos de polling
// volatile es obligatorio para evitar optimizaciones del compilador
// ============================================================
extern volatile bool FLAG_ADC_COMPLETE;  // ISR DMA → Core 0: ráfaga lista
extern volatile bool FLAG_DATA_READY;    // Core 0 → Core 1: vector empaquetado
 
#endif // COMMON_H