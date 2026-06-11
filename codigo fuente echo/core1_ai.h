/**
 * @file    core1_ai.h
 * @brief   Interfaz pública del subsistema de IA - Proyecto "echo"
 * @details Expone únicamente el punto de entrada del Core 1.
 *          Esta función es pasada como argumento a multicore_launch_core1()
 *          desde core0_main.c una vez que el hardware está inicializado.
 */
 
#ifndef CORE1_AI_H
#define CORE1_AI_H
 
/**
 * @brief  Punto de entrada del Core 1. Ejecuta la FSM de inferencia de IA.
 * @note   Llamar únicamente desde multicore_launch_core1(core1_main).
 *         Nunca retorna — contiene un bucle infinito de polling.
 */
void core1_main(void);
 
#endif // CORE1_AI_H