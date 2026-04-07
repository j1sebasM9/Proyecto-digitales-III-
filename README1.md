# echo. 

Título del Proyecto:
echo: Guante Biónico para la Traducción Autónoma de Lenguaje de Señas mediante Sensores de Efecto Hall, Inferencia en el Borde (Edge AI) y Síntesis de Voz Integrada.

Resumen Técnico:

El proyecto consiste en el diseño y desarrollo de un sistema wearable de bajo costo y alta durabilidad destinado a cerrar la brecha de comunicación para la comunidad con discapacidad auditiva. A diferencia de las soluciones convencionales que utilizan sensores de flexión resistivos (sujetos a fatiga mecánica y deriva de señal), este sistema implementa una arquitectura de detección magnética sin contacto que garantiza una vida útil superior del hardware.

Se utilizan 14 sensores de efecto hall lineales (modelo 49E) distribuidos estratégicamente en las articulaciones de los dedos (DIP, PIP y MCP). Cada sensor interactúa con un imán de neodimio N35; al flexionar el dedo, la variación en la densidad del flujo magnético es capturada como una señal analógica proporcional al ángulo de flexión. Para complementar la "forma" de la mano con la "orientación" en el espacio, el sistema integra una Unidad de Medición Inercial (IMU MPU6050) en el dorso.

El procesamiento y la digitalización se realizan mediante una Raspberry Pi Pico, utilizando un multiplexor analógico (CD74HC4067) para expandir la capacidad de lectura de los 14 nodos. El sistema ejecuta localmente un modelo de Red Neuronal Artificial (ANN) optimizado para microcontroladores (TinyML), permitiendo la clasificación de gestos en tiempo real.

Como elemento diferenciador, el sistema incorpora un módulo de audio DFPlayer Mini junto a un altavoz miniatura, permitiendo que el guante traduzca los gestos directamente en voz audible de forma autónoma. Esto elimina la dependencia de dispositivos externos o conectividad a la nube, garantizando un sistema de comunicación inmediata, privado y totalmente portable.
