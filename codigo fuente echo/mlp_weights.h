#ifndef MLP_WEIGHTS_H
#define MLP_WEIGHTS_H

#include "common.h"

// Declaración externa de los parámetros de escalado (StandardScaler)
extern const float SCALER_MEAN[NUM_FEATURES];
extern const float SCALER_STD[NUM_FEATURES];

// Declaración externa de las matrices de pesos y sesgos de la red neuronal
extern const float WEIGHTS_L0[L0_IN][L0_OUT];
extern const float BIAS_L0[L0_OUT];

extern const float WEIGHTS_L1[L1_IN][L1_OUT];
extern const float BIAS_L1[L1_OUT];

extern const float WEIGHTS_L2[L2_IN][L2_OUT];
extern const float BIAS_L2[L2_OUT];

// Diccionario ASCII para mapear el Argmax al gesto real
extern const char CLASS_MAP[NUM_CLASSES];

#endif // MLP_WEIGHTS_H