/**
 * Funções e constantes relacionados ao kinect
 */
#ifndef KINECT_H
#define KINECT_H

#include "Arduino.h"

#include <Servo.h>

typedef enum {
 KINECT_STOP = 0,
 KINECT_LEFT = 1,
 KINECT_RIGHT = 2,
 KINECT_SWEEP = 3,
 KINECT_RESET = 4,

 KINECT_NUM_STATES
} kinect_states;

/* Inicializa a biblioteca Servo */
void kinect_init(const int servoPin);

/* Atualiza o angulo do kinect de acordo com o estado atual */
void kinect_update();

/* Atualiza o estado atual do movimento do kinect */
void kinect_setState(kinect_states state);

kinect_states kinect_currentState();

float kinect_currentAngle();

void kinect_setAngle(float angle);

#endif

