/*
 * Modelo identificado del motor DC
 * Generado automáticamente
 * Fecha: 2025-12-03 14:30:38
 */

// ============================================================================
// FUNCIÓN DE TRANSFERENCIA CONTINUA
// ============================================================================
// G(s) = 999.3659 / (0.3008*s + 1)
//
// Polo: s = -3.32 rad/s
// Ancho de banda: 0.53 Hz
// Ajuste: 100.0%

// ============================================================================
// MODELO DISCRETIZADO (Método Tustin, Ts = 0.1887 s)
// ============================================================================
// Ecuación en diferencias:
// y[k] = 0.522515*y[k-1] + 1192.956288*u[k] + 1192.956288*u[k-1]

#ifndef MODELO_MOTOR_H
#define MODELO_MOTOR_H

// Parámetros del modelo continuo
#define K_MOTOR 999.3659f      // Ganancia [RPM/V]
#define TAU_MOTOR 0.3008f  // Constante de tiempo [s]
#define TS_SAMPLE 0.1887f             // Período de muestreo [s]
#define FS_SAMPLE 5.30f           // Frecuencia de muestreo [Hz]

// Coeficientes de la ecuación en diferencias
#define A1 0.522515f
#define B0 1192.956288f
#define B1 1192.956288f

// Variables de estado (declarar como static en tu código)
typedef struct {
    float y_prev;  // y[k-1]
    float u_prev;  // u[k-1]
} ModeloMotor_t;

// ============================================================================
// FUNCIONES
// ============================================================================

/**
 * Inicializa el modelo del motor
 * @param modelo: Puntero a la estructura del modelo
 */
void modelo_motor_init(ModeloMotor_t *modelo) {
    modelo->y_prev = 0.0f;
    modelo->u_prev = 0.0f;
}

/**
 * Actualiza el modelo del motor (llamar cada Ts)
 * @param modelo: Puntero a la estructura del modelo
 * @param u: Entrada de control [V]
 * @return: Velocidad estimada [RPM]
 */
float modelo_motor_update(ModeloMotor_t *modelo, float u) {
    // Ecuación en diferencias
    float y = A1 * modelo->y_prev + B0 * u + B1 * modelo->u_prev;
    
    // Actualizar estados
    modelo->y_prev = y;
    modelo->u_prev = u;
    
    return y;
}

/**
 * Resetea el modelo del motor
 * @param modelo: Puntero a la estructura del modelo
 */
void modelo_motor_reset(ModeloMotor_t *modelo) {
    modelo->y_prev = 0.0f;
    modelo->u_prev = 0.0f;
}

#endif // MODELO_MOTOR_H

// ============================================================================
// EJEMPLO DE USO
// ============================================================================
/*
#include "modelo_motor.h"

// Variable global del modelo
static ModeloMotor_t motor_modelo;

void setup() {
    // Inicializar modelo
    modelo_motor_init(&motor_modelo);
    
    // Configurar timer para muestreo a 5.30 Hz
    // ...
}

void loop() {
    // Esta función debe llamarse cada 188.65 ms
    
    // Leer entrada de control (PWM convertido a voltaje)
    float voltaje = leer_voltaje_pwm();
    
    // Actualizar modelo
    float velocidad_estimada = modelo_motor_update(&motor_modelo, voltaje);
    
    // Usar velocidad estimada para control, logging, etc.
    // ...
}
*/
