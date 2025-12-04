
/*
 * Modelo identificado del motor DC
 * Generado automáticamente
 */

// Función de transferencia continua:
// G(s) = 599.8892 / (0.3992*s + 1)

// Modelo discretizado (Ts = 0.2488 s):
// Ecuación en diferencias:
// y[k] = 0.524847*y[k-1] + 712.598082*u[k] + 712.598082*u[k-1]

#define K_MOTOR 599.8892f      // RPM/V
#define TAU_MOTOR 0.3992f  // s
#define TS_SAMPLE 0.2488f             // s

// Coeficientes de la ecuación en diferencias
#define A1 0.524847f
#define B0 712.598082f
#define B1 712.598082f

float y_prev = 0.0;  // y[k-1]
float u_prev = 0.0;  // u[k-1]

float actualizar_modelo(float u) {
    float y = A1 * y_prev + B0 * u + B1 * u_prev;
    
    // Actualizar estados
    y_prev = y;
    u_prev = u;
    
    return y;
}
