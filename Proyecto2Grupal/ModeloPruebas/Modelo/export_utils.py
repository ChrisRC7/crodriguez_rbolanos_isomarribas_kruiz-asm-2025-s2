"""
============================================================================
MÓDULO DE EXPORTACIÓN
Funciones para exportar modelos en diferentes formatos
============================================================================
"""

import numpy as np
from config import *


def exportar_numpy(modelo, analisis, archivo=None):
    """
    Exporta modelo en formato NumPy (.npz)
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        archivo: Nombre del archivo (None usa config)
    """
    if archivo is None:
        archivo = f'{EXPORT_PREFIX}.npz'
    
    np.savez(archivo,
             K=modelo['K'],
             tau=modelo['tau'],
             polo=modelo['polo'],
             fit=modelo['fit'],
             rmse=modelo.get('rmse', 0),
             BW_hz=analisis['BW_hz'],
             fs_recomendada=analisis['fs_recomendada'],
             Ts_recomendado=analisis['Ts_recomendado'])
    
    print(f"  ✓ Modelo NumPy: {archivo}")


def exportar_codigo_c(modelo, analisis, archivo=None):
    """
    Exporta código C para microcontrolador
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        archivo: Nombre del archivo (None usa config)
    """
    from frequency_analysis import discretizar_modelo, obtener_coeficientes_discretos
    
    if archivo is None:
        archivo = f'{EXPORT_PREFIX}.c'
    
    Ts = analisis['Ts_recomendado']
    sys_discreto = discretizar_modelo(analisis['sys'], Ts, 'tustin')
    
    if sys_discreto is None:
        print("  ❌ Error al discretizar para exportación C")
        return
    
    coefs = obtener_coeficientes_discretos(sys_discreto)
    
    codigo_c = f"""/*
 * Modelo identificado del motor DC
 * Generado automáticamente
 * Fecha: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
 */

// ============================================================================
// FUNCIÓN DE TRANSFERENCIA CONTINUA
// ============================================================================
// G(s) = {modelo['K']:.4f} / ({modelo['tau']:.4f}*s + 1)
//
// Polo: s = {modelo['polo']:.2f} rad/s
// Ancho de banda: {analisis['BW_hz']:.2f} Hz
// Ajuste: {modelo['fit']:.1f}%

// ============================================================================
// MODELO DISCRETIZADO (Método Tustin, Ts = {Ts:.4f} s)
// ============================================================================
// Ecuación en diferencias:
// y[k] = {coefs['a1']:.6f}*y[k-1] + {coefs['b0']:.6f}*u[k] + {coefs['b1']:.6f}*u[k-1]

#ifndef MODELO_MOTOR_H
#define MODELO_MOTOR_H

// Parámetros del modelo continuo
#define K_MOTOR {modelo['K']:.4f}f      // Ganancia [RPM/V]
#define TAU_MOTOR {modelo['tau']:.4f}f  // Constante de tiempo [s]
#define TS_SAMPLE {Ts:.4f}f             // Período de muestreo [s]
#define FS_SAMPLE {1/Ts:.2f}f           // Frecuencia de muestreo [Hz]

// Coeficientes de la ecuación en diferencias
#define A1 {coefs['a1']:.6f}f
#define B0 {coefs['b0']:.6f}f
#define B1 {coefs['b1']:.6f}f

// Variables de estado (declarar como static en tu código)
typedef struct {{
    float y_prev;  // y[k-1]
    float u_prev;  // u[k-1]
}} ModeloMotor_t;

// ============================================================================
// FUNCIONES
// ============================================================================

/**
 * Inicializa el modelo del motor
 * @param modelo: Puntero a la estructura del modelo
 */
void modelo_motor_init(ModeloMotor_t *modelo) {{
    modelo->y_prev = 0.0f;
    modelo->u_prev = 0.0f;
}}

/**
 * Actualiza el modelo del motor (llamar cada Ts)
 * @param modelo: Puntero a la estructura del modelo
 * @param u: Entrada de control [V]
 * @return: Velocidad estimada [RPM]
 */
float modelo_motor_update(ModeloMotor_t *modelo, float u) {{
    // Ecuación en diferencias
    float y = A1 * modelo->y_prev + B0 * u + B1 * modelo->u_prev;
    
    // Actualizar estados
    modelo->y_prev = y;
    modelo->u_prev = u;
    
    return y;
}}

/**
 * Resetea el modelo del motor
 * @param modelo: Puntero a la estructura del modelo
 */
void modelo_motor_reset(ModeloMotor_t *modelo) {{
    modelo->y_prev = 0.0f;
    modelo->u_prev = 0.0f;
}}

#endif // MODELO_MOTOR_H

// ============================================================================
// EJEMPLO DE USO
// ============================================================================
/*
#include "modelo_motor.h"

// Variable global del modelo
static ModeloMotor_t motor_modelo;

void setup() {{
    // Inicializar modelo
    modelo_motor_init(&motor_modelo);
    
    // Configurar timer para muestreo a {1/Ts:.2f} Hz
    // ...
}}

void loop() {{
    // Esta función debe llamarse cada {Ts*1000:.2f} ms
    
    // Leer entrada de control (PWM convertido a voltaje)
    float voltaje = leer_voltaje_pwm();
    
    // Actualizar modelo
    float velocidad_estimada = modelo_motor_update(&motor_modelo, voltaje);
    
    // Usar velocidad estimada para control, logging, etc.
    // ...
}}
*/
"""
    
    with open(archivo, 'w', encoding='utf-8') as f:
        f.write(codigo_c)
    
    print(f"  ✓ Código C: {archivo}")


def exportar_codigo_python(modelo, analisis, archivo=None):
    """
    Exporta clase Python para simulación
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        archivo: Nombre del archivo (None usa config)
    """
    from frequency_analysis import discretizar_modelo, obtener_coeficientes_discretos
    
    if archivo is None:
        archivo = f'{EXPORT_PREFIX}.py'
    
    Ts = analisis['Ts_recomendado']
    sys_discreto = discretizar_modelo(analisis['sys'], Ts, 'tustin')
    
    if sys_discreto is None:
        print("  ❌ Error al discretizar para exportación Python")
        return
    
    coefs = obtener_coeficientes_discretos(sys_discreto)
    
    codigo_python = f'''"""
Modelo identificado del motor DC
Generado automáticamente
Fecha: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
"""

import numpy as np


class ModeloMotor:
    """
    Modelo de primer orden del motor DC
    
    Función de transferencia continua:
    G(s) = {modelo['K']:.4f} / ({modelo['tau']:.4f}*s + 1)
    
    Polo: s = {modelo['polo']:.2f} rad/s
    Ancho de banda: {analisis['BW_hz']:.2f} Hz
    Ajuste: {modelo['fit']:.1f}%
    """
    
    # Parámetros del modelo continuo
    K = {modelo['K']:.4f}        # Ganancia [RPM/V]
    tau = {modelo['tau']:.4f}    # Constante de tiempo [s]
    Ts = {Ts:.4f}                # Período de muestreo [s]
    fs = {1/Ts:.2f}              # Frecuencia de muestreo [Hz]
    
    # Coeficientes discretos (Tustin)
    a1 = {coefs['a1']:.6f}
    b0 = {coefs['b0']:.6f}
    b1 = {coefs['b1']:.6f}
    
    def __init__(self):
        """Inicializa el modelo"""
        self.y_prev = 0.0
        self.u_prev = 0.0
        self.reset()
    
    def reset(self):
        """Resetea el estado del modelo"""
        self.y_prev = 0.0
        self.u_prev = 0.0
    
    def update(self, u):
        """
        Actualiza el modelo con nueva entrada
        
        Args:
            u: Entrada de control [V]
            
        Returns:
            Velocidad estimada [RPM]
        """
        # Ecuación en diferencias
        y = self.a1 * self.y_prev + self.b0 * u + self.b1 * self.u_prev
        
        # Actualizar estados
        self.y_prev = y
        self.u_prev = u
        
        return y
    
    def simular(self, u_array, y0=0):
        """
        Simula respuesta a array de entradas
        
        Args:
            u_array: Array de entradas [V]
            y0: Condición inicial [RPM]
            
        Returns:
            Array de velocidades [RPM]
        """
        self.reset()
        self.y_prev = y0
        
        y_array = np.zeros_like(u_array)
        
        for i, u in enumerate(u_array):
            y_array[i] = self.update(u)
        
        return y_array
    
    def respuesta_escalon(self, amplitud=5.0, duracion=3.0):
        """
        Genera respuesta a escalón
        
        Args:
            amplitud: Amplitud del escalón [V]
            duracion: Duración de la simulación [s]
            
        Returns:
            tuple: (tiempo, velocidad)
        """
        n_puntos = int(duracion / self.Ts)
        tiempo = np.arange(n_puntos) * self.Ts
        u_escalon = np.ones(n_puntos) * amplitud
        
        velocidad = self.simular(u_escalon)
        
        return tiempo, velocidad
    
    def __repr__(self):
        return f"ModeloMotor(K={{self.K}}, tau={{self.tau}}, Ts={{self.Ts}})"


# Ejemplo de uso
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Crear modelo
    motor = ModeloMotor()
    
    # Simular escalón de 5V
    t, v = motor.respuesta_escalon(amplitud=5.0, duracion=3.0)
    
    # Graficar
    plt.figure(figsize=(10, 6))
    plt.plot(t, v, 'b-', linewidth=2)
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Velocidad (RPM)')
    plt.title('Respuesta al Escalón del Modelo')
    plt.grid(True, alpha=0.3)
    plt.show()
    
    print(f"Modelo: {{motor}}")
    print(f"Velocidad final: {{v[-1]:.2f}} RPM")
'''
    
    with open(archivo, 'w', encoding='utf-8') as f:
        f.write(codigo_python)
    
    print(f"  ✓ Código Python: {archivo}")


def exportar_arduino(modelo, analisis, archivo=None):
    """
    Exporta código Arduino/PlatformIO
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        archivo: Nombre del archivo (None usa config)
    """
    from frequency_analysis import discretizar_modelo, obtener_coeficientes_discretos
    
    if archivo is None:
        archivo = f'{EXPORT_PREFIX}.ino'
    
    Ts = analisis['Ts_recomendado']
    Ts_ms = Ts * 1000
    sys_discreto = discretizar_modelo(analisis['sys'], Ts, 'tustin')
    
    if sys_discreto is None:
        print("  ❌ Error al discretizar para exportación Arduino")
        return
    
    coefs = obtener_coeficientes_discretos(sys_discreto)
    
    codigo_arduino = f"""/*
 * Modelo del Motor DC - Arduino
 * Generado automáticamente
 * 
 * Modelo: G(s) = {modelo['K']:.4f} / ({modelo['tau']:.4f}*s + 1)
 * Frecuencia de muestreo: {1/Ts:.2f} Hz ({Ts_ms:.2f} ms)
 */

// ============================================================================
// PARÁMETROS DEL MODELO
// ============================================================================
const float K_MOTOR = {modelo['K']:.4f};      // Ganancia [RPM/V]
const float TAU_MOTOR = {modelo['tau']:.4f};  // Constante de tiempo [s]
const float TS_SAMPLE = {Ts:.4f};             // Período de muestreo [s]
const unsigned long TS_MS = {Ts_ms:.0f};      // Período de muestreo [ms]

// Coeficientes discretos
const float A1 = {coefs['a1']:.6f};
const float B0 = {coefs['b0']:.6f};
const float B1 = {coefs['b1']:.6f};

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
float y_prev = 0.0;  // Velocidad anterior [RPM]
float u_prev = 0.0;  // Entrada anterior [V]

unsigned long tiempo_anterior = 0;

// ============================================================================
// FUNCIONES
// ============================================================================

void modelo_init() {{
    y_prev = 0.0;
    u_prev = 0.0;
}}

float modelo_update(float u) {{
    // Ecuación en diferencias
    float y = A1 * y_prev + B0 * u + B1 * u_prev;
    
    // Actualizar estados
    y_prev = y;
    u_prev = u;
    
    return y;
}}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {{
    Serial.begin(115200);
    
    // Inicializar modelo
    modelo_init();
    
    Serial.println("Modelo del Motor DC inicializado");
    Serial.print("Frecuencia de muestreo: ");
    Serial.print(1000.0 / TS_MS);
    Serial.println(" Hz");
    
    tiempo_anterior = millis();
}}

void loop() {{
    unsigned long tiempo_actual = millis();
    
    // Ejecutar cada TS_MS milisegundos
    if (tiempo_actual - tiempo_anterior >= TS_MS) {{
        tiempo_anterior = tiempo_actual;
        
        // Leer entrada (ejemplo: PWM a voltaje)
        float pwm = analogRead(A0);
        float voltaje = (pwm / 1023.0) * 5.0;  // Convertir a voltios
        
        // Actualizar modelo
        float velocidad_estimada = modelo_update(voltaje);
        
        // Enviar datos por serial
        Serial.print(tiempo_actual);
        Serial.print(",");
        Serial.print(voltaje, 4);
        Serial.print(",");
        Serial.println(velocidad_estimada, 2);
    }}
}}
"""
    
    with open(archivo, 'w', encoding='utf-8') as f:
        f.write(codigo_arduino)
    
    print(f"  ✓ Código Arduino: {archivo}")


def exportar_matlab(modelo, analisis, archivo=None):
    """
    Exporta script MATLAB
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        archivo: Nombre del archivo (None usa config)
    """
    if archivo is None:
        archivo = f'{EXPORT_PREFIX}.m'
    
    Ts = analisis['Ts_recomendado']
    
    codigo_matlab = f"""% Modelo identificado del motor DC
% Generado automáticamente

%% Parámetros del modelo continuo
K = {modelo['K']:.4f};      % Ganancia [RPM/V]
tau = {modelo['tau']:.4f};  % Constante de tiempo [s]
Ts = {Ts:.4f};              % Período de muestreo [s]

%% Crear función de transferencia continua
num_cont = K;
den_cont = [tau 1];
sys_cont = tf(num_cont, den_cont);

disp('Modelo continuo:');
disp(sys_cont);

%% Discretizar usando Tustin (bilinear)
sys_disc = c2d(sys_cont, Ts, 'tustin');

disp('Modelo discreto:');
disp(sys_disc);

%% Visualizar respuesta al escalón
figure('Name', 'Respuesta al Escalón');
step(sys_cont, sys_disc);
legend('Continuo', 'Discreto (Tustin)');
grid on;

%% Diagrama de Bode
figure('Name', 'Diagrama de Bode');
bode(sys_cont);
grid on;

%% Información adicional
info.K = K;
info.tau = tau;
info.Ts = Ts;
info.polo = -1/tau;
info.BW_hz = {analisis['BW_hz']:.4f};
info.ajuste = {modelo['fit']:.2f};

disp(' ');
disp('Información del modelo:');
disp(info);
"""
    
    with open(archivo, 'w', encoding='utf-8') as f:
        f.write(codigo_matlab)
    
    print(f"  ✓ Script MATLAB: {archivo}")


def exportar_modelo_completo(modelo, analisis, prefijo=None):
    """
    Exporta el modelo en todos los formatos configurados
    
    Args:
        modelo: Diccionario con parámetros del modelo
        analisis: Diccionario con análisis de frecuencias
        prefijo: Prefijo para archivos (None usa config)
    """
    print("\n[Exportando modelo...]")
    
    if prefijo is None:
        prefijo = EXPORT_PREFIX
    
    # NumPy
    if EXPORT_NPZ:
        exportar_numpy(modelo, analisis, f'{prefijo}.npz')
    
    # Código C
    if EXPORT_C:
        exportar_codigo_c(modelo, analisis, f'{prefijo}.c')
    
    # Código Python
    if EXPORT_PYTHON:
        exportar_codigo_python(modelo, analisis, f'{prefijo}.py')
    
    # Formatos adicionales
    exportar_arduino(modelo, analisis, f'{prefijo}.ino')
    exportar_matlab(modelo, analisis, f'{prefijo}.m')
    
    print("✓ Exportación completa")
