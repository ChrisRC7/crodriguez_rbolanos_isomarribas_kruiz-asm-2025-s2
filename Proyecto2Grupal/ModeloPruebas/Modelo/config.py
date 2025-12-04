"""
============================================================================
CONFIGURACIÓN DEL SISTEMA DE IDENTIFICACIÓN
============================================================================
"""

# Parámetros del experimento
VOLTAJE_ESCALON = 5.0  # Voltios aplicados en el escalón
ARCHIVO_DATOS = None   # Cambiar a 'datos_experimento.csv' para datos reales

# Parámetros de datos de ejemplo (cuando no hay datos reales)
EJEMPLO_K_REAL = 1000      # Ganancia RPM/V
EJEMPLO_TAU_REAL = 0.3     # Constante de tiempo (s)
EJEMPLO_DT = 0.02          # Período de muestreo (s) - 50 Hz
EJEMPLO_T_FINAL = 3        # Duración del experimento (s)
EJEMPLO_RUIDO_STD = 20     # Desviación estándar del ruido (RPM)

# Parámetros de ajuste de curva
AJUSTE_K_MIN = 0           # Límite inferior para K
AJUSTE_K_MAX_FACTOR = 2    # Factor multiplicador del valor final para K_max
AJUSTE_TAU_MIN = 0.01      # Límite inferior para tau (s)
AJUSTE_TAU_MAX = 5.0       # Límite superior para tau (s)
AJUSTE_MAX_ITERATIONS = 5000  # Iteraciones máximas para curve_fit

# Parámetros de suavizado
SUAVIZADO_VENTANA = 5      # Tamaño de ventana para suavizado de datos

# Parámetros de frecuencia
FREQ_RANGO_MIN = 1e-2      # Frecuencia mínima para análisis (rad/s)
FREQ_RANGO_MAX = 1e3       # Frecuencia máxima para análisis (rad/s)
FREQ_NUM_PUNTOS = 1000     # Número de puntos en análisis de frecuencia

# Parámetros de discretización
METODO_DISCRETIZACION = 'tustin'  # Opciones: 'tustin', 'zoh', 'euler'
FACTOR_NYQUIST = 2         # Factor sobre ancho de banda para Nyquist
FACTOR_RECOMENDADO = 10    # Factor sobre ancho de banda para fs recomendada

# Parámetros de visualización
PLOT_STYLE = 'seaborn-v0_8-darkgrid'
PLOT_FIGSIZE_MAIN = (16, 10)
PLOT_FIGSIZE_DISCRETE = (14, 5)
PLOT_FONTSIZE = 10
PLOT_DPI = 100

# Parámetros de exportación
EXPORT_PREFIX = 'modelo_identificado'  # Prefijo para archivos exportados
EXPORT_NPZ = True          # Exportar en formato NumPy
EXPORT_C = True            # Exportar código C
EXPORT_PYTHON = True       # Exportar código Python
