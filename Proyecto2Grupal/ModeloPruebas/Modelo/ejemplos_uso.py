"""
============================================================================
EJEMPLOS DE USO DE LOS MÓDULOS
Demuestra cómo usar cada módulo de forma independiente
============================================================================
"""

# ==============================================================================
# EJEMPLO 1: Carga y validación de datos
# ==============================================================================
print("="*70)
print("EJEMPLO 1: Carga y validación de datos")
print("="*70)

from data_loader import cargar_datos, validar_datos, suavizar_datos

# Cargar datos (automáticamente genera ejemplo si no hay archivo)
tiempo, velocidad, dt = cargar_datos(None)

# Validar datos
validacion = validar_datos(tiempo, velocidad)
print(f"\nDatos válidos: {validacion['valido']}")
if validacion['warnings']:
    print("Advertencias:")
    for warning in validacion['warnings']:
        print(f"  - {warning}")

# Suavizar datos
velocidad_suave = suavizar_datos(velocidad, ventana=5)
print(f"\n✓ Datos suavizados con ventana de 5 muestras")


# ==============================================================================
# EJEMPLO 2: Identificación con un solo método
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 2: Identificación con un solo método")
print("="*70)

from model_identification import identificar_por_ajuste

voltaje = 5.0
modelo = identificar_por_ajuste(tiempo, velocidad, voltaje)

print(f"\nModelo identificado:")
print(f"  K = {modelo['K']:.4f} RPM/V")
print(f"  τ = {modelo['tau']:.4f} s")
print(f"  Polo = {modelo['polo']:.2f} rad/s")
print(f"  Ajuste = {modelo['fit']:.1f}%")
print(f"  RMSE = {modelo['rmse']:.2f} RPM")


# ==============================================================================
# EJEMPLO 3: Comparación de métodos
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 3: Comparación de métodos")
print("="*70)

from model_identification import comparar_metodos

resultados = comparar_metodos(tiempo, velocidad, voltaje)

print("\nResultados de cada método:")
print(f"{'Método':<20} {'K (RPM/V)':<12} {'τ (s)':<10} {'Ajuste (%)':<12}")
print("-"*60)
for nombre, resultado in resultados.items():
    if resultado.get('exito'):
        K = resultado['K']
        tau = resultado['tau']
        fit = resultado.get('fit', '-')
        print(f"{nombre:<20} {K:<12.2f} {tau:<10.4f} {fit}")


# ==============================================================================
# EJEMPLO 4: Análisis de frecuencias
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 4: Análisis de frecuencias")
print("="*70)

from frequency_analysis import analizar_frecuencias, discretizar_modelo

# Usar el modelo identificado
K_total = modelo['K'] * voltaje
tau = modelo['tau']

analisis = analizar_frecuencias(K_total, tau)

print(f"\nAnálisis de frecuencias:")
print(f"  Ancho de banda: {analisis['BW_hz']:.2f} Hz ({analisis['BW_rad']:.2f} rad/s)")
print(f"  Frecuencia Nyquist: {analisis['fs_nyquist']:.2f} Hz")
print(f"  Frecuencia recomendada: {analisis['fs_recomendada']:.2f} Hz")
print(f"  Período recomendado: {analisis['Ts_recomendado']*1000:.2f} ms")

# Discretizar el modelo
sys_discreto = discretizar_modelo(analisis['sys'], analisis['Ts_recomendado'], 'tustin')
print(f"\n✓ Modelo discretizado con método Tustin")
print(f"  Período de muestreo: {analisis['Ts_recomendado']*1000:.2f} ms")


# ==============================================================================
# EJEMPLO 5: Obtener coeficientes para implementación
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 5: Coeficientes para implementación")
print("="*70)

from frequency_analysis import obtener_coeficientes_discretos

coefs = obtener_coeficientes_discretos(sys_discreto)

print("\nEcuación en diferencias:")
print(f"y[k] = {coefs['a1']:.6f}*y[k-1] + {coefs['b0']:.6f}*u[k] + {coefs['b1']:.6f}*u[k-1]")
print("\nCoeficientes individuales:")
print(f"  a1 = {coefs['a1']:.6f}")
print(f"  b0 = {coefs['b0']:.6f}")
print(f"  b1 = {coefs['b1']:.6f}")


# ==============================================================================
# EJEMPLO 6: Visualización simple
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 6: Visualización simple")
print("="*70)

import matplotlib.pyplot as plt
from visualization import plot_respuesta_temporal
from model_identification import identificar_por_63

modelo_63 = identificar_por_63(tiempo, velocidad, voltaje)

fig, ax = plt.subplots(figsize=(10, 6))
plot_respuesta_temporal(tiempo, velocidad, modelo, modelo_63, ax)
plt.savefig('respuesta_temporal.png', dpi=150, bbox_inches='tight')
print("\n✓ Gráfica guardada como 'respuesta_temporal.png'")
plt.close()


# ==============================================================================
# EJEMPLO 7: Exportar solo formato específico
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 7: Exportar formato específico")
print("="*70)

from export_utils import exportar_codigo_python, exportar_arduino

# Exportar solo Python
exportar_codigo_python(modelo, analisis, 'mi_modelo.py')

# Exportar solo Arduino
exportar_arduino(modelo, analisis, 'motor_control.ino')

print("\n✓ Archivos exportados")


# ==============================================================================
# EJEMPLO 8: Simular con el modelo exportado
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 8: Simular con modelo exportado")
print("="*70)

# Importar el modelo que acabamos de exportar
import sys
sys.path.insert(0, '.')
from mi_modelo import ModeloMotor

# Crear instancia
motor = ModeloMotor()

# Simular escalón
t_sim, v_sim = motor.respuesta_escalon(amplitud=voltaje, duracion=3.0)

print(f"\nSimulación completada:")
print(f"  Duración: {t_sim[-1]:.2f} s")
print(f"  Velocidad final: {v_sim[-1]:.2f} RPM")
print(f"  Valor esperado: {modelo['K'] * voltaje:.2f} RPM")


# ==============================================================================
# EJEMPLO 9: Cambiar parámetros de configuración
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 9: Modificar configuración")
print("="*70)

import config

# Guardar valores originales
voltaje_original = config.VOLTAJE_ESCALON
ruido_original = config.EJEMPLO_RUIDO_STD

# Cambiar temporalmente
config.VOLTAJE_ESCALON = 3.0
config.EJEMPLO_RUIDO_STD = 50

print(f"\nConfiguración modificada:")
print(f"  Voltaje: {config.VOLTAJE_ESCALON} V")
print(f"  Ruido: {config.EJEMPLO_RUIDO_STD} RPM")

# Restaurar
config.VOLTAJE_ESCALON = voltaje_original
config.EJEMPLO_RUIDO_STD = ruido_original

print(f"\n✓ Configuración restaurada")


# ==============================================================================
# EJEMPLO 10: Crear datos personalizados
# ==============================================================================
print("\n" + "="*70)
print("EJEMPLO 10: Generar datos personalizados")
print("="*70)

import numpy as np
from data_loader import guardar_datos_csv

# Generar respuesta personalizada
t_custom = np.linspace(0, 5, 250)
K_custom = 800
tau_custom = 0.5
v_custom = K_custom * (1 - np.exp(-t_custom/tau_custom))

# Agregar ruido
v_custom += 15 * np.random.randn(len(t_custom))

# Guardar
guardar_datos_csv(t_custom, v_custom, 'datos_personalizados.csv')

print(f"\n✓ Datos personalizados generados y guardados")
print(f"  Parámetros: K={K_custom}, τ={tau_custom}")


# ==============================================================================
# RESUMEN
# ==============================================================================
print("\n" + "="*70)
print("RESUMEN DE EJEMPLOS")
print("="*70)
print("""
✓ Ejemplo 1: Carga y validación de datos
✓ Ejemplo 2: Identificación básica
✓ Ejemplo 3: Comparación de métodos
✓ Ejemplo 4: Análisis de frecuencias
✓ Ejemplo 5: Coeficientes discretos
✓ Ejemplo 6: Visualización simple
✓ Ejemplo 7: Exportación específica
✓ Ejemplo 8: Simulación con modelo
✓ Ejemplo 9: Modificación de config
✓ Ejemplo 10: Datos personalizados

Todos los ejemplos completados exitosamente!
""")
