"""
============================================================================
MÓDULO DE CARGA DE DATOS
Funciones para cargar datos experimentales o generar datos de ejemplo
============================================================================
"""

import numpy as np
import pandas as pd
from config import *


def cargar_datos_csv(archivo):
    """
    Carga datos desde archivo CSV
    
    Formato CSV esperado:
    tiempo,velocidad
    0.000,0
    0.020,15
    0.040,45
    ...
    
    Args:
        archivo: Ruta al archivo CSV
        
    Returns:
        tuple: (tiempo, velocidad, dt)
    """
    print(f"Cargando datos desde: {archivo}")
    datos = pd.read_csv(archivo)
    tiempo = datos.iloc[:, 0].values  # Primera columna: tiempo
    velocidad = datos.iloc[:, 1].values  # Segunda columna: velocidad
    
    # Calcular período de muestreo
    dt = np.mean(np.diff(tiempo))
    fs = 1/dt
    
    print(f"✓ Datos cargados: {len(tiempo)} muestras")
    print(f"  Período de muestreo: {dt:.4f} s ({fs:.1f} Hz)")
    print(f"  Duración total: {tiempo[-1]:.2f} s")
    
    return tiempo, velocidad, dt


def generar_datos_ejemplo():
    """
    Genera datos de ejemplo para pruebas
    Simula respuesta de sistema de primer orden con ruido
    
    Returns:
        tuple: (tiempo, velocidad, dt)
    """
    print("⚠️  Usando datos de ejemplo - Reemplazar con datos reales")
    
    # Generar tiempo
    tiempo = np.arange(0, EJEMPLO_T_FINAL, EJEMPLO_DT)
    
    # Respuesta teórica de primer orden
    velocidad_teorica = (EJEMPLO_K_REAL * VOLTAJE_ESCALON * 
                        (1 - np.exp(-tiempo/EJEMPLO_TAU_REAL)))
    
    # Añadir ruido gaussiano
    ruido = EJEMPLO_RUIDO_STD * np.random.randn(len(tiempo))
    velocidad = velocidad_teorica + ruido
    velocidad = np.maximum(velocidad, 0)  # No negativas
    
    # Calcular período de muestreo
    dt = EJEMPLO_DT
    fs = 1/dt
    
    print(f"✓ Datos generados: {len(tiempo)} muestras")
    print(f"  Período de muestreo: {dt:.4f} s ({fs:.1f} Hz)")
    print(f"  Duración total: {tiempo[-1]:.2f} s")
    print(f"  Parámetros teóricos: K={EJEMPLO_K_REAL}, τ={EJEMPLO_TAU_REAL}s")
    
    return tiempo, velocidad, dt


def cargar_datos(archivo=None):
    """
    Función principal de carga de datos
    Decide si cargar desde archivo o generar ejemplo
    
    Args:
        archivo: Ruta al archivo CSV (None para datos de ejemplo)
        
    Returns:
        tuple: (tiempo, velocidad, dt)
    """
    if archivo is not None:
        return cargar_datos_csv(archivo)
    else:
        return generar_datos_ejemplo()


def guardar_datos_csv(tiempo, velocidad, archivo='datos_experimento.csv'):
    """
    Guarda datos en formato CSV
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades
        archivo: Nombre del archivo de salida
    """
    df = pd.DataFrame({
        'tiempo': tiempo,
        'velocidad': velocidad
    })
    df.to_csv(archivo, index=False)
    print(f"✓ Datos guardados en: {archivo}")


def suavizar_datos(velocidad, ventana=None):
    """
    Suaviza datos usando media móvil
    
    Args:
        velocidad: Array de velocidades
        ventana: Tamaño de ventana (None usa config.SUAVIZADO_VENTANA)
        
    Returns:
        Array de velocidades suavizadas
    """
    if ventana is None:
        ventana = SUAVIZADO_VENTANA
    
    velocidad_suave = np.convolve(velocidad, np.ones(ventana)/ventana, mode='same')
    return velocidad_suave


def validar_datos(tiempo, velocidad):
    """
    Valida que los datos sean correctos
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades
        
    Returns:
        dict: Resultado de validación con warnings si los hay
    """
    warnings = []
    
    # Verificar longitud mínima
    if len(tiempo) < 50:
        warnings.append("⚠️ Muy pocas muestras (< 50)")
    
    # Verificar orden temporal
    if not np.all(np.diff(tiempo) > 0):
        warnings.append("❌ Tiempo no es monótono creciente")
    
    # Verificar valores negativos
    if np.any(velocidad < 0):
        warnings.append("⚠️ Hay valores negativos de velocidad")
    
    # Verificar NaN o Inf
    if np.any(np.isnan(velocidad)) or np.any(np.isinf(velocidad)):
        warnings.append("❌ Hay valores NaN o Inf")
    
    # Verificar período de muestreo constante
    dt_array = np.diff(tiempo)
    dt_std = np.std(dt_array)
    if dt_std > 0.01 * np.mean(dt_array):
        warnings.append("⚠️ Período de muestreo no es constante")
    
    return {
        'valido': len([w for w in warnings if '❌' in w]) == 0,
        'warnings': warnings
    }
