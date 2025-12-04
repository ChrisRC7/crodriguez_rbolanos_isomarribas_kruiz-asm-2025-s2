"""
============================================================================
MÓDULO DE IDENTIFICACIÓN DE MODELOS
Métodos para identificar modelos de primer orden
============================================================================
"""

import numpy as np
from scipy.optimize import curve_fit
from config import *


def modelo_primer_orden(t, K, tau, t0=0):
    """
    Modelo teórico de primer orden
    G(s) = K/(τs + 1)
    
    Args:
        t: Vector de tiempo
        K: Ganancia del sistema
        tau: Constante de tiempo
        t0: Tiempo de inicio (delay)
        
    Returns:
        Respuesta del sistema
    """
    t_shifted = t - t0
    t_shifted = np.maximum(t_shifted, 0)
    return K * (1 - np.exp(-t_shifted/tau))


def identificar_por_ajuste(tiempo, velocidad, voltaje_escalon):
    """
    Identificación por ajuste de curva (curve fitting)
    Equivalente a tfest() de MATLAB
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades medidas
        voltaje_escalon: Voltaje aplicado en el escalón
        
    Returns:
        dict: Diccionario con parámetros identificados y métricas
    """
    # Valores iniciales inteligentes
    vel_final = np.mean(velocidad[-20:])
    K_init = vel_final
    tau_init = 0.3
    
    try:
        # Ajuste de curva con bounds
        popt, pcov = curve_fit(
            modelo_primer_orden, 
            tiempo, 
            velocidad,
            p0=[K_init, tau_init],
            bounds=([AJUSTE_K_MIN, AJUSTE_TAU_MIN], 
                   [vel_final*AJUSTE_K_MAX_FACTOR, AJUSTE_TAU_MAX]),
            maxfev=AJUSTE_MAX_ITERATIONS
        )
        
        K_id, tau_id = popt
        
        # Calcular modelo ajustado
        y_ajustado = modelo_primer_orden(tiempo, K_id, tau_id)
        
        # Calcular métricas de ajuste
        residuos = velocidad - y_ajustado
        ss_res = np.sum(residuos**2)
        ss_tot = np.sum((velocidad - np.mean(velocidad))**2)
        r2 = 1 - (ss_res / ss_tot)
        fit_percent = r2 * 100  # Similar al "fit" de MATLAB
        rmse = np.sqrt(np.mean(residuos**2))
        
        # Ganancia normalizada por voltaje
        K_normalizado = K_id / voltaje_escalon
        
        # Calcular polo
        polo = -1/tau_id
        
        return {
            'metodo': 'Ajuste de curva',
            'K': K_normalizado,
            'tau': tau_id,
            'polo': polo,
            'fit': fit_percent,
            'rmse': rmse,
            'y_modelo': y_ajustado,
            'K_total': K_id,
            'exito': True
        }
        
    except Exception as e:
        print(f"❌ Error en ajuste: {e}")
        return {'exito': False, 'error': str(e)}


def identificar_por_63(tiempo, velocidad, voltaje_escalon, velocidad_suave=None):
    """
    Identificación por método del 63.2%
    Método clásico de identificación de sistemas de primer orden
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades medidas
        voltaje_escalon: Voltaje aplicado
        velocidad_suave: Velocidad suavizada (opcional)
        
    Returns:
        dict: Diccionario con parámetros identificados
    """
    from data_loader import suavizar_datos
    
    # Suavizar datos si no se proporciona
    if velocidad_suave is None:
        velocidad_suave = suavizar_datos(velocidad)
    
    # Valor final (promedio últimos valores)
    vel_final = np.mean(velocidad_suave[-20:])
    
    # Ganancia
    K = vel_final / voltaje_escalon
    
    # Constante de tiempo (63.2% del valor final)
    vel_63 = 0.632 * vel_final
    idx_63 = np.where(velocidad_suave >= vel_63)[0]
    
    if len(idx_63) > 0:
        tau = tiempo[idx_63[0]]
        polo = -1/tau
        idx_punto = idx_63[0]
        exito = True
    else:
        tau = None
        polo = None
        idx_punto = None
        exito = False
        print("⚠️ No se pudo determinar τ por método 63.2%")
    
    return {
        'metodo': 'Método 63.2%',
        'K': K,
        'tau': tau,
        'polo': polo,
        'vel_final': vel_final,
        'vel_63': vel_63,
        'idx_63': idx_punto,
        'velocidad_suave': velocidad_suave,
        'exito': exito
    }


def identificar_por_tangente(tiempo, velocidad, voltaje_escalon):
    """
    Identificación por método de la tangente
    Traza tangente en el punto de inflexión
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades
        voltaje_escalon: Voltaje aplicado
        
    Returns:
        dict: Parámetros identificados
    """
    from data_loader import suavizar_datos
    
    velocidad_suave = suavizar_datos(velocidad)
    
    # Calcular derivada
    derivada = np.gradient(velocidad_suave, tiempo)
    
    # Punto de inflexión (máxima derivada)
    idx_inflex = np.argmax(derivada)
    t_inflex = tiempo[idx_inflex]
    v_inflex = velocidad_suave[idx_inflex]
    pendiente_max = derivada[idx_inflex]
    
    # Valor final
    vel_final = np.mean(velocidad_suave[-20:])
    K = vel_final / voltaje_escalon
    
    # Tau desde geometría de la tangente
    # En t=0, la tangente intersecta
    tau = vel_final / pendiente_max
    
    if tau > 0:
        polo = -1/tau
        exito = True
    else:
        polo = None
        exito = False
    
    return {
        'metodo': 'Método de la tangente',
        'K': K,
        'tau': tau,
        'polo': polo,
        't_inflex': t_inflex,
        'v_inflex': v_inflex,
        'pendiente_max': pendiente_max,
        'vel_final': vel_final,
        'exito': exito
    }


def comparar_metodos(tiempo, velocidad, voltaje_escalon):
    """
    Compara todos los métodos de identificación
    
    Args:
        tiempo: Array de tiempos
        velocidad: Array de velocidades
        voltaje_escalon: Voltaje aplicado
        
    Returns:
        dict: Resultados de todos los métodos
    """
    resultados = {}
    
    # Método de ajuste
    print("  Método 1: Ajuste de curva...")
    resultados['ajuste'] = identificar_por_ajuste(tiempo, velocidad, voltaje_escalon)
    
    # Método 63.2%
    print("  Método 2: 63.2%...")
    resultados['63'] = identificar_por_63(tiempo, velocidad, voltaje_escalon)
    
    # Método tangente
    print("  Método 3: Tangente...")
    resultados['tangente'] = identificar_por_tangente(tiempo, velocidad, voltaje_escalon)
    
    return resultados


def calcular_metricas_modelo(tiempo, velocidad, y_modelo):
    """
    Calcula métricas de calidad del ajuste
    
    Args:
        tiempo: Array de tiempos
        velocidad: Datos medidos
        y_modelo: Datos del modelo
        
    Returns:
        dict: Métricas calculadas
    """
    residuos = velocidad - y_modelo
    
    # RMSE
    rmse = np.sqrt(np.mean(residuos**2))
    
    # R²
    ss_res = np.sum(residuos**2)
    ss_tot = np.sum((velocidad - np.mean(velocidad))**2)
    r2 = 1 - (ss_res / ss_tot)
    
    # MAE
    mae = np.mean(np.abs(residuos))
    
    # Fit percentage (MATLAB-style)
    fit_percent = r2 * 100
    
    return {
        'rmse': rmse,
        'r2': r2,
        'mae': mae,
        'fit': fit_percent,
        'residuos': residuos
    }
