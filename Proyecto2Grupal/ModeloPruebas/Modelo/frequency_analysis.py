"""
============================================================================
MÓDULO DE ANÁLISIS DE FRECUENCIAS
Funciones para análisis de Bode, ancho de banda y discretización
============================================================================
"""

import numpy as np
from scipy import signal
from config import *


def crear_funcion_transferencia(K, tau):
    """
    Crea función de transferencia continua de primer orden
    G(s) = K/(τs + 1)
    
    Args:
        K: Ganancia
        tau: Constante de tiempo
        
    Returns:
        TransferFunction de scipy
    """
    num = [K]
    den = [tau, 1]
    sys = signal.TransferFunction(num, den)
    return sys


def calcular_ancho_banda(sys):
    """
    Calcula el ancho de banda (-3dB) del sistema
    
    Args:
        sys: Sistema (TransferFunction)
        
    Returns:
        dict: Información de ancho de banda
    """
    # Rango de frecuencias
    w = np.logspace(np.log10(FREQ_RANGO_MIN), 
                   np.log10(FREQ_RANGO_MAX), 
                   FREQ_NUM_PUNTOS)
    
    # Respuesta en frecuencia
    w_rad, h = signal.freqs(sys.num, sys.den, worN=w)
    mag_db = 20 * np.log10(np.abs(h))
    
    # Encontrar frecuencia de corte (-3dB)
    idx_3db = np.where(mag_db <= (mag_db[0] - 3))[0]
    if len(idx_3db) > 0:
        BW_rad = w_rad[idx_3db[0]]
    else:
        # Aproximación para sistema de primer orden
        BW_rad = 1 / sys.den[0]
    
    BW_hz = BW_rad / (2*np.pi)
    
    return {
        'BW_rad': BW_rad,
        'BW_hz': BW_hz,
        'w': w_rad,
        'mag_db': mag_db,
        'h': h
    }


def calcular_frecuencias_muestreo(BW_hz):
    """
    Calcula frecuencias de muestreo según criterio de Nyquist
    
    Args:
        BW_hz: Ancho de banda en Hz
        
    Returns:
        dict: Frecuencias recomendadas
    """
    fs_nyquist = FACTOR_NYQUIST * BW_hz
    fs_recomendada = FACTOR_RECOMENDADO * BW_hz
    Ts_recomendado = 1 / fs_recomendada
    
    return {
        'fs_nyquist': fs_nyquist,
        'fs_recomendada': fs_recomendada,
        'Ts_recomendado': Ts_recomendado,
        'Ts_nyquist': 1 / fs_nyquist
    }


def analizar_frecuencias(K, tau):
    """
    Análisis completo de frecuencias del sistema
    
    Args:
        K: Ganancia del sistema
        tau: Constante de tiempo
        
    Returns:
        dict: Resultados del análisis completo
    """
    # Crear sistema
    sys = crear_funcion_transferencia(K, tau)
    
    # Calcular ancho de banda
    bw_info = calcular_ancho_banda(sys)
    
    # Calcular frecuencias de muestreo
    fs_info = calcular_frecuencias_muestreo(bw_info['BW_hz'])
    
    # Combinar resultados
    resultado = {
        'sys': sys,
        **bw_info,
        **fs_info
    }
    
    return resultado


def discretizar_modelo(sys_continuo, Ts, metodo=None):
    """
    Discretiza el modelo continuo
    
    Args:
        sys_continuo: Sistema continuo (TransferFunction)
        Ts: Período de muestreo
        metodo: 'tustin', 'zoh', 'euler' (None usa config)
        
    Returns:
        Sistema discreto (TransferFunction)
    """
    if metodo is None:
        metodo = METODO_DISCRETIZACION
    
    # Mapeo de nombres de métodos
    metodos_validos = {
        'tustin': 'bilinear',
        'bilinear': 'bilinear',
        'zoh': 'zoh',
        'euler': 'euler',
        'backward': 'backward_diff',
        'forward': 'forward_diff'
    }
    
    metodo_scipy = metodos_validos.get(metodo, 'bilinear')
    
    try:
        sys_discreto = sys_continuo.to_discrete(Ts, method=metodo_scipy)
        return sys_discreto
    except Exception as e:
        print(f"❌ Error en discretización: {e}")
        return None


def obtener_coeficientes_discretos(sys_discreto):
    """
    Extrae coeficientes de la ecuación en diferencias
    y[k] = -a1*y[k-1] + b0*u[k] + b1*u[k-1]
    
    Args:
        sys_discreto: Sistema discreto
        
    Returns:
        dict: Coeficientes normalizados
    """
    num, den = sys_discreto.num, sys_discreto.den
    
    # Extraer coeficientes correctamente
    if num.ndim > 1:
        num = num[0]
    if den.ndim > 1:
        den = den[0]
    
    # Normalizar por den[0]
    num_norm = num / den[0]
    den_norm = den / den[0]
    
    return {
        'num': num_norm,
        'den': den_norm,
        'a1': -den_norm[1] if len(den_norm) > 1 else 0,
        'b0': num_norm[0],
        'b1': num_norm[1] if len(num_norm) > 1 else 0
    }


def analisis_polos_ceros(sys):
    """
    Analiza polos y ceros del sistema
    
    Args:
        sys: Sistema (TransferFunction)
        
    Returns:
        dict: Información de polos y ceros
    """
    polos = sys.poles
    ceros = sys.zeros
    
    # Clasificar polos
    polos_reales = polos[np.isreal(polos)]
    polos_complejos = polos[~np.isreal(polos)]
    
    # Estabilidad
    estable = np.all(np.real(polos) < 0)
    
    return {
        'polos': polos,
        'ceros': ceros,
        'polos_reales': polos_reales,
        'polos_complejos': polos_complejos,
        'estable': estable,
        'num_polos': len(polos),
        'num_ceros': len(ceros)
    }


def respuesta_temporal(sys, t_final=None, num_puntos=300):
    """
    Calcula respuesta al escalón del sistema
    
    Args:
        sys: Sistema (TransferFunction)
        t_final: Tiempo final de simulación
        num_puntos: Número de puntos
        
    Returns:
        tuple: (tiempo, respuesta)
    """
    if t_final is None:
        # Automático: 5 constantes de tiempo
        if hasattr(sys, 'den') and len(sys.den) > 0:
            tau = sys.den[0]
            t_final = 5 * tau
        else:
            t_final = 5.0
    
    t, y = signal.step(sys, N=num_puntos, T=np.linspace(0, t_final, num_puntos))
    
    return t, y


def respuesta_frecuencia(sys, w=None):
    """
    Calcula respuesta en frecuencia (Bode)
    
    Args:
        sys: Sistema
        w: Frecuencias (None para automático)
        
    Returns:
        dict: Magnitud, fase y frecuencias
    """
    if w is None:
        w = np.logspace(FREQ_RANGO_MIN, FREQ_RANGO_MAX, FREQ_NUM_PUNTOS)
    
    w_rad, h = signal.freqs(sys.num, sys.den, worN=w)
    mag = np.abs(h)
    mag_db = 20 * np.log10(mag)
    fase_deg = np.angle(h) * 180 / np.pi
    
    return {
        'w': w_rad,
        'mag': mag,
        'mag_db': mag_db,
        'fase': fase_deg
    }


def comparar_continuo_discreto(sys_continuo, Ts):
    """
    Compara respuesta de sistema continuo vs discretizado
    
    Args:
        sys_continuo: Sistema continuo
        Ts: Período de muestreo
        
    Returns:
        dict: Datos de comparación
    """
    # Discretizar
    sys_discreto = discretizar_modelo(sys_continuo, Ts)
    
    if sys_discreto is None:
        return None
    
    # Respuestas temporales
    t_cont, y_cont = respuesta_temporal(sys_continuo)
    t_disc, y_disc = signal.dstep(sys_discreto, n=len(t_cont))
    
    return {
        'sys_discreto': sys_discreto,
        't_continuo': t_cont,
        'y_continuo': y_cont,
        't_discreto': t_disc,
        'y_discreto': y_disc[0],
        'coefs': obtener_coeficientes_discretos(sys_discreto)
    }
