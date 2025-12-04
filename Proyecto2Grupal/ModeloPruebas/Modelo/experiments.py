"""
============================================================================
SCRIPT DE EXPERIMENTOS
Facilita realizar pruebas y comparaciones de diferentes configuraciones
============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt

from config import *
from data_loader import cargar_datos, generar_datos_ejemplo, suavizar_datos
from model_identification import (identificar_por_ajuste, identificar_por_63,
                                 identificar_por_tangente, comparar_metodos)
from frequency_analysis import analizar_frecuencias, discretizar_modelo
from visualization import plot_comparacion_metodos


def experimento_comparar_metodos():
    """
    Compara los diferentes métodos de identificación
    """
    print("="*70)
    print(" EXPERIMENTO: COMPARACIÓN DE MÉTODOS")
    print("="*70)
    
    # Cargar datos
    tiempo, velocidad, dt = cargar_datos(None)
    
    # Comparar métodos
    print("\nComparando métodos de identificación...")
    resultados = comparar_metodos(tiempo, velocidad, VOLTAJE_ESCALON)
    
    # Mostrar resultados
    print("\n" + "="*70)
    print(" RESULTADOS")
    print("="*70)
    
    for nombre, resultado in resultados.items():
        print(f"\n{nombre}:")
        if resultado.get('exito'):
            print(f"  K = {resultado['K']:.4f} RPM/V")
            print(f"  τ = {resultado['tau']:.4f} s")
            if 'fit' in resultado:
                print(f"  Ajuste = {resultado['fit']:.2f}%")
        else:
            print("  ❌ Falló")
    
    # Graficar comparación
    plot_comparacion_metodos(resultados, tiempo, velocidad)
    plt.show()


def experimento_variacion_parametros():
    """
    Estudia cómo varían los parámetros con diferentes configuraciones
    """
    print("="*70)
    print(" EXPERIMENTO: VARIACIÓN DE PARÁMETROS")
    print("="*70)
    
    # Parámetros a probar
    K_vals = [500, 1000, 1500, 2000]
    tau_vals = [0.1, 0.3, 0.5, 1.0]
    
    resultados = []
    
    for K_real in K_vals:
        for tau_real in tau_vals:
            # Generar datos con parámetros específicos
            print(f"\nProbando K={K_real}, τ={tau_real}...")
            
            # Modificar config temporalmente
            import config
            config.EJEMPLO_K_REAL = K_real
            config.EJEMPLO_TAU_REAL = tau_real
            
            tiempo, velocidad, dt = generar_datos_ejemplo()
            
            # Identificar
            modelo = identificar_por_ajuste(tiempo, velocidad, VOLTAJE_ESCALON)
            
            if modelo.get('exito'):
                error_K = abs(modelo['K']*VOLTAJE_ESCALON - K_real) / K_real * 100
                error_tau = abs(modelo['tau'] - tau_real) / tau_real * 100
                
                resultados.append({
                    'K_real': K_real,
                    'tau_real': tau_real,
                    'K_id': modelo['K']*VOLTAJE_ESCALON,
                    'tau_id': modelo['tau'],
                    'error_K': error_K,
                    'error_tau': error_tau,
                    'fit': modelo['fit']
                })
    
    # Mostrar resultados
    print("\n" + "="*70)
    print(" RESUMEN DE ERRORES")
    print("="*70)
    print(f"{'K_real':>8} {'τ_real':>8} {'K_id':>8} {'τ_id':>8} {'Error_K%':>10} {'Error_τ%':>10} {'Fit%':>8}")
    print("-"*70)
    
    for r in resultados:
        print(f"{r['K_real']:8.0f} {r['tau_real']:8.2f} {r['K_id']:8.0f} "
              f"{r['tau_id']:8.4f} {r['error_K']:10.2f} {r['error_tau']:10.2f} {r['fit']:8.1f}")
    
    # Estadísticas
    errores_K = [r['error_K'] for r in resultados]
    errores_tau = [r['error_tau'] for r in resultados]
    
    print(f"\nError promedio K: {np.mean(errores_K):.2f}%")
    print(f"Error promedio τ: {np.mean(errores_tau):.2f}%")


def experimento_efecto_ruido():
    """
    Estudia el efecto del ruido en la identificación
    """
    print("="*70)
    print(" EXPERIMENTO: EFECTO DEL RUIDO")
    print("="*70)
    
    # Niveles de ruido a probar
    niveles_ruido = [0, 10, 20, 50, 100]
    
    resultados = []
    
    for ruido_std in niveles_ruido:
        print(f"\nNivel de ruido: {ruido_std} RPM...")
        
        # Modificar config temporalmente
        import config
        config.EJEMPLO_RUIDO_STD = ruido_std
        
        tiempo, velocidad, dt = generar_datos_ejemplo()
        
        modelo = identificar_por_ajuste(tiempo, velocidad, VOLTAJE_ESCALON)
        
        if modelo.get('exito'):
            resultados.append({
                'ruido': ruido_std,
                'K': modelo['K'],
                'tau': modelo['tau'],
                'fit': modelo['fit'],
                'rmse': modelo['rmse']
            })
    
    # Graficar resultados
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('EFECTO DEL RUIDO EN LA IDENTIFICACIÓN', fontsize=14, fontweight='bold')
    
    ruidos = [r['ruido'] for r in resultados]
    Ks = [r['K'] for r in resultados]
    taus = [r['tau'] for r in resultados]
    fits = [r['fit'] for r in resultados]
    rmses = [r['rmse'] for r in resultados]
    
    # K vs ruido
    axes[0, 0].plot(ruidos, Ks, 'bo-', linewidth=2, markersize=8)
    axes[0, 0].axhline(y=EJEMPLO_K_REAL/VOLTAJE_ESCALON, color='r', linestyle='--', label='Valor real')
    axes[0, 0].set_xlabel('Ruido (RPM)')
    axes[0, 0].set_ylabel('K identificada (RPM/V)')
    axes[0, 0].set_title('Ganancia vs Ruido')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Tau vs ruido
    axes[0, 1].plot(ruidos, taus, 'go-', linewidth=2, markersize=8)
    axes[0, 1].axhline(y=EJEMPLO_TAU_REAL, color='r', linestyle='--', label='Valor real')
    axes[0, 1].set_xlabel('Ruido (RPM)')
    axes[0, 1].set_ylabel('τ identificada (s)')
    axes[0, 1].set_title('Constante de Tiempo vs Ruido')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Fit vs ruido
    axes[1, 0].plot(ruidos, fits, 'mo-', linewidth=2, markersize=8)
    axes[1, 0].set_xlabel('Ruido (RPM)')
    axes[1, 0].set_ylabel('Ajuste (%)')
    axes[1, 0].set_title('Calidad de Ajuste vs Ruido')
    axes[1, 0].grid(True, alpha=0.3)
    
    # RMSE vs ruido
    axes[1, 1].plot(ruidos, rmses, 'ro-', linewidth=2, markersize=8)
    axes[1, 1].set_xlabel('Ruido (RPM)')
    axes[1, 1].set_ylabel('RMSE (RPM)')
    axes[1, 1].set_title('Error RMSE vs Ruido')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def experimento_frecuencias_muestreo():
    """
    Analiza el efecto de diferentes frecuencias de muestreo
    """
    print("="*70)
    print(" EXPERIMENTO: FRECUENCIAS DE MUESTREO")
    print("="*70)
    
    # Frecuencias a probar (Hz)
    frecuencias = [10, 20, 50, 100, 200]
    
    # Generar datos de referencia a alta frecuencia
    import config
    config.EJEMPLO_DT = 0.005  # 200 Hz
    tiempo_ref, velocidad_ref, _ = generar_datos_ejemplo()
    
    resultados = []
    
    for fs in frecuencias:
        print(f"\nFrecuencia: {fs} Hz...")
        
        # Submuestrear
        dt = 1/fs
        factor = int(dt / 0.005)
        tiempo = tiempo_ref[::factor]
        velocidad = velocidad_ref[::factor]
        
        modelo = identificar_por_ajuste(tiempo, velocidad, VOLTAJE_ESCALON)
        
        if modelo.get('exito'):
            resultados.append({
                'fs': fs,
                'K': modelo['K'],
                'tau': modelo['tau'],
                'fit': modelo['fit']
            })
    
    # Mostrar resultados
    print("\n" + "="*70)
    print(" RESULTADOS")
    print("="*70)
    print(f"{'Fs (Hz)':>10} {'K (RPM/V)':>12} {'τ (s)':>10} {'Ajuste (%)':>12}")
    print("-"*70)
    
    for r in resultados:
        print(f"{r['fs']:10.0f} {r['K']:12.2f} {r['tau']:10.4f} {r['fit']:12.1f}")


def menu_experimentos():
    """
    Menú interactivo para seleccionar experimentos
    """
    while True:
        print("\n" + "="*70)
        print(" MENÚ DE EXPERIMENTOS")
        print("="*70)
        print("1. Comparar métodos de identificación")
        print("2. Variación de parámetros (K y τ)")
        print("3. Efecto del ruido")
        print("4. Frecuencias de muestreo")
        print("0. Salir")
        print("="*70)
        
        opcion = input("\nSelecciona un experimento: ")
        
        if opcion == '1':
            experimento_comparar_metodos()
        elif opcion == '2':
            experimento_variacion_parametros()
        elif opcion == '3':
            experimento_efecto_ruido()
        elif opcion == '4':
            experimento_frecuencias_muestreo()
        elif opcion == '0':
            print("Saliendo...")
            break
        else:
            print("Opción no válida")


if __name__ == "__main__":
    menu_experimentos()
