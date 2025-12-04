"""
============================================================================
MÓDULO DE VISUALIZACIÓN
Funciones para generar todas las gráficas del análisis
============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from config import *

# Configurar estilo
plt.style.use(PLOT_STYLE)
plt.rcParams['figure.figsize'] = PLOT_FIGSIZE_MAIN
plt.rcParams['font.size'] = PLOT_FONTSIZE
plt.rcParams['figure.dpi'] = PLOT_DPI


def plot_respuesta_temporal(tiempo, velocidad, modelo_ajuste, modelo_63, ax=None):
    """
    Grafica respuesta temporal con datos y modelo
    
    Args:
        tiempo: Array de tiempos
        velocidad: Datos medidos
        modelo_ajuste: Resultados del ajuste
        modelo_63: Resultados del método 63.2%
        ax: Axes de matplotlib (None crea nuevo)
        
    Returns:
        Axes
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 6))
    
    # Datos medidos
    ax.plot(tiempo, velocidad, 'b.', markersize=3, alpha=0.5, label='Datos medidos')
    
    # Datos suavizados
    ax.plot(tiempo, modelo_63['velocidad_suave'], 'g-', linewidth=1.5, 
            label='Datos suavizados')
    
    # Modelo identificado
    ax.plot(tiempo, modelo_ajuste['y_modelo'], 'r-', linewidth=2, 
            label='Modelo identificado')
    
    # Marcar punto de 63.2%
    if modelo_63['idx_63'] is not None:
        idx = modelo_63['idx_63']
        ax.plot(tiempo[idx], modelo_63['velocidad_suave'][idx], 'ro', 
               markersize=10, markerfacecolor='red', markeredgewidth=2)
        ax.axhline(y=modelo_63['vel_63'], color='orange', linestyle='--', alpha=0.6)
        ax.axvline(x=modelo_63['tau'], color='orange', linestyle='--', alpha=0.6)
        ax.text(modelo_63['tau']+0.1, modelo_63['vel_63'], 
               f"τ={modelo_63['tau']:.3f}s", fontsize=10, 
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax.set_xlabel('Tiempo (s)', fontsize=11)
    ax.set_ylabel('Velocidad (RPM)', fontsize=11)
    ax.set_title(f"Respuesta al Escalón (Ajuste: {modelo_ajuste['fit']:.1f}%)", 
                fontsize=12, fontweight='bold')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    
    return ax


def plot_polos_ceros(modelo_ajuste, ax=None):
    """
    Diagrama de polos y ceros
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 6))
    
    polo = modelo_ajuste['polo']
    ax.plot(polo, 0, 'rx', markersize=20, markeredgewidth=3, label='Polo')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    
    # Círculo en origen
    circle = plt.Circle((0, 0), 0.5, fill=False, color='gray', 
                       linestyle='--', alpha=0.3)
    ax.add_patch(circle)
    
    ax.set_xlabel('Parte Real', fontsize=11)
    ax.set_ylabel('Parte Imaginaria', fontsize=11)
    ax.set_title('Diagrama de Polos y Ceros', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.text(polo, 0.3, f's = {polo:.2f}', fontsize=11, ha='center',
           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    ax.set_xlim([polo*1.5, 1])
    ax.set_ylim([-2, 2])
    ax.set_aspect('equal')
    
    return ax


def plot_error(tiempo, velocidad, y_modelo, ax=None):
    """
    Grafica error de ajuste
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 4))
    
    error = velocidad - y_modelo
    rmse = np.sqrt(np.mean(error**2))
    
    ax.plot(tiempo, error, 'r-', linewidth=1)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.fill_between(tiempo, error, 0, alpha=0.3, color='red')
    ax.set_xlabel('Tiempo (s)', fontsize=11)
    ax.set_ylabel('Error (RPM)', fontsize=11)
    ax.set_title(f"Error de Ajuste (RMSE: {rmse:.2f} RPM)", 
                fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    return ax


def plot_bode(analisis_freq, ax_mag=None, ax_fase=None):
    """
    Diagrama de Bode (magnitud y fase)
    """
    if ax_mag is None or ax_fase is None:
        fig, (ax_mag, ax_fase) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Magnitud
    ax_mag.semilogx(analisis_freq['w'], analisis_freq['mag_db'], 'b-', linewidth=2)
    ax_mag.axhline(y=analisis_freq['mag_db'][0]-3, color='r', 
                  linestyle='--', label='-3dB')
    ax_mag.axvline(x=analisis_freq['BW_rad'], color='r', linestyle='--')
    ax_mag.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax_mag.set_ylabel('Magnitud (dB)', fontsize=11)
    ax_mag.set_title('Diagrama de Bode - Magnitud', fontsize=12, fontweight='bold')
    ax_mag.grid(True, which='both', alpha=0.3)
    ax_mag.legend()
    
    # Fase
    from scipy import signal
    fase = np.angle(analisis_freq['h']) * 180/np.pi
    ax_fase.semilogx(analisis_freq['w'], fase, 'b-', linewidth=2)
    ax_fase.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax_fase.set_ylabel('Fase (grados)', fontsize=11)
    ax_fase.set_title('Diagrama de Bode - Fase', fontsize=12, fontweight='bold')
    ax_fase.grid(True, which='both', alpha=0.3)
    
    return ax_mag, ax_fase


def plot_tabla_parametros(modelo_ajuste, analisis_freq, dt, ax=None):
    """
    Tabla con parámetros identificados
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.axis('off')
    
    tabla_texto = f"""
╔═══════════════════════════════════════╗
║   PARÁMETROS IDENTIFICADOS            ║
╠═══════════════════════════════════════╣
║  Modelo: G(s) = K/(τs + 1)            ║
║                                       ║
║  K = {modelo_ajuste['K']:.2f} RPM/V                ║
║  τ = {modelo_ajuste['tau']:.4f} s                  ║
║  Polo = {modelo_ajuste['polo']:.2f} rad/s             ║
║                                       ║
║  Ajuste: {modelo_ajuste['fit']:.1f}%                     ║
║  RMSE: {modelo_ajuste['rmse']:.2f} RPM                   ║
╠═══════════════════════════════════════╣
║   FRECUENCIA DE MUESTREO              ║
╠═══════════════════════════════════════╣
║  Ancho de banda:                      ║
║    {analisis_freq['BW_rad']:.2f} rad/s ({analisis_freq['BW_hz']:.2f} Hz)      ║
║                                       ║
║  Nyquist: {analisis_freq['fs_nyquist']:.2f} Hz                 ║
║  Recomendada: {analisis_freq['fs_recomendada']:.2f} Hz            ║
║  Período: {analisis_freq['Ts_recomendado']*1000:.2f} ms                  ║
║                                       ║
║  Frecuencia actual: {1/dt:.1f} Hz          ║
║  Estado: {"✓ Adecuada" if 1/dt >= analisis_freq['fs_recomendada'] else "✗ Aumentar"}                  ║
╚═══════════════════════════════════════╝
    """
    
    ax.text(0.05, 0.5, tabla_texto, fontsize=9, family='monospace',
           verticalalignment='center', fontweight='normal')
    
    return ax


def graficar_resultados_completos(tiempo, velocidad, modelo_ajuste, modelo_63, 
                                  analisis_freq, voltaje_escalon, dt):
    """
    Genera figura completa con todos los análisis (6 subplots)
    """
    fig = plt.figure(figsize=PLOT_FIGSIZE_MAIN)
    fig.suptitle('IDENTIFICACIÓN DEL MODELO - RESPUESTA AL ESCALÓN', 
                fontsize=16, fontweight='bold')
    
    # Subplot 1: Respuesta temporal
    ax1 = plt.subplot(2, 3, 1)
    plot_respuesta_temporal(tiempo, velocidad, modelo_ajuste, modelo_63, ax1)
    
    # Subplot 2: Polos y ceros
    ax2 = plt.subplot(2, 3, 2)
    plot_polos_ceros(modelo_ajuste, ax2)
    
    # Subplot 3: Error
    ax3 = plt.subplot(2, 3, 3)
    plot_error(tiempo, velocidad, modelo_ajuste['y_modelo'], ax3)
    
    # Subplot 4-5: Bode
    ax4 = plt.subplot(2, 3, 4)
    ax5 = plt.subplot(2, 3, 5)
    plot_bode(analisis_freq, ax4, ax5)
    
    # Subplot 6: Tabla
    ax6 = plt.subplot(2, 3, 6)
    plot_tabla_parametros(modelo_ajuste, analisis_freq, dt, ax6)
    
    plt.tight_layout()
    
    return fig


def graficar_discretizacion(sys_continuo, Ts):
    """
    Compara modelo continuo vs discretizado
    """
    from frequency_analysis import discretizar_modelo
    from scipy import signal
    
    fig = plt.figure(figsize=PLOT_FIGSIZE_DISCRETE)
    fig.suptitle('DISCRETIZACIÓN DEL MODELO', fontsize=16, fontweight='bold')
    
    # Discretizar con diferentes métodos
    sys_tustin = discretizar_modelo(sys_continuo, Ts, 'tustin')
    sys_zoh = discretizar_modelo(sys_continuo, Ts, 'zoh')
    
    # Subplot 1: Respuesta al escalón
    ax1 = plt.subplot(1, 2, 1)
    t_cont, y_cont = signal.step(sys_continuo, N=300)
    t_tust, y_tust = signal.dstep(sys_tustin, n=150)
    t_zoh, y_zoh = signal.dstep(sys_zoh, n=150)
    
    ax1.plot(t_cont, y_cont, 'b-', linewidth=2, label='Continuo')
    ax1.plot(t_tust, y_tust[0], 'r--', linewidth=2, label='Tustin')
    ax1.plot(t_zoh, y_zoh[0], 'g:', linewidth=2, label='ZOH')
    ax1.set_xlabel('Tiempo (s)', fontsize=11)
    ax1.set_ylabel('Amplitud', fontsize=11)
    ax1.set_title('Comparación de Respuesta al Escalón', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Polos en plano Z
    ax2 = plt.subplot(1, 2, 2)
    polos_z = sys_tustin.poles
    ax2.plot(np.real(polos_z), np.imag(polos_z), 'rx', markersize=15, 
            markeredgewidth=3, label='Polos (Tustin)')
    
    # Círculo unitario
    theta = np.linspace(0, 2*np.pi, 100)
    ax2.plot(np.cos(theta), np.sin(theta), 'k--', linewidth=1, 
            label='Círculo unitario')
    
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    ax2.set_xlabel('Parte Real', fontsize=11)
    ax2.set_ylabel('Parte Imaginaria', fontsize=11)
    ax2.set_title('Polos en el Plano Z', fontsize=12, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    ax2.set_xlim([-1.5, 1.5])
    ax2.set_ylim([-1.5, 1.5])
    
    # Anotar polo
    for polo in polos_z:
        ax2.text(np.real(polo), np.imag(polo)+0.15, 
                f'z={polo:.3f}', fontsize=9, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    plt.tight_layout()
    
    return fig


def plot_comparacion_metodos(resultados_comparacion, tiempo, velocidad):
    """
    Compara visualmente diferentes métodos de identificación
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('COMPARACIÓN DE MÉTODOS DE IDENTIFICACIÓN', 
                fontsize=16, fontweight='bold')
    
    # Subplot 1: Respuestas temporales
    ax1 = axes[0, 0]
    ax1.plot(tiempo, velocidad, 'b.', markersize=2, alpha=0.4, label='Datos')
    
    colores = ['r', 'g', 'm']
    i = 0
    for nombre, resultado in resultados_comparacion.items():
        if resultado.get('exito') and 'y_modelo' in resultado:
            ax1.plot(tiempo, resultado['y_modelo'], 
                    color=colores[i], linewidth=2, label=nombre)
            i += 1
    
    ax1.set_xlabel('Tiempo (s)')
    ax1.set_ylabel('Velocidad (RPM)')
    ax1.set_title('Respuestas de Modelos Identificados')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Comparación de parámetros
    ax2 = axes[0, 1]
    metodos = []
    K_vals = []
    tau_vals = []
    
    for nombre, resultado in resultados_comparacion.items():
        if resultado.get('exito'):
            metodos.append(nombre)
            K_vals.append(resultado.get('K', 0))
            tau_vals.append(resultado.get('tau', 0))
    
    x = np.arange(len(metodos))
    width = 0.35
    ax2.bar(x - width/2, K_vals, width, label='K (RPM/V)', color='skyblue')
    ax2_twin = ax2.twinx()
    ax2_twin.bar(x + width/2, tau_vals, width, label='τ (s)', color='lightcoral')
    
    ax2.set_xlabel('Método')
    ax2.set_ylabel('K (RPM/V)', color='skyblue')
    ax2_twin.set_ylabel('τ (s)', color='lightcoral')
    ax2.set_title('Comparación de Parámetros')
    ax2.set_xticks(x)
    ax2.set_xticklabels(metodos, rotation=15)
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Subplot 3: Métricas de ajuste
    ax3 = axes[1, 0]
    fits = [r.get('fit', 0) for r in resultados_comparacion.values() if r.get('exito')]
    ax3.bar(metodos, fits, color=['green' if f > 95 else 'orange' for f in fits])
    ax3.set_ylabel('Ajuste (%)')
    ax3.set_title('Calidad de Ajuste por Método')
    ax3.set_ylim([0, 105])
    ax3.axhline(y=95, color='r', linestyle='--', label='Objetivo 95%')
    ax3.legend()
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Subplot 4: Tabla resumen
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    tabla = "╔══════════════════════════════════════╗\n"
    tabla += "║  RESUMEN DE MÉTODOS                  ║\n"
    tabla += "╠══════════════════════════════════════╣\n"
    
    for nombre, resultado in resultados_comparacion.items():
        if resultado.get('exito'):
            tabla += f"║ {nombre:20s}           ║\n"
            tabla += f"║   K = {resultado['K']:6.2f} RPM/V              ║\n"
            tabla += f"║   τ = {resultado['tau']:6.4f} s                ║\n"
            if 'fit' in resultado:
                tabla += f"║   Ajuste = {resultado['fit']:5.1f}%                ║\n"
            tabla += "╠══════════════════════════════════════╣\n"
    
    tabla += "╚══════════════════════════════════════╝"
    
    ax4.text(0.1, 0.5, tabla, fontsize=9, family='monospace',
            verticalalignment='center')
    
    plt.tight_layout()
    
    return fig
