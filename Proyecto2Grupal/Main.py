"""
============================================================================
IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC CON PUERTA CORREDIZA
Proyecto: Control de Velocidad de Puerta Corrediza
Método: Respuesta al Escalón (Step Response)
============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import pandas as pd
try:
    import control as ct
    CONTROL_AVAILABLE = True
except ImportError:
    CONTROL_AVAILABLE = False
    print("⚠️  Librería 'control' no disponible. Usando método manual de discretización.")

# Configurar estilo de gráficas
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 10

# ============================================================================
# 1. GENERAR DATOS DE SIMULACIÓN
# ============================================================================
def generar_datos_simulacion():
    """Genera datos de respuesta al escalón de segundo orden con ruido"""
    
    print("Simulación")
    
    # Especificaciones del motor DC:
    # - Voltaje nominal: 12V DC
    # - Potencia: ~40W
    # - Velocidad sin carga: ~2400 RPM
    # - Puerta: 1 kg
    
    # Parámetros del sistema real (con carga de 1kg)
    # Ganancia teórica sin carga: 2400 RPM / 12V = 200 RPM/V
    # Con carga se reduce ~50% por fricción e inercia
    K_real = 100        # Ganancia RPM/V (con carga de 1kg, reducida 50%)
    wn_real = 4.0       # Frecuencia natural (rad/s) - más lento que motor rápido
    zeta_real = 0.9     # Factor de amortiguamiento (sobreamortiguado)
    V_escalon = 6.0     # Voltaje aplicado (V) - 50% del nominal para control
    
    # Generar vector de tiempo
    dt = 0.02           # 50 Hz de muestreo
    t_final = 3.0       # 3 segundos
    tiempo = np.arange(0, t_final, dt)
    
    # Calcular respuesta teórica de segundo orden
    if zeta_real >= 1:  # Sobreamortiguado
        s1 = -zeta_real*wn_real + wn_real*np.sqrt(zeta_real**2 - 1)
        s2 = -zeta_real*wn_real - wn_real*np.sqrt(zeta_real**2 - 1)
        velocidad_teorica = K_real * V_escalon * (1 + (s1*np.exp(s2*tiempo) - s2*np.exp(s1*tiempo))/(s2-s1))
    else:  # Subamortiguado
        wd = wn_real * np.sqrt(1 - zeta_real**2)
        velocidad_teorica = K_real * V_escalon * (1 - np.exp(-zeta_real*wn_real*tiempo) * 
                           (np.cos(wd*tiempo) + (zeta_real/np.sqrt(1-zeta_real**2))*np.sin(wd*tiempo)))
    
    # Agregar ruido realista
    ruido = 15 * np.random.randn(len(tiempo))  # Menos ruido para motor más lento
    velocidad = velocidad_teorica + ruido
    velocidad = np.maximum(velocidad, 0)  # No negativas
    
    fs = 1/dt
    print(f"  Motor: 12V DC, 40W, 2400 RPM (sin carga)")
    print(f"  Carga: Puerta de 1 kg")
    
    return tiempo, velocidad, dt, V_escalon

# ============================================================================
# 2. IDENTIFICACIÓN DEL MODELO DE SEGUNDO ORDEN
# ============================================================================
def modelo_segundo_orden(t, K, wn, zeta):
    """Modelo teórico de segundo orden para ajuste"""
    if zeta < 1:  # Subamortiguado
        wd = wn * np.sqrt(1 - zeta**2)
        y = K * (1 - np.exp(-zeta*wn*t) * 
                 (np.cos(wd*t) + (zeta/np.sqrt(1-zeta**2))*np.sin(wd*t)))
    elif zeta == 1:  # Críticamente amortiguado
        y = K * (1 - np.exp(-wn*t) * (1 + wn*t))
    else:  # Sobreamortiguado
        s1 = -zeta*wn + wn*np.sqrt(zeta**2 - 1)
        s2 = -zeta*wn - wn*np.sqrt(zeta**2 - 1)
        y = K * (1 + (s1*np.exp(s2*t) - s2*np.exp(s1*t))/(s2-s1))
    return y

def identificar_modelo_segundo_orden(tiempo, velocidad, voltaje_escalon):
    """Identifica parámetros del modelo de segundo orden"""
    
    # Estimar valores iniciales
    vel_final = np.mean(velocidad[-20:])
    K_init = vel_final
    
    # Estimar frecuencia natural
    y_10 = 0.1 * vel_final
    y_90 = 0.9 * vel_final
    idx_10 = np.where(velocidad >= y_10)[0]
    idx_90 = np.where(velocidad >= y_90)[0]
    
    if len(idx_10) > 0 and len(idx_90) > 0:
        t_rise = tiempo[idx_90[0]] - tiempo[idx_10[0]]
        wn_init = 1.8 / t_rise
    else:
        wn_init = 5.0
    
    zeta_init = 0.7
    
    try:
        # Ajuste por curva
        popt, pcov = curve_fit(
            modelo_segundo_orden, 
            tiempo, 
            velocidad,
            p0=[K_init, wn_init, zeta_init],
            bounds=([0, 0.1, 0.1], [vel_final*2, 50.0, 5.0]),
            maxfev=10000
        )
        
        K_id, wn_id, zeta_id = popt
        
        # Modelo ajustado
        y_ajustado = modelo_segundo_orden(tiempo, K_id, wn_id, zeta_id)
        
        # Métricas de ajuste
        residuos = velocidad - y_ajustado
        ss_res = np.sum(residuos**2)
        ss_tot = np.sum((velocidad - np.mean(velocidad))**2)
        r2 = 1 - (ss_res / ss_tot)
        fit_percent = r2 * 100
        rmse = np.sqrt(np.mean(residuos**2))
        
        # Ganancia normalizada
        K_normalizado = K_id / voltaje_escalon
        
        # Calcular polos
        if zeta_id < 1:  # Subamortiguado (polos complejos)
            real_part = -zeta_id * wn_id
            imag_part = wn_id * np.sqrt(1 - zeta_id**2)
            polo1 = complex(real_part, imag_part)
            polo2 = complex(real_part, -imag_part)
            tipo_amort = "Subamortiguado"
        else:  # Sobreamortiguado (polos reales)
            polo1 = -zeta_id*wn_id + wn_id*np.sqrt(zeta_id**2 - 1)
            polo2 = -zeta_id*wn_id - wn_id*np.sqrt(zeta_id**2 - 1)
            tipo_amort = "Sobreamortiguado" if zeta_id > 1.2 else "Críticamente amortiguado"
        
        return {
            'K': K_normalizado,
            'wn': wn_id,
            'zeta': zeta_id,
            'polo1': polo1,
            'polo2': polo2,
            'tipo_amort': tipo_amort,
            'fit': fit_percent,
            'rmse': rmse,
            'y_modelo': y_ajustado,
            'K_total': K_id
        }
        
    except Exception as e:
        print(f"Error en ajuste: {e}")
        return None

# ============================================================================
# 3. ANÁLISIS DE FRECUENCIAS
# ============================================================================
def analizar_frecuencias(K, wn, zeta):
    """Calcula ancho de banda y frecuencias de muestreo"""
    
    # Función de transferencia de segundo orden
    num = [K * wn**2]
    den = [1, 2*zeta*wn, wn**2]
    sys = signal.TransferFunction(num, den)
    
    # Respuesta en frecuencia
    w = np.logspace(-2, 3, 1000)
    w_rad, h = signal.freqs(num, den, worN=w)
    mag_db = 20 * np.log10(np.abs(h))
    
    # Ancho de banda (-3dB)
    idx_3db = np.where(mag_db <= (mag_db[0] - 3))[0]
    if len(idx_3db) > 0:
        BW_rad = w_rad[idx_3db[0]]
    else:
        BW_rad = wn
    
    BW_hz = BW_rad / (2*np.pi)
    
    # Frecuencias de muestreo
    fs_nyquist = 2 * BW_hz
    fs_recomendada = 10 * BW_hz
    Ts_recomendado = 1 / fs_recomendada
    
    return {
        'sys': sys,
        'BW_rad': BW_rad,
        'BW_hz': BW_hz,
        'fs_nyquist': fs_nyquist,
        'fs_recomendada': fs_recomendada,
        'Ts_recomendado': Ts_recomendado,
        'w': w_rad,
        'mag_db': mag_db
    }

# ============================================================================
# 4. DISCRETIZACIÓN DEL MODELO
# ============================================================================
def discretizar_modelo(K, wn, zeta, Ts):
    """Discretiza el modelo de segundo orden usando ZOH"""
    
    # Sistema continuo
    num_c = [K * wn**2]
    den_c = [1, 2*zeta*wn, wn**2]
    sys_continuo = signal.TransferFunction(num_c, den_c)
    
    if CONTROL_AVAILABLE:
        # Usar librería control (método preferido)
        sys_c = ct.TransferFunction(num_c, den_c)
        sys_discreto = ct.sample_system(sys_c, Ts, method='zoh')
        num_d = sys_discreto.num[0][0]
        den_d = sys_discreto.den[0][0]
    else:
        # Método manual usando scipy
        sys_discreto = signal.cont2discrete((num_c, den_c), Ts, method='zoh')
        num_d = sys_discreto[0].flatten()
        den_d = sys_discreto[1]
    
    # Calcular polos discretos
    polos_discretos = np.roots(den_d)
    
    # Calcular ceros discretos (si existen)
    if len(num_d) > 1:
        ceros_discretos = np.roots(num_d)
    else:
        ceros_discretos = np.array([])
    
    return {
        'sys_continuo': sys_continuo,
        'num_d': num_d,
        'den_d': den_d,
        'polos_d': polos_discretos,
        'ceros_d': ceros_discretos,
        'Ts': Ts
    }

def simular_respuesta_discreta(num_d, den_d, tiempo, voltaje_escalon):
    """Simula la respuesta del sistema discreto"""
    
    # Generar respuesta al escalón
    tout, yout = signal.dstep((num_d, den_d, 1), n=len(tiempo))
    
    # signal.dstep devuelve una tupla (tout, yout)
    # yout es un array, extraerlo correctamente
    if isinstance(yout, tuple):
        y_out = yout[0].flatten()
    else:
        y_out = yout.flatten() if hasattr(yout, 'flatten') else np.array(yout).flatten()
    
    # Escalar por el voltaje del escalón
    y_out = y_out * voltaje_escalon
    
    return y_out

# ============================================================================
# 5. VISUALIZACIÓN
# ============================================================================
def graficar_resultados(tiempo, velocidad, modelo, analisis, voltaje_escalon, dt):
    """Genera todas las gráficas para el reporte"""
    
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('IDENTIFICACIÓN DEL MODELO - RESPUESTA AL ESCALÓN (SEGUNDO ORDEN)', 
                  fontsize=16, fontweight='bold')
    
    # ========== Subplot 1: Respuesta al escalón ==========
    ax1 = plt.subplot(2, 3, 1)
    
    # Suavizar datos para visualización
    ventana = 5
    velocidad_suave = np.convolve(velocidad, np.ones(ventana)/ventana, mode='same')
    
    ax1.plot(tiempo, velocidad, 'b.', markersize=3, alpha=0.5, label='Datos medidos')
    ax1.plot(tiempo, velocidad_suave, 'g-', linewidth=1.5, label='Datos suavizados')
    ax1.plot(tiempo, modelo['y_modelo'], 'r-', linewidth=2, label='Modelo identificado')
    
    ax1.set_xlabel('Tiempo (s)', fontsize=11)
    ax1.set_ylabel('Velocidad (RPM)', fontsize=11)
    ax1.set_title(f"Respuesta al Escalón (Ajuste: {modelo['fit']:.1f}%)", 
                 fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    
    # ========== Subplot 2: Diagrama de polos ==========
    ax2 = plt.subplot(2, 3, 2)
    
    if isinstance(modelo['polo1'], complex):
        # Polos complejos conjugados
        ax2.plot(np.real(modelo['polo1']), np.imag(modelo['polo1']), 
                'rx', markersize=20, markeredgewidth=3, label='Polos')
        ax2.plot(np.real(modelo['polo2']), np.imag(modelo['polo2']), 
                'rx', markersize=20, markeredgewidth=3)
        
        ax2.text(np.real(modelo['polo1'])-0.5, np.imag(modelo['polo1'])+0.5, 
                f's₁ = {modelo["polo1"]:.2f}', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        ax2.text(np.real(modelo['polo2'])-0.5, np.imag(modelo['polo2'])-0.8, 
                f's₂ = {modelo["polo2"]:.2f}', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        
        y_lim = abs(np.imag(modelo['polo1'])) * 1.5
        ax2.set_ylim([-y_lim, y_lim])
    else:
        # Polos reales
        ax2.plot(modelo['polo1'], 0, 'rx', markersize=20, markeredgewidth=3)
        ax2.plot(modelo['polo2'], 0, 'rx', markersize=20, markeredgewidth=3, label='Polos')
        
        ax2.text(modelo['polo1'], 0.3, f's₁ = {modelo["polo1"]:.2f}', 
                fontsize=10, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        ax2.text(modelo['polo2'], -0.3, f's₂ = {modelo["polo2"]:.2f}', 
                fontsize=10, ha='center',
                bbox=dict(boxstyle='round', facecolor='cyan', alpha=0.7))
        ax2.set_ylim([-2, 2])
    
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    
    circle = plt.Circle((0, 0), 0.5, fill=False, color='gray', linestyle='--', alpha=0.3)
    ax2.add_patch(circle)
    
    ax2.set_xlabel('Parte Real', fontsize=11)
    ax2.set_ylabel('Parte Imaginaria', fontsize=11)
    ax2.set_title('Diagrama de Polos y Ceros', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    x_min = min(np.real(modelo['polo1']) if isinstance(modelo['polo1'], complex) else modelo['polo1'],
                np.real(modelo['polo2']) if isinstance(modelo['polo2'], complex) else modelo['polo2']) * 1.5
    ax2.set_xlim([x_min, 1])
    
    # ========== Subplot 3: Error de ajuste ==========
    ax3 = plt.subplot(2, 3, 3)
    error = velocidad - modelo['y_modelo']
    ax3.plot(tiempo, error, 'r-', linewidth=1)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax3.fill_between(tiempo, error, 0, alpha=0.3, color='red')
    ax3.set_xlabel('Tiempo (s)', fontsize=11)
    ax3.set_ylabel('Error (RPM)', fontsize=11)
    ax3.set_title(f"Error de Ajuste (RMSE: {modelo['rmse']:.2f} RPM)", 
                 fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # ========== Subplot 4: Bode - Magnitud ==========
    ax4 = plt.subplot(2, 3, 4)
    ax4.semilogx(analisis['w'], analisis['mag_db'], 'b-', linewidth=2)
    ax4.axhline(y=analisis['mag_db'][0]-3, color='r', linestyle='--', label='-3dB')
    ax4.axvline(x=analisis['BW_rad'], color='r', linestyle='--')
    ax4.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax4.set_ylabel('Magnitud (dB)', fontsize=11)
    ax4.set_title('Diagrama de Bode - Magnitud', fontsize=12, fontweight='bold')
    ax4.grid(True, which='both', alpha=0.3)
    ax4.legend()
    
    # ========== Subplot 5: Bode - Fase ==========
    ax5 = plt.subplot(2, 3, 5)
    num = [modelo['K_total'] * modelo['wn']**2]
    den = [1, 2*modelo['zeta']*modelo['wn'], modelo['wn']**2]
    _, h = signal.freqs(num, den, worN=analisis['w'])
    fase = np.angle(h) * 180/np.pi
    ax5.semilogx(analisis['w'], fase, 'b-', linewidth=2)
    ax5.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax5.set_ylabel('Fase (grados)', fontsize=11)
    ax5.set_title('Diagrama de Bode - Fase', fontsize=12, fontweight='bold')
    ax5.grid(True, which='both', alpha=0.3)
    
    # ========== Subplot 6: Tabla de parámetros ==========
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')
    
    if isinstance(modelo['polo1'], complex):
        polos_str = f"s₁,₂ = {np.real(modelo['polo1']):.2f} ± j{abs(np.imag(modelo['polo1'])):.2f}"
    else:
        polos_str = f"s₁ = {modelo['polo1']:.2f}, s₂ = {modelo['polo2']:.2f}"
    
    tabla_texto = f"""
╔═══════════════════════════════════════╗
║   PARÁMETROS IDENTIFICADOS            ║
╠═══════════════════════════════════════╣
║  Modelo: G(s) = K·ωₙ²/(s²+2ζωₙs+ωₙ²)    ║
║                                       ║
║  K = {modelo['K']:.2f} RPM/V          ║
║  ωₙ = {modelo['wn']:.2f} rad/s        ║
║  ζ = {modelo['zeta']:.2f}             ║
║                                       ║
║  Polos: {polos_str:<28}               ║
║                                       ║
║  Tipo: {modelo['tipo_amort']:<28}     ║
║  Ajuste: {modelo['fit']:.1f}%         ║
║  RMSE: {modelo['rmse']:.2f} RPM       ║
╠═══════════════════════════════════════╣
║   FRECUENCIA DE MUESTREO              ║
╠═══════════════════════════════════════╣
║  Ancho de banda:                      ║
║    {analisis['BW_rad']:.2f} rad/s ({analisis['BW_hz']:.2f} Hz)      ║
║                                       ║
║  Nyquist: {analisis['fs_nyquist']:.2f} Hz                 ║
║  Recomendada: {analisis['fs_recomendada']:.2f} Hz            ║
║  Período: {analisis['Ts_recomendado']*1000:.2f} ms                  ║
║                                       ║
║  Frecuencia actual: {1/dt:.1f} Hz          ║
║  Estado: {"✓ Adecuada" if 1/dt >= analisis['fs_recomendada'] else "✗ Aumentar"}                  ║
╚═══════════════════════════════════════╝
    """
    
    ax6.text(0.05, 0.5, tabla_texto, fontsize=9, family='monospace',
            verticalalignment='center')
    
    plt.tight_layout()
    return fig

def graficar_discretizacion(tiempo, velocidad, modelo, modelo_discreto, voltaje_escalon, dt):
    """Genera gráficas comparativas del modelo continuo vs discreto"""
    
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('DISCRETIZACIÓN DEL MODELO - COMPARACIÓN CONTINUO VS DISCRETO', 
                  fontsize=16, fontweight='bold')
    
    # Simular respuesta discreta
    y_discreto = simular_respuesta_discreta(
        modelo_discreto['num_d'], 
        modelo_discreto['den_d'], 
        tiempo, 
        voltaje_escalon
    )
    
    # ========== Subplot 1: Comparación de respuestas ==========
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(tiempo, velocidad, 'b.', markersize=3, alpha=0.5, label='Datos medidos')
    ax1.plot(tiempo, modelo['y_modelo'], 'r-', linewidth=2, label='Modelo continuo')
    ax1.plot(tiempo[:len(y_discreto)], y_discreto, 'g--', linewidth=2, label='Modelo discreto', marker='o', markersize=4, markevery=5)
    ax1.set_xlabel('Tiempo (s)', fontsize=11)
    ax1.set_ylabel('Velocidad (RPM)', fontsize=11)
    ax1.set_title('Comparación: Continuo vs Discreto', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    
    # ========== Subplot 2: Diagrama de polos z-plane ==========
    ax2 = plt.subplot(2, 3, 2)
    
    # Círculo unitario
    theta = np.linspace(0, 2*np.pi, 100)
    ax2.plot(np.cos(theta), np.sin(theta), 'k--', linewidth=1, label='Círculo unitario')
    
    # Polos discretos
    for i, polo in enumerate(modelo_discreto['polos_d']):
        ax2.plot(np.real(polo), np.imag(polo), 'rx', markersize=20, markeredgewidth=3,
                label='Polos' if i == 0 else '')
        ax2.text(np.real(polo)+0.05, np.imag(polo)+0.05, 
                f'z{i+1}={polo:.3f}', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Ceros discretos
    if len(modelo_discreto['ceros_d']) > 0:
        for i, cero in enumerate(modelo_discreto['ceros_d']):
            ax2.plot(np.real(cero), np.imag(cero), 'bo', markersize=15, markeredgewidth=2,
                    label='Ceros' if i == 0 else '')
            ax2.text(np.real(cero)+0.05, np.imag(cero)-0.1, 
                    f'z{i+1}={cero:.3f}', fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='cyan', alpha=0.7))
    
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    ax2.set_xlabel('Parte Real', fontsize=11)
    ax2.set_ylabel('Parte Imaginaria', fontsize=11)
    ax2.set_title('Diagrama de Polos y Ceros (Plano Z)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.axis('equal')
    ax2.set_xlim([-1.5, 1.5])
    ax2.set_ylim([-1.5, 1.5])
    
    # ========== Subplot 3: Error discretización ==========
    ax3 = plt.subplot(2, 3, 3)
    error_disc = modelo['y_modelo'][:len(y_discreto)] - y_discreto
    ax3.plot(tiempo[:len(y_discreto)], error_disc, 'purple', linewidth=1.5)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax3.fill_between(tiempo[:len(y_discreto)], error_disc, 0, alpha=0.3, color='purple')
    ax3.set_xlabel('Tiempo (s)', fontsize=11)
    ax3.set_ylabel('Error (RPM)', fontsize=11)
    rmse_disc = np.sqrt(np.mean(error_disc**2))
    ax3.set_title(f'Error Continuo - Discreto (RMSE: {rmse_disc:.2f} RPM)', 
                 fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # ========== Subplot 4: Respuesta en frecuencia (Magnitud) ==========
    ax4 = plt.subplot(2, 3, 4)
    
    # Continuo
    w = np.logspace(-2, 3, 1000)
    num_c = [modelo['K_total'] * modelo['wn']**2]
    den_c = [1, 2*modelo['zeta']*modelo['wn'], modelo['wn']**2]
    _, h_c = signal.freqs(num_c, den_c, worN=w)
    mag_c_db = 20 * np.log10(np.abs(h_c))
    
    # Discreto
    w_d = np.logspace(-2, np.log10(np.pi/modelo_discreto['Ts']), 1000)
    _, h_d = signal.freqz(modelo_discreto['num_d'], modelo_discreto['den_d'], 
                         worN=w_d*modelo_discreto['Ts'])
    mag_d_db = 20 * np.log10(np.abs(h_d))
    
    ax4.semilogx(w, mag_c_db, 'r-', linewidth=2, label='Continuo')
    ax4.semilogx(w_d, mag_d_db, 'g--', linewidth=2, label='Discreto')
    ax4.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax4.set_ylabel('Magnitud (dB)', fontsize=11)
    ax4.set_title('Bode - Magnitud (Continuo vs Discreto)', fontsize=12, fontweight='bold')
    ax4.grid(True, which='both', alpha=0.3)
    ax4.legend()
    
    # ========== Subplot 5: Respuesta en frecuencia (Fase) ==========
    ax5 = plt.subplot(2, 3, 5)
    fase_c = np.angle(h_c) * 180/np.pi
    fase_d = np.angle(h_d) * 180/np.pi
    ax5.semilogx(w, fase_c, 'r-', linewidth=2, label='Continuo')
    ax5.semilogx(w_d, fase_d, 'g--', linewidth=2, label='Discreto')
    ax5.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax5.set_ylabel('Fase (grados)', fontsize=11)
    ax5.set_title('Bode - Fase (Continuo vs Discreto)', fontsize=12, fontweight='bold')
    ax5.grid(True, which='both', alpha=0.3)
    ax5.legend()
    
    # ========== Subplot 6: Tabla de parámetros discretos ==========
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')
    
    # Formatear numerador y denominador
    num_str = ' + '.join([f'{coef:.6f}z^{len(modelo_discreto["num_d"])-i-1}' 
                         for i, coef in enumerate(modelo_discreto['num_d'])])
    den_str = ' + '.join([f'{coef:.6f}z^{len(modelo_discreto["den_d"])-i-1}' 
                         for i, coef in enumerate(modelo_discreto['den_d'])])
    
    # Formatear polos
    if np.any(np.iscomplex(modelo_discreto['polos_d'])):
        polos_str = ', '.join([f'{p:.4f}' for p in modelo_discreto['polos_d']])
    else:
        polos_str = ', '.join([f'{p.real:.4f}' for p in modelo_discreto['polos_d']])
    
    # Verificar estabilidad
    magnitudes = np.abs(modelo_discreto['polos_d'])
    estable = "✓ ESTABLE" if np.all(magnitudes < 1) else "✗ INESTABLE"
    max_mag = np.max(magnitudes)
    
    tabla_texto = f"""
╔═══════════════════════════════════════╗
║   MODELO DISCRETIZADO (ZOH)           ║
╠═══════════════════════════════════════╣
║  Ts = {modelo_discreto['Ts']*1000:.2f} ms                          ║
║  fs = {1/modelo_discreto['Ts']:.2f} Hz                       ║
║                                       ║
║  Numerador:                           ║
║  {modelo_discreto['num_d'][0]:.6f}z + {modelo_discreto['num_d'][1]:.6f}              ║
║                                       ║
║  Denominador:                         ║
║  z² + {modelo_discreto['den_d'][1]:.6f}z + {modelo_discreto['den_d'][2]:.6f}       ║
║                                       ║
║  Polos discretos:                     ║
║  z₁ = {modelo_discreto['polos_d'][0]:.4f}                  ║
║  z₂ = {modelo_discreto['polos_d'][1]:.4f}                  ║
║                                       ║
║  |z_max| = {max_mag:.4f}                      ║
║  Estabilidad: {estable:<21}    ║
║                                       ║
║  RMSE discretización: {rmse_disc:.2f} RPM      ║
╚═══════════════════════════════════════╝
    """
    
    ax6.text(0.05, 0.5, tabla_texto, fontsize=9, family='monospace',
            verticalalignment='center')
    
    plt.tight_layout()
    return fig

# ============================================================================
# 6. PROGRAMA PRINCIPAL
# ============================================================================
def main():
    """Ejecuta el proceso completo de identificación"""
    
    print("="*70)
    print(" IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC (SEGUNDO ORDEN)")
    print("="*70)
    
    # Parámetros
    VOLTAJE_ESCALON = 6.0  # Voltaje aplicado (50% del nominal de 12V)
    
    # 1.  datos
    print("\n[1/4] Datos...")
    tiempo, velocidad, dt, V_escalon = generar_datos_simulacion()
    
    # 2. Identificar modelo
    print("\n[2/4] Identificando modelo de segundo orden...")
    modelo = identificar_modelo_segundo_orden(tiempo, velocidad, VOLTAJE_ESCALON)
    
    if modelo is None:
        print("❌ Error en la identificación")
        return
    
    print(f"  ✓ K = {modelo['K']:.2f} RPM/V")
    print(f"  ✓ ωₙ = {modelo['wn']:.2f} rad/s")
    print(f"  ✓ ζ = {modelo['zeta']:.2f}")
    print(f"  ✓ Tipo: {modelo['tipo_amort']}")
    
    if isinstance(modelo['polo1'], complex):
        print(f"  ✓ Polos: {modelo['polo1']:.2f}, {modelo['polo2']:.2f} (complejos)")
    else:
        print(f"  ✓ Polo 1: {modelo['polo1']:.2f} rad/s")
        print(f"  ✓ Polo 2: {modelo['polo2']:.2f} rad/s")
    
    print(f"  ✓ Ajuste: {modelo['fit']:.1f}%")
    
    # 3. Analizar frecuencias
    print("\n[3/4] Analizando frecuencias...")
    analisis = analizar_frecuencias(modelo['K_total'], modelo['wn'], modelo['zeta'])
    
    print(f"  ✓ Ancho de banda: {analisis['BW_hz']:.2f} Hz")
    print(f"  ✓ Frecuencia recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"  ✓ Período recomendado: {analisis['Ts_recomendado']*1000:.2f} ms")
    
    # 4. Generar gráficas
    print("\n[4/5] Generando gráficas del modelo continuo...")
    fig = graficar_resultados(tiempo, velocidad, modelo, analisis, VOLTAJE_ESCALON, dt)
    
    # 5. Discretizar modelo
    print("\n[5/5] Discretizando modelo...")
    Ts_discreto = analisis['Ts_recomendado']
    modelo_discreto = discretizar_modelo(modelo['K_total'], modelo['wn'], modelo['zeta'], Ts_discreto)
    
    print(f"  ✓ Ts = {Ts_discreto*1000:.2f} ms")
    print(f"  ✓ Polos discretos: {modelo_discreto['polos_d']}")
    magnitudes = np.abs(modelo_discreto['polos_d'])
    print(f"  ✓ |z_max| = {np.max(magnitudes):.4f}")
    print(f"  ✓ Sistema: {'ESTABLE' if np.all(magnitudes < 1) else 'INESTABLE'}")
    
    # Generar ventana de discretización
    fig_discreto = graficar_discretizacion(tiempo, velocidad, modelo, modelo_discreto, VOLTAJE_ESCALON, dt)
    
    # Resumen final
    print("\n" + "="*70)
    print(" RESUMEN FINAL")
    print("="*70)
    print(f"\nModelo identificado (Segundo Orden):")
    print(f"  G(s) = {modelo['K']:.2f}·{modelo['wn']:.2f}² / (s² + {2*modelo['zeta']*modelo['wn']:.2f}s + {modelo['wn']:.2f}²)")
    print(f"\nParámetros:")
    print(f"  K = {modelo['K']:.2f} RPM/V")
    print(f"  ωₙ = {modelo['wn']:.2f} rad/s")
    print(f"  ζ = {modelo['zeta']:.2f}")
    print(f"\nPolos:")
    if isinstance(modelo['polo1'], complex):
        print(f"  s₁,₂ = {modelo['polo1']:.2f}, {modelo['polo2']:.2f}")
    else:
        print(f"  s₁ = {modelo['polo1']:.2f} rad/s")
        print(f"  s₂ = {modelo['polo2']:.2f} rad/s")
    print(f"  Ceros: Ninguno")
    print(f"  Tipo: {modelo['tipo_amort']}")
    print(f"\nAjuste: {modelo['fit']:.1f}%")
    print(f"Frecuencia de muestreo recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"\nModelo Discreto (ZOH, Ts={Ts_discreto*1000:.2f} ms):")
    print(f"  Numerador: {modelo_discreto['num_d']}")
    print(f"  Denominador: {modelo_discreto['den_d']}")
    print(f"  Polos: {modelo_discreto['polos_d']}")
    print(f"  Estabilidad: {'ESTABLE (todos los polos dentro del círculo unitario)' if np.all(magnitudes < 1) else 'INESTABLE'}")
    print("\n✓ Proceso completado!")
    print("\n📊 Se generaron 2 ventanas:")
    print("   1. Identificación del modelo continuo")
    print("   2. Discretización y comparación continuo vs discreto")
    
    plt.show()

if __name__ == "__main__":
    main()