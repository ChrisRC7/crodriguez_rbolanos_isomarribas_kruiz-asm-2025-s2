"""
============================================================================
IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC CON PUERTA CORREDIZA
Proyecto: Control de Velocidad de Puerta Corrediza
Método: Respuesta al Escalón (Step Response) - SEGUNDO ORDEN

Equivalente a System Identification Toolbox de MATLAB usando Python
============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit

# Configurar estilo de gráficas (similar a MATLAB)
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 10

# ============================================================================
# PARÁMETROS DE LA PLANTA - MODIFICAR AQUÍ
# ============================================================================
# Especificaciones del motor
V_escalon = 12.0      # Voltaje nominal (V DC)
Potencia = 40.0       # Potencia nominal (W)
RPM_nominal = 2400.0  # Velocidad nominal (RPM)

# Parámetros del sistema de segundo orden
# G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
K_real = RPM_nominal / V_escalon  # Ganancia: 200 RPM/V
wn_real = 8.0         # Frecuencia natural (rad/s) - ajustar según respuesta deseada
zeta_real = 0.6       # Factor de amortiguamiento (0 < zeta < 1 para subamortiguado)

# Parámetros de simulación
dt = 0.01             # Período de muestreo: 100 Hz (más rápido para segundo orden)
t_final = 3.0         # Duración de la simulación (segundos)
ruido_amplitud = 20.0 # Amplitud del ruido (RPM)

# ============================================================================
# 1. GENERAR DATOS DE SIMULACIÓN
# ============================================================================
def generar_datos_simulacion():
    """
    Genera datos de simulación basados en los parámetros configurados
    Sistema de segundo orden: G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    """
    # Generar tiempo
    tiempo = np.arange(0, t_final, dt)
    
    # Crear sistema de segundo orden
    num = [K_real * wn_real**2]
    den = [1, 2*zeta_real*wn_real, wn_real**2]
    sistema = signal.TransferFunction(num, den)
    
    # Generar respuesta al escalón
    t_sim, velocidad_teorica = signal.step(sistema, T=tiempo)
    velocidad_teorica = velocidad_teorica * V_escalon
    
    # Añadir ruido
    ruido = ruido_amplitud * np.random.randn(len(tiempo))
    velocidad = velocidad_teorica + ruido
    velocidad = np.maximum(velocidad, 0)  # No negativas
    
    fs = 1/dt
    
    print(f"✓ Datos generados: {len(tiempo)} muestras")
    print(f"  Período de muestreo: {dt:.4f} s ({fs:.1f} Hz)")
    print(f"  Duración total: {tiempo[-1]:.2f} s")
    
    return tiempo, velocidad, dt

# ============================================================================
# 2. IDENTIFICACIÓN DEL MODELO
# ============================================================================
def modelo_segundo_orden(t, K, wn, zeta):
    """Modelo teórico de segundo orden"""
    num = [K * wn**2]
    den = [1, 2*zeta*wn, wn**2]
    sistema = signal.TransferFunction(num, den)
    _, y = signal.step(sistema, T=t)
    return y

def identificar_modelo_ajuste(tiempo, velocidad, voltaje_escalon):
    """
    Identificación por ajuste de curva (curve fitting)
    Sistema de segundo orden: G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    """
    
    # Valores iniciales inteligentes
    vel_final = np.mean(velocidad[-20:])
    K_init = vel_final / voltaje_escalon
    wn_init = 5.0
    zeta_init = 0.7
    
    try:
        # Ajuste de curva con bounds
        popt, pcov = curve_fit(
            modelo_segundo_orden, 
            tiempo, 
            velocidad,
            p0=[K_init, wn_init, zeta_init],
            bounds=([0, 0.1, 0.01], [K_init*2, 50.0, 2.0]),
            maxfev=10000
        )
        
        K_id, wn_id, zeta_id = popt
        
        # Calcular modelo ajustado
        y_ajustado = modelo_segundo_orden(tiempo, K_id, wn_id, zeta_id)
        
        # Calcular métricas de ajuste
        residuos = velocidad - y_ajustado
        ss_res = np.sum(residuos**2)
        ss_tot = np.sum((velocidad - np.mean(velocidad))**2)
        r2 = 1 - (ss_res / ss_tot)
        fit_percent = r2 * 100
        rmse = np.sqrt(np.mean(residuos**2))
        
        # Calcular polos
        a = 1
        b = 2*zeta_id*wn_id
        c = wn_id**2
        discriminante = b**2 - 4*a*c
        
        if discriminante >= 0:
            polo1 = (-b + np.sqrt(discriminante)) / (2*a)
            polo2 = (-b - np.sqrt(discriminante)) / (2*a)
        else:
            real = -b / (2*a)
            imag = np.sqrt(-discriminante) / (2*a)
            polo1 = complex(real, imag)
            polo2 = complex(real, -imag)
        
        # Calcular overshoot y tiempo de asentamiento
        if zeta_id < 1:
            overshoot = 100 * np.exp(-np.pi * zeta_id / np.sqrt(1 - zeta_id**2))
        else:
            overshoot = 0
        
        ts = 4 / (zeta_id * wn_id)  # Tiempo de asentamiento (2%)
        
        return {
            'metodo': 'Ajuste de curva - Segundo Orden',
            'K': K_id,
            'wn': wn_id,
            'zeta': zeta_id,
            'polo1': polo1,
            'polo2': polo2,
            'fit': fit_percent,
            'rmse': rmse,
            'overshoot': overshoot,
            'ts': ts,
            'y_modelo': y_ajustado
        }
        
    except Exception as e:
        print(f"Error en ajuste: {e}")
        return None

def identificar_modelo_63(tiempo, velocidad, voltaje_escalon):
    """
    Análisis básico de respuesta para segundo orden
    """
    
    # Suavizar datos
    ventana = 5
    velocidad_suave = np.convolve(velocidad, np.ones(ventana)/ventana, mode='same')
    
    # Valor final (promedio últimos valores)
    vel_final = np.mean(velocidad_suave[-20:])
    
    # Ganancia
    K = vel_final / voltaje_escalon
    
    # Encontrar pico máximo (overshoot)
    idx_max = np.argmax(velocidad_suave)
    vel_max = velocidad_suave[idx_max]
    overshoot_pct = ((vel_max - vel_final) / vel_final) * 100 if vel_final > 0 else 0
    tiempo_pico = tiempo[idx_max]
    
    return {
        'metodo': 'Análisis de respuesta',
        'K': K,
        'vel_final': vel_final,
        'vel_max': vel_max,
        'overshoot': overshoot_pct,
        'tiempo_pico': tiempo_pico,
        'idx_max': idx_max,
        'velocidad_suave': velocidad_suave
    }

# ============================================================================
# 3. ANÁLISIS DE FRECUENCIAS Y DISCRETIZACIÓN
# ============================================================================
def analizar_frecuencias(K, wn, zeta):
    """
    Calcula ancho de banda y frecuencias de muestreo recomendadas
    Para sistema de segundo orden
    """
    
    # Crear función de transferencia
    num = [K * wn**2]
    den = [1, 2*zeta*wn, wn**2]
    sys = signal.TransferFunction(num, den)
    
    # Calcular ancho de banda (-3dB)
    w = np.logspace(-2, 3, 1000)
    w_rad, h = signal.freqs(num, den, worN=w)
    mag_db = 20 * np.log10(np.abs(h))
    
    # Encontrar frecuencia de corte (-3dB)
    idx_3db = np.where(mag_db <= (mag_db[0] - 3))[0]
    if len(idx_3db) > 0:
        BW_rad = w_rad[idx_3db[0]]
    else:
        BW_rad = wn  # Aproximación para segundo orden
    
    BW_hz = BW_rad / (2*np.pi)
    
    # Frecuencias según criterio de Nyquist
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

def discretizar_modelo(sys_continuo, Ts, metodo='tustin'):
    """
    Discretiza el modelo continuo
    Métodos: 'tustin', 'zoh', 'euler'
    """
    
    if metodo == 'tustin' or metodo == 'bilinear':
        sys_discreto = sys_continuo.to_discrete(Ts, method='bilinear')
    elif metodo == 'zoh':
        sys_discreto = sys_continuo.to_discrete(Ts, method='zoh')
    elif metodo == 'euler':
        sys_discreto = sys_continuo.to_discrete(Ts, method='euler')
    else:
        sys_discreto = sys_continuo.to_discrete(Ts, method='bilinear')
    
    return sys_discreto

# ============================================================================
# 4. VISUALIZACIÓN COMPLETA
# ============================================================================
def graficar_resultados(tiempo, velocidad, modelo_ajuste, modelo_63, 
                        analisis_freq, voltaje_escalon, dt):
    """
    Genera todas las gráficas necesarias para el reporte
    Sistema de segundo orden
    """
    
    # FIGURA 1: Validación del Modelo (6 subplots)
    fig1 = plt.figure(figsize=(16, 10))
    fig1.suptitle('IDENTIFICACIÓN DEL MODELO - RESPUESTA AL ESCALÓN (SEGUNDO ORDEN)', 
                  fontsize=16, fontweight='bold')
    
    # Subplot 1: Respuesta temporal
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(tiempo, velocidad, 'b.', markersize=3, alpha=0.5, 
             label='Datos medidos')
    ax1.plot(tiempo, modelo_63['velocidad_suave'], 'g-', linewidth=1.5, 
             label='Datos suavizados')
    ax1.plot(tiempo, modelo_ajuste['y_modelo'], 'r-', linewidth=2, 
             label='Modelo identificado')
    
    # Marcar pico máximo (overshoot)
    if modelo_63['idx_max'] is not None:
        idx = modelo_63['idx_max']
        ax1.plot(tiempo[idx], modelo_63['velocidad_suave'][idx], 'mo', 
                markersize=10, markerfacecolor='magenta', markeredgewidth=2)
        ax1.axhline(y=modelo_63['vel_final'], color='orange', 
                   linestyle='--', alpha=0.6, label='Valor final')
        ax1.text(tiempo[idx]+0.1, modelo_63['vel_max'], 
                f"OS={modelo_63['overshoot']:.1f}%", fontsize=10, 
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax1.set_xlabel('Tiempo (s)', fontsize=11)
    ax1.set_ylabel('Velocidad (RPM)', fontsize=11)
    ax1.set_title(f"Respuesta al Escalón (Ajuste: {modelo_ajuste['fit']:.1f}%)", 
                 fontsize=12, fontweight='bold')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Diagrama de Polos y Ceros
    ax2 = plt.subplot(2, 3, 2)
    polo1 = modelo_ajuste['polo1']
    polo2 = modelo_ajuste['polo2']
    
    if isinstance(polo1, complex):
        ax2.plot(np.real(polo1), np.imag(polo1), 'rx', markersize=20, markeredgewidth=3)
        ax2.plot(np.real(polo2), np.imag(polo2), 'rx', markersize=20, markeredgewidth=3, 
                label='Polos')
        ax2.text(np.real(polo1), np.imag(polo1)+0.5, 
                f's = {polo1:.2f}', fontsize=9, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    else:
        ax2.plot(polo1, 0, 'rx', markersize=20, markeredgewidth=3, label='Polos')
        ax2.plot(polo2, 0, 'rx', markersize=20, markeredgewidth=3)
        ax2.text(polo1, 0.5, f's₁ = {polo1:.2f}', fontsize=9, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        ax2.text(polo2, -0.5, f's₂ = {polo2:.2f}', fontsize=9, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    
    ax2.set_xlabel('Parte Real', fontsize=11)
    ax2.set_ylabel('Parte Imaginaria', fontsize=11)
    ax2.set_title('Diagrama de Polos y Ceros', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_aspect('equal')
    
    # Subplot 3: Error de ajuste
    ax3 = plt.subplot(2, 3, 3)
    error = velocidad - modelo_ajuste['y_modelo']
    ax3.plot(tiempo, error, 'r-', linewidth=1)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax3.fill_between(tiempo, error, 0, alpha=0.3, color='red')
    ax3.set_xlabel('Tiempo (s)', fontsize=11)
    ax3.set_ylabel('Error (RPM)', fontsize=11)
    ax3.set_title(f"Error de Ajuste (RMSE: {modelo_ajuste['rmse']:.2f} RPM)", 
                 fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # Subplot 4-5: Diagrama de Bode
    ax4 = plt.subplot(2, 3, 4)
    ax4.semilogx(analisis_freq['w'], analisis_freq['mag_db'], 'b-', linewidth=2)
    ax4.axhline(y=analisis_freq['mag_db'][0]-3, color='r', 
               linestyle='--', label='-3dB')
    ax4.axvline(x=analisis_freq['BW_rad'], color='r', linestyle='--')
    ax4.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax4.set_ylabel('Magnitud (dB)', fontsize=11)
    ax4.set_title('Diagrama de Bode - Magnitud', fontsize=12, fontweight='bold')
    ax4.grid(True, which='both', alpha=0.3)
    ax4.legend()
    
    # Fase
    ax5 = plt.subplot(2, 3, 5)
    num_bode = [modelo_ajuste['K'] * modelo_ajuste['wn']**2]
    den_bode = [1, 2*modelo_ajuste['zeta']*modelo_ajuste['wn'], modelo_ajuste['wn']**2]
    _, h = signal.freqs(num_bode, den_bode, worN=analisis_freq['w'])
    fase = np.angle(h) * 180/np.pi
    ax5.semilogx(analisis_freq['w'], fase, 'b-', linewidth=2)
    ax5.set_xlabel('Frecuencia (rad/s)', fontsize=11)
    ax5.set_ylabel('Fase (grados)', fontsize=11)
    ax5.set_title('Diagrama de Bode - Fase', fontsize=12, fontweight='bold')
    ax5.grid(True, which='both', alpha=0.3)
    
    # Subplot 6: Tabla de parámetros
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')
    
    tabla_texto = f"""
╔═══════════════════════════════════════╗
║   PARÁMETROS IDENTIFICADOS            ║
╠═══════════════════════════════════════╣
║  Modelo: G(s) = K·ωₙ²/(s²+2ζωₙs+ωₙ²)  ║
║                                       ║
║  K = {modelo_ajuste['K']:.2f} RPM/V                ║
║  ωₙ = {modelo_ajuste['wn']:.2f} rad/s               ║
║  ζ = {modelo_ajuste['zeta']:.3f}                      ║
║                                       ║
║  Overshoot: {modelo_ajuste['overshoot']:.1f}%               ║
║  Ts (2%): {modelo_ajuste['ts']:.3f} s                  ║
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
    
    ax6.text(0.05, 0.5, tabla_texto, fontsize=9, family='monospace',
            verticalalignment='center', fontweight='normal')
    
    plt.tight_layout()
    
    return fig1

def graficar_discretizacion(sys_continuo, Ts):
    """Compara modelo continuo vs discretizado"""
    
    fig2 = plt.figure(figsize=(14, 5))
    fig2.suptitle('DISCRETIZACIÓN DEL MODELO', fontsize=16, fontweight='bold')
    
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
    ax1.set_title('Comparación de Respuesta al Escalón', fontsize=12, 
                 fontweight='bold')
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
    
    # Añadir anotación del polo
    for polo in polos_z:
        ax2.text(np.real(polo), np.imag(polo)+0.15, 
                f'z={polo:.3f}', fontsize=9, ha='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    plt.tight_layout()
    
    return fig2

# ============================================================================
# MAIN - PROGRAMA PRINCIPAL
# ============================================================================
def main():
    """Función principal para ejecutar todo el proceso"""
    
    print("="*70)
    print(" IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC (SEGUNDO ORDEN)")
    print("="*70)
    print(f"\nEspecificaciones del motor:")
    print(f"  Voltaje: {V_escalon} V DC")
    print(f"  Potencia: {Potencia} W")
    print(f"  RPM nominal: {RPM_nominal} RPM")
    print(f"\nParámetros de la planta:")
    print(f"  K = {K_real:.2f} RPM/V")
    print(f"  ωₙ = {wn_real} rad/s")
    print(f"  ζ = {zeta_real}")
    print(f"  Ruido = {ruido_amplitud} RPM")
    
    # 1. Generar datos de simulación
    print("\n[1/5] Generando datos de simulación...")
    tiempo, velocidad, dt = generar_datos_simulacion()
    
    # 2. Identificar modelo por ajuste
    print("\n[2/5] Identificando modelo por ajuste de curva...")
    modelo_ajuste = identificar_modelo_ajuste(tiempo, velocidad, V_escalon)
    
    if modelo_ajuste is None:
        print("❌ Error en la identificación")
        return
    
    print(f"  ✓ K = {modelo_ajuste['K']:.2f} RPM/V")
    print(f"  ✓ ωₙ = {modelo_ajuste['wn']:.2f} rad/s")
    print(f"  ✓ ζ = {modelo_ajuste['zeta']:.3f}")
    print(f"  ✓ Overshoot = {modelo_ajuste['overshoot']:.1f}%")
    print(f"  ✓ Ts (2%) = {modelo_ajuste['ts']:.3f} s")
    print(f"  ✓ Ajuste = {modelo_ajuste['fit']:.1f}%")
    
    # 3. Análisis básico de respuesta
    print("\n[3/5] Analizando respuesta...")
    modelo_63 = identificar_modelo_63(tiempo, velocidad, V_escalon)
    
    print(f"  ✓ K (medido) = {modelo_63['K']:.2f} RPM/V")
    print(f"  ✓ Overshoot (medido) = {modelo_63['overshoot']:.1f}%")
    
    # 4. Analizar frecuencias
    print("\n[4/5] Analizando frecuencias y discretización...")
    analisis = analizar_frecuencias(modelo_ajuste['K'], modelo_ajuste['wn'], modelo_ajuste['zeta'])
    
    print(f"  ✓ Ancho de banda: {analisis['BW_hz']:.2f} Hz")
    print(f"  ✓ Frecuencia recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"  ✓ Período recomendado: {analisis['Ts_recomendado']*1000:.2f} ms")
    
    # 5. Generar gráficas
    print("\n[5/5] Generando gráficas...")
    fig1 = graficar_resultados(tiempo, velocidad, modelo_ajuste, modelo_63, 
                               analisis, V_escalon, dt)
    fig2 = graficar_discretizacion(analisis['sys'], analisis['Ts_recomendado'])
    
    # Resumen final
    print("\n" + "="*70)
    print(" RESUMEN FINAL")
    print("="*70)
    print(f"\nModelo identificado (segundo orden):")
    print(f"  G(s) = {modelo_ajuste['K']:.2f}·{modelo_ajuste['wn']:.2f}² / (s² + {2*modelo_ajuste['zeta']*modelo_ajuste['wn']:.2f}s + {modelo_ajuste['wn']:.2f}²)")
    print(f"\nPolos y Ceros:")
    if isinstance(modelo_ajuste['polo1'], complex):
        print(f"  Polos complejos conjugados:")
        print(f"    s₁ = {modelo_ajuste['polo1']:.3f}")
        print(f"    s₂ = {modelo_ajuste['polo2']:.3f}")
    else:
        print(f"  Polos reales:")
        print(f"    s₁ = {modelo_ajuste['polo1']:.3f}")
        print(f"    s₂ = {modelo_ajuste['polo2']:.3f}")
    print(f"  Ceros: Ninguno")
    print(f"\nCaracterísticas de la respuesta:")
    print(f"  Overshoot: {modelo_ajuste['overshoot']:.1f}%")
    print(f"  Tiempo de asentamiento (2%): {modelo_ajuste['ts']:.3f} s")
    print(f"  Ajuste del modelo: {modelo_ajuste['fit']:.1f}%")
    print(f"\nFrecuencia de muestreo recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"Período de muestreo: {analisis['Ts_recomendado']*1000:.2f} ms")
    print("\n✓ Proceso completado exitosamente!")
    print("\nPresiona Ctrl+C para salir o cierra las ventanas de gráficas.")
    
    plt.show()

if __name__ == "__main__":
    main()