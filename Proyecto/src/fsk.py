import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile
import os

# Crear carpeta fmprueba si no existe
output_folder = 'fmprueba'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    print(f"Carpeta '{output_folder}' creada.")

# Parámetros de la simulación
fs = 44100  # Frecuencia de muestreo (Hz) - estándar de audio
t_duration = 3.0  # Duración de la señal (segundos)
t = np.linspace(0, t_duration, int(fs * t_duration))

# Señal mensaje (senoidal)
fm = 440  # Frecuencia del mensaje (Hz) - nota La
Am = 1  # Amplitud del mensaje
m_t = Am * np.sin(2 * np.pi * fm * t)

# Parámetros de modulación FM optimizados
fc = 5000  # Frecuencia portadora (Hz)
kf = 300  # Índice de sensibilidad de frecuencia reducido (Hz/V)
beta = kf * Am / fm  # Índice de modulación

# Modulación FM
# La señal FM se define como: s(t) = Ac * cos(2*pi*fc*t + 2*pi*kf*integral(m(t)))
integral_m = np.cumsum(m_t) / fs  # Integral numérica
s_t = np.cos(2 * np.pi * fc * t + 2 * np.pi * kf * integral_m)

# Guardar archivos de audio
def guardar_audio(senal, nombre_archivo, fs):
    # Normalizar a rango de 16 bits
    senal_norm = np.int16(senal / np.max(np.abs(senal)) * 32767)
    ruta_completa = os.path.join(output_folder, nombre_archivo)
    wavfile.write(ruta_completa, fs, senal_norm)
    print(f"Audio guardado: {ruta_completa}")

# Guardar señal original
guardar_audio(m_t, 'señal_original.wav', fs)

# Guardar señal modulada
guardar_audio(s_t, 'señal_modulada_fm.wav', fs)

# FFT de las señales
def calcular_fft(senal, fs):
    N = len(senal)
    fft_senal = np.fft.fft(senal)
    fft_freq = np.fft.fftfreq(N, 1/fs)
    
    # Solo mitad positiva del espectro
    mitad = N // 2
    fft_senal = fft_senal[:mitad]
    fft_freq = fft_freq[:mitad]
    
    magnitud = np.abs(fft_senal) / N * 2  # Normalizar
    fase = np.angle(fft_senal)
    
    return fft_freq, magnitud, fase

# Calcular FFT
freq_m, mag_m, fase_m = calcular_fft(m_t, fs)
freq_s, mag_s, fase_s = calcular_fft(s_t, fs)

# ============================================================
# DEMODULACIÓN FM MEJORADA
# ============================================================
# Método mejorado: Discriminador de frecuencia con filtrado óptimo

# Paso 1: Derivada (discriminador de frecuencia)
s_t_diff = np.gradient(s_t, 1/fs)  # Derivada mejorada usando gradient

# Paso 2: Detector de envolvente usando transformada de Hilbert
s_analytic = signal.hilbert(s_t_diff)
s_envelope = np.abs(s_analytic)

# Paso 3: Filtro paso bajo con FASE CERO
orden_filtro = 8
freq_corte = fm * 2.5
b, a = signal.butter(orden_filtro, freq_corte, btype='low', fs=fs)

# ✅ CAMBIO CLAVE: usar filtfilt en lugar de filter
m_demod = signal.filtfilt(b, a, s_envelope)  # <- fase cero automática

# Paso 4: Remover DC offset
m_demod = m_demod - np.mean(m_demod)

# Paso 5: Compensación de fase avanzada (alineación temporal)
# Encontrar el desfase por correlación cruzada
correlation = signal.correlate(m_t, m_demod, mode='same')
lag = np.argmax(correlation) - len(m_t) // 2

print("\n" + "="*70)
print("MENSAJES DE DEPURACIÓN - ANÁLISIS DE FASE")
print("="*70)
print(f"[DEBUG] Lag detectado por correlación cruzada: {lag} muestras")
print(f"[DEBUG] Lag en tiempo: {lag/fs*1000:.4f} ms")

# Aplicar corrección de fase
if abs(lag) < len(m_t) // 10:  # Solo si el desfase es razonable
    m_demod = np.roll(m_demod, -lag)
    # Rellenar bordes con extrapolación
    if lag > 0:
        m_demod[-lag:] = m_demod[-lag-10:-lag].mean()
    elif lag < 0:
        m_demod[:-lag] = m_demod[-lag:-lag+10].mean()
    print(f"[DEBUG] Corrección temporal aplicada: {lag} muestras desplazadas")
else:
    print(f"[DEBUG] Lag demasiado grande, no se aplicó corrección temporal")

# Paso 6: Análisis de fase en el dominio de la frecuencia
# Calcular FFT de ambas señales ANTES de corrección
fft_original = np.fft.fft(m_t)
fft_demod_antes = np.fft.fft(m_demod)
freq_fft = np.fft.fftfreq(len(m_t), 1/fs)

# Encontrar el índice de la frecuencia fundamental (440 Hz)
idx_fundamental = np.argmin(np.abs(freq_fft[:len(freq_fft)//2] - fm))
freq_encontrada = freq_fft[idx_fundamental]

print(f"\n[DEBUG] Frecuencia objetivo: {fm} Hz")
print(f"[DEBUG] Frecuencia encontrada en FFT: {freq_encontrada:.2f} Hz")
print(f"[DEBUG] Índice del fundamental: {idx_fundamental}")

# Calcular fase en la componente fundamental
fase_original = np.angle(fft_original[idx_fundamental])
fase_demod_antes = np.angle(fft_demod_antes[idx_fundamental])
diferencia_fase = fase_original - fase_demod_antes

print(f"\n[DEBUG] Fase señal original (440 Hz): {np.degrees(fase_original):.2f}°")
print(f"[DEBUG] Fase señal demod ANTES (440 Hz): {np.degrees(fase_demod_antes):.2f}°")
print(f"[DEBUG] Diferencia de fase: {np.degrees(diferencia_fase):.2f}°")

# Aplicar corrección de fase rotando en frecuencia
# Método: multiplicar FFT por exp(j*diferencia_fase) en todas las frecuencias
fft_demod_corregida = fft_demod_antes * np.exp(1j * diferencia_fase)
m_demod_corregida = np.real(np.fft.ifft(fft_demod_corregida))

# Verificar la corrección
fase_demod_despues = np.angle(fft_demod_corregida[idx_fundamental])
print(f"[DEBUG] Fase señal demod DESPUÉS (440 Hz): {np.degrees(fase_demod_despues):.2f}°")
print(f"[DEBUG] Error residual de fase: {np.degrees(fase_demod_despues - fase_original):.4f}°")

# Usar la señal corregida
m_demod = m_demod_corregida

# Normalizar después de la corrección
m_demod = m_demod / np.max(np.abs(m_demod)) * Am
print(f"[DEBUG] Normalización aplicada (factor: {np.max(np.abs(m_demod_corregida))/Am:.4f})")
print("="*70 + "\n")

# Guardar señal demodulada mejorada
guardar_audio(m_demod, 'señal_demodulada.wav', fs)

# FFT de la señal demodulada
freq_demod, mag_demod, fase_demod = calcular_fft(m_demod, fs)

# Crear gráficas mejoradas
fig = plt.figure(figsize=(18, 13))
fig.suptitle('ANÁLISIS COMPLETO DE MODULACIÓN Y DEMODULACIÓN FM', 
             fontsize=16, fontweight='bold', y=0.995)

# Calcular índices para mostrar solo una porción en tiempo
samples_mostrar = int(0.01 * fs)  # Mostrar 10ms

# ============== FILA 1: SEÑAL ORIGINAL ==============
# Señal mensaje original - Tiempo
ax1 = plt.subplot(4, 3, 1)
plt.plot(t[:samples_mostrar], m_t[:samples_mostrar], 'b', linewidth=2)
plt.title('Señal Mensaje Original (Tiempo)', fontsize=11, fontweight='bold')
plt.xlabel('Tiempo (s)', fontsize=9)
plt.ylabel('Amplitud', fontsize=9)
plt.grid(True, alpha=0.3)

# Señal mensaje - Magnitud FFT
ax2 = plt.subplot(4, 3, 2)
plt.plot(freq_m, mag_m, 'b', linewidth=1.5)
plt.title('Señal Mensaje - Magnitud FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Magnitud', fontsize=9)
plt.xlim([0, 2000])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.5, label=f'{fm} Hz')
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Señal mensaje - Fase FFT
ax3 = plt.subplot(4, 3, 3)
plt.plot(freq_m, fase_m, 'b', linewidth=1.5)
plt.title('Señal Mensaje - Fase FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Fase (rad)', fontsize=9)
plt.xlim([0, 2000])
plt.grid(True, alpha=0.3)

# ============== FILA 2: SEÑAL MODULADA ==============
# Señal modulada FM - Tiempo
ax4 = plt.subplot(4, 3, 4)
plt.plot(t[:samples_mostrar], s_t[:samples_mostrar], 'r', linewidth=1)
plt.title('Señal Modulada FM (Tiempo)', fontsize=11, fontweight='bold')
plt.xlabel('Tiempo (s)', fontsize=9)
plt.ylabel('Amplitud', fontsize=9)
plt.grid(True, alpha=0.3)

# Señal modulada - Magnitud FFT
ax5 = plt.subplot(4, 3, 5)
plt.plot(freq_s, mag_s, 'r', linewidth=1)
plt.title('Señal Modulada FM - Magnitud FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Magnitud', fontsize=9)
plt.xlim([fc - 1500, fc + 1500])
plt.axvline(x=fc, color='b', linestyle='--', alpha=0.5, label=f'Portadora: {fc} Hz')
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Señal modulada - Fase FFT
ax6 = plt.subplot(4, 3, 6)
plt.plot(freq_s, fase_s, 'r', linewidth=1)
plt.title('Señal Modulada FM - Fase FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Fase (rad)', fontsize=9)
plt.xlim([fc - 1500, fc + 1500])
plt.grid(True, alpha=0.3)

# ============== FILA 3: SEÑAL DEMODULADA ==============
# Señal demodulada - Tiempo
ax7 = plt.subplot(4, 3, 7)
plt.plot(t[:samples_mostrar], m_t[:samples_mostrar], 'b--', linewidth=2, 
         alpha=0.6, label='Original')
plt.plot(t[:samples_mostrar], m_demod[:samples_mostrar], 'g', linewidth=1.5, 
         label='Demodulada')
plt.title('Señal Demodulada (Tiempo)', fontsize=11, fontweight='bold')
plt.xlabel('Tiempo (s)', fontsize=9)
plt.ylabel('Amplitud', fontsize=9)
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Señal demodulada - Magnitud FFT
ax8 = plt.subplot(4, 3, 8)
plt.plot(freq_m, mag_m, 'b--', linewidth=2, alpha=0.6, label='Original')
plt.plot(freq_demod, mag_demod, 'g', linewidth=1.5, label='Demodulada')
plt.title('Señal Demodulada - Magnitud FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Magnitud', fontsize=9)
plt.xlim([0, 2000])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.3)
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Señal demodulada - Fase FFT
ax9 = plt.subplot(4, 3, 9)
plt.plot(freq_m, fase_m, 'b--', linewidth=2, alpha=0.6, label='Original')
plt.plot(freq_demod, fase_demod, 'g', linewidth=1.5, label='Demodulada')
plt.title('Señal Demodulada - Fase FFT', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Fase (rad)', fontsize=9)
plt.xlim([0, 2000])
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# ============== FILA 4: ANÁLISIS Y COMPARACIÓN ==============
# Comparación superpuesta - Tiempo (más muestras)
ax10 = plt.subplot(4, 3, 10)
samples_comp = int(0.02 * fs)  # 20ms
plt.plot(t[:samples_comp], m_t[:samples_comp], 'b', linewidth=2.5, 
         label='Original', alpha=0.7)
plt.plot(t[:samples_comp], m_demod[:samples_comp], 'g--', linewidth=2, 
         label='Demodulada')
plt.title('Comparación: Original vs Demodulada', fontsize=11, fontweight='bold')
plt.xlabel('Tiempo (s)', fontsize=9)
plt.ylabel('Amplitud', fontsize=9)
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Espectro completo de señal modulada
ax11 = plt.subplot(4, 3, 11)
plt.plot(freq_s, mag_s, 'r', linewidth=1)
plt.title('Espectro Completo FM', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)', fontsize=9)
plt.ylabel('Magnitud', fontsize=9)
plt.xlim([0, 10000])
plt.axvline(x=fc, color='b', linestyle='--', alpha=0.3, label='Portadora')
plt.axvspan(fc - kf*Am, fc + kf*Am, alpha=0.2, color='yellow', 
            label='Desviación ±' + str(int(kf*Am)) + ' Hz')
plt.legend(fontsize=8)
plt.grid(True, alpha=0.3)

# Información de parámetros y calidad
ax12 = plt.subplot(4, 3, 12)
ax12.axis('off')

# Calcular métricas de calidad
m_t_norm = m_t / np.max(np.abs(m_t))
m_demod_norm = m_demod / np.max(np.abs(m_demod))
error_rms = np.sqrt(np.mean((m_t_norm - m_demod_norm)**2))

# SNR (Signal-to-Noise Ratio)
signal_power = np.mean(m_t_norm**2)
noise_power = np.mean((m_t_norm - m_demod_norm)**2)
snr_db = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else float('inf')

# Correlación
correlation_coef = np.corrcoef(m_t_norm, m_demod_norm)[0, 1]

info_text = f"""PARÁMETROS DE MODULACIÓN FM

━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Señal Mensaje:
  • Frecuencia: {fm} Hz (nota La)
  • Amplitud: {Am} V

Portadora:
  • Frecuencia: {fc} Hz

Modulación:
  • Sensibilidad (kf): {kf} Hz/V
  • Índice modulación (β): {beta:.3f}
  • Desviación freq.: ±{kf * Am} Hz
  • Ancho banda (Carson): 
    {2 * (kf * Am + fm):.1f} Hz

Demodulación:
  • Filtro: Butterworth orden {orden_filtro}
  • Freq. corte: {freq_corte:.1f} Hz
  • Compensación de fase: Activa

━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MÉTRICAS DE CALIDAD:
  • Error RMS: {error_rms:.6f}
  • SNR: {snr_db:.2f} dB
  • Correlación: {correlation_coef:.6f}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Archivos en '{output_folder}/':
  ✓ señal_original.wav
  ✓ señal_modulada_fm.wav
  ✓ señal_demodulada.wav
  ✓ graficas_fm.png
"""
ax12.text(0.05, 0.5, info_text, fontsize=8.5, family='monospace',
          verticalalignment='center',
          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

plt.tight_layout(rect=[0, 0, 1, 0.99])

# Guardar gráfica principal
ruta_grafica = os.path.join(output_folder, 'graficas_fm.png')
plt.savefig(ruta_grafica, dpi=200, bbox_inches='tight')
print(f"Gráfica guardada: {ruta_grafica}")

# ============================================================
# VENTANA ADICIONAL: ANÁLISIS DETALLADO DE FASE
# ============================================================
fig2 = plt.figure(figsize=(16, 10))
fig2.suptitle('ANÁLISIS DETALLADO DE FASE - DEPURACIÓN', 
              fontsize=14, fontweight='bold')

# Gráfica 1: Fase completa en rango amplio
ax1 = plt.subplot(3, 2, 1)
plt.plot(freq_m, fase_m, 'b-', linewidth=2, label='Original', alpha=0.7)
plt.plot(freq_demod, fase_demod, 'g-', linewidth=2, label='Demodulada', alpha=0.7)
plt.title('Fase FFT - Rango Completo (0-2000 Hz)', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Fase (radianes)')
plt.xlim([0, 2000])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.5, label=f'{fm} Hz')
plt.legend()
plt.grid(True, alpha=0.3)

# Gráfica 2: Zoom en frecuencia fundamental ±50 Hz
ax2 = plt.subplot(3, 2, 2)
plt.plot(freq_m, fase_m, 'b-', linewidth=3, label='Original', alpha=0.7)
plt.plot(freq_demod, fase_demod, 'g-', linewidth=3, label='Demodulada', alpha=0.7)
plt.title(f'Fase FFT - Zoom en Fundamental ({fm} Hz ± 50 Hz)', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Fase (radianes)')
plt.xlim([fm - 50, fm + 50])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.5, linewidth=2)
plt.legend()
plt.grid(True, alpha=0.3)

# Gráfica 3: Diferencia de fase
ax3 = plt.subplot(3, 2, 3)
# Calcular diferencia de fase punto a punto
fase_diff = np.zeros_like(fase_m)
for i in range(len(fase_m)):
    fase_diff[i] = np.angle(np.exp(1j * (fase_m[i] - fase_demod[i])))
plt.plot(freq_m, np.degrees(fase_diff), 'r-', linewidth=1.5)
plt.title('Diferencia de Fase (Original - Demodulada)', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Diferencia de Fase (grados)')
plt.xlim([0, 2000])
plt.axvline(x=fm, color='b', linestyle='--', alpha=0.5)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.grid(True, alpha=0.3)

# Gráfica 4: Magnitud para referencia
ax4 = plt.subplot(3, 2, 4)
plt.plot(freq_m, mag_m, 'b-', linewidth=2, label='Original', alpha=0.7)
plt.plot(freq_demod, mag_demod, 'g--', linewidth=2, label='Demodulada', alpha=0.7)
plt.title('Magnitud FFT para Referencia', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Magnitud')
plt.xlim([0, 2000])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.5)
plt.legend()
plt.grid(True, alpha=0.3)
plt.yscale('log')

# Gráfica 5: Fase unwrapped (desenvolvida)
ax5 = plt.subplot(3, 2, 5)
fase_m_unwrap = np.unwrap(fase_m)
fase_demod_unwrap = np.unwrap(fase_demod)
plt.plot(freq_m, fase_m_unwrap, 'b-', linewidth=2, label='Original', alpha=0.7)
plt.plot(freq_demod, fase_demod_unwrap, 'g-', linewidth=2, label='Demodulada', alpha=0.7)
plt.title('Fase FFT Unwrapped (desenvolvida)', fontsize=11, fontweight='bold')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Fase (radianes)')
plt.xlim([0, 2000])
plt.axvline(x=fm, color='r', linestyle='--', alpha=0.5)
plt.legend()
plt.grid(True, alpha=0.3)

# Gráfica 6: Tabla de valores en frecuencia fundamental
ax6 = plt.subplot(3, 2, 6)
ax6.axis('off')

# Buscar valores exactos en fm
idx_fm = np.argmin(np.abs(freq_m - fm))
fase_orig_fm = fase_m[idx_fm]
fase_demod_fm = fase_demod[idx_fm]
mag_orig_fm = mag_m[idx_fm]
mag_demod_fm = mag_demod[idx_fm]

debug_text = f"""VALORES EN FRECUENCIA FUNDAMENTAL ({fm} Hz)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ORIGINAL:
  Frecuencia: {freq_m[idx_fm]:.2f} Hz
  Magnitud:   {mag_orig_fm:.6f}
  Fase:       {fase_orig_fm:.6f} rad
  Fase:       {np.degrees(fase_orig_fm):.2f}°

DEMODULADA:
  Frecuencia: {freq_demod[idx_fm]:.2f} Hz
  Magnitud:   {mag_demod_fm:.6f}
  Fase:       {fase_demod_fm:.6f} rad
  Fase:       {np.degrees(fase_demod_fm):.2f}°

DIFERENCIAS:
  Δ Magnitud: {mag_orig_fm - mag_demod_fm:.6f}
  Δ Fase:     {fase_orig_fm - fase_demod_fm:.6f} rad
  Δ Fase:     {np.degrees(fase_orig_fm - fase_demod_fm):.2f}°
  
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PARÁMETROS DE CORRECCIÓN:
  Lag temporal:     {lag} muestras
  Lag en tiempo:    {lag/fs*1000:.4f} ms
  Corrección fase:  {np.degrees(diferencia_fase):.2f}°

NOTA: Si la diferencia de fase es grande,
el problema puede estar en:
  • Filtro introduce retardo de grupo
  • Método de demodulación afecta fase
  • Necesita corrección adicional
"""

ax6.text(0.05, 0.5, debug_text, fontsize=9, family='monospace',
         verticalalignment='center',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout(rect=[0, 0, 1, 0.97])

# Guardar gráfica de fase
ruta_grafica_fase = os.path.join(output_folder, 'analisis_fase_detallado.png')
plt.savefig(ruta_grafica_fase, dpi=200, bbox_inches='tight')
print(f"Gráfica de fase guardada: {ruta_grafica_fase}")

plt.show()

# Imprimir métricas de comparación
print("=" * 70)
print("RESULTADOS DE LA DEMODULACIÓN FM MEJORADA")
print("=" * 70)
print(f"Frecuencia del mensaje original: {fm} Hz")
print(f"Índice de modulación (β): {beta:.3f}")
print(f"Desviación de frecuencia: ±{kf * Am} Hz")
print(f"Ancho de banda teórico (Carson): {2 * (kf * Am + fm):.1f} Hz")
print(f"\n{'MÉTRICAS DE CALIDAD':^70}")
print("-" * 70)
print(f"{'Error RMS:':<30} {error_rms:.6f}")
print(f"{'SNR (Signal-to-Noise Ratio):':<30} {snr_db:.2f} dB")
print(f"{'Coeficiente de Correlación:':<30} {correlation_coef:.6f}")
print(f"{'Calidad de recuperación:':<30} {(1-error_rms)*100:.2f}%")
print("=" * 70)
print(f"\nMEJORAS IMPLEMENTADAS:")
print(f"  ✓ Índice de modulación reducido a {beta:.3f} (menos distorsión)")
print(f"  ✓ Filtro Butterworth de orden {orden_filtro} (mejor selectividad)")
print(f"  ✓ Compensación de fase temporal (lag: {lag} muestras)")
print(f"  ✓ Corrección de fase en frecuencia (Δφ: {np.degrees(diferencia_fase):.2f}°)")
print(f"  ✓ Derivada mejorada con np.gradient")
print(f"  ✓ Normalización optimizada")
print("=" * 70)
print(f"\nTodos los archivos guardados en: {output_folder}/")
print(f"  • 3 archivos de audio (.wav) - {fs} Hz, {t_duration}s")
print(f"  • 1 gráfica de alta resolución (.png) - 200 DPI")