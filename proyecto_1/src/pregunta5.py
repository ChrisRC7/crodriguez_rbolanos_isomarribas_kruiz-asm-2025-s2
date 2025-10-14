"""
Pregunta 5: Implementación de Modulación y Demodulación usando FFT 

Este script implementa un ejemplo completo de modulación y demodulación
usando la Transformada Rápida de Fourier (FFT) para filtrado en el dominio
de la frecuencia.
"""

import numpy as np
import scipy.fft
import scipy.io.wavfile
from matplotlib import pyplot as plt
import os

# ==================== PARÁMETROS DE CONFIGURACIÓN ====================
SAMPLE_RATE = 44100  # Hz - Frecuencia de muestreo
DURATION = 5         # Segundos - Duración de las señales
FREQ_DESEADA = 400   # Hz - Frecuencia de la señal deseada
FREQ_RUIDO = 4000    # Hz - Frecuencia del ruido a filtrar
AMPLITUD_RUIDO = 0.3 # Factor de amplitud del ruido (30%)

# ==================== FUNCIONES AUXILIARES ====================

def generate_sine_wave(freq, sample_rate, duration):
    """
    Genera una onda sinusoidal.
    
    Parámetros:
        freq: Frecuencia de la onda en Hz
        sample_rate: Frecuencia de muestreo en Hz
        duration: Duración de la señal en segundos
    
    Retorna:
        t: Vector de tiempo
        y: Vector de amplitudes de la señal
    """
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    y = np.sin(2 * np.pi * freq * t)
    return t, y


def normalize_audio(signal, target_amplitude=32767):
    """
    Normaliza una señal de audio a 16 bits.
    
    Parámetros:
        signal: Señal a normalizar
        target_amplitude: Amplitud objetivo (32767 para 16 bits)
    
    Retorna:
        Signal normalizada como entero de 16 bits
    """
    normalized = np.int16((signal / np.max(np.abs(signal))) * target_amplitude)
    return normalized


def plot_time_domain(signal, sample_rate, title="Señal en el Dominio del Tiempo", samples=1000):
    """
    Grafica una señal en el dominio del tiempo.
    
    Parámetros:
        signal: Señal a graficar
        sample_rate: Frecuencia de muestreo
        title: Título de la gráfica
        samples: Número de muestras a mostrar
    """
    t = np.arange(samples) / sample_rate
    plt.figure(figsize=(12, 4))
    plt.plot(t, signal[:samples], linewidth=0.5)
    plt.title(title)
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Amplitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def plot_frequency_domain(xf, yf, title="Espectro de Frecuencia"):
    """
    Grafica el espectro de magnitud de una señal.
    
    Parámetros:
        xf: Vector de frecuencias
        yf: Vector de coeficientes FFT (complejos)
        title: Título de la gráfica
    """
    plt.figure(figsize=(12, 4))
    plt.plot(xf, np.abs(yf))
    plt.title(title)
    plt.xlabel("Frecuencia (Hz)")
    plt.ylabel("Magnitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ==================== PROCESO PRINCIPAL ====================

def main():
    print("="*70)
    print("SISTEMA DE MODULACIÓN Y DEMODULACIÓN USANDO FFT")
    print("="*70)
    
    # -------- PASO 1: GENERACIÓN DE SEÑALES --------
    print("\n[PASO 1] Generando señales...")
    print(f"  - Señal deseada: {FREQ_DESEADA} Hz")
    print(f"  - Señal de ruido: {FREQ_RUIDO} Hz (amplitud {AMPLITUD_RUIDO*100}%)")
    
    t, signal_deseada = generate_sine_wave(FREQ_DESEADA, SAMPLE_RATE, DURATION)
    _, signal_ruido = generate_sine_wave(FREQ_RUIDO, SAMPLE_RATE, DURATION)
    
    # -------- PASO 2: MODULACIÓN (MEZCLA DE SEÑALES) --------
    print("\n[PASO 2] Mezclando señales (Modulación)...")
    signal_ruido = signal_ruido * AMPLITUD_RUIDO
    signal_mixta = signal_deseada + signal_ruido
    
    # Normalizar para guardar
    signal_mixta_norm = normalize_audio(signal_mixta)
    
    # Guardar señal mixta
    output_dir = "audio_output"
    os.makedirs(output_dir, exist_ok=True)
    mixed_path = os.path.join(output_dir, "signal_mixta.wav")
    scipy.io.wavfile.write(mixed_path, SAMPLE_RATE, signal_mixta_norm)
    print(f"  ✓ Señal mixta guardada en: {mixed_path}")
    
    # Mostrar señal mixta en el tiempo
    print("\n  Mostrando señal mixta en el dominio del tiempo...")
    plot_time_domain(signal_mixta_norm, SAMPLE_RATE, 
                     "Señal Mixta (400 Hz + 4000 Hz) - Dominio del Tiempo")
    
    # -------- PASO 3: APLICAR FFT --------
    print("\n[PASO 3] Aplicando FFT...")
    N = len(signal_mixta_norm)
    yf = scipy.fft.rfft(signal_mixta_norm)
    xf = scipy.fft.rfftfreq(N, 1 / SAMPLE_RATE)
    
    print(f"  ✓ FFT calculada: {len(yf)} bins de frecuencia")
    print(f"  ✓ Resolución frecuencial: {SAMPLE_RATE/N:.2f} Hz/bin")
    
    # Mostrar espectro antes del filtrado
    print("\n  Mostrando espectro ANTES del filtrado...")
    plot_frequency_domain(xf, yf, "Espectro de Frecuencia - ANTES del Filtrado")
    
    # -------- PASO 4: FILTRADO (DEMODULACIÓN) --------
    print("\n[PASO 4] Filtrando frecuencia de ruido...")
    
    # Calcular el índice correspondiente a la frecuencia de ruido
    points_per_freq = len(xf) / (SAMPLE_RATE / 2)
    target_idx = int(points_per_freq * FREQ_RUIDO)
    
    print(f"  - Índice objetivo en el espectro: {target_idx}")
    print(f"  - Frecuencia exacta del bin: {xf[target_idx]:.2f} Hz")
    
    # Guardar copia del espectro original para comparación
    yf_filtrado = yf.copy()
    
    # Anular la componente de ruido (bin objetivo ± 1)
    yf_filtrado[target_idx - 1 : target_idx + 2] = 0
    
    print(f"  ✓ Componentes de frecuencia anuladas en bins {target_idx-1} a {target_idx+1}")
    
    # Mostrar espectro después del filtrado
    print("\n  Mostrando espectro DESPUÉS del filtrado...")
    plot_frequency_domain(xf, yf_filtrado, "Espectro de Frecuencia - DESPUÉS del Filtrado")
    
    # -------- PASO 5: APLICAR FFT INVERSA --------
    print("\n[PASO 5] Aplicando FFT Inversa (iFFT)...")
    signal_limpia = scipy.fft.irfft(yf_filtrado)
    
    print(f"  ✓ Señal reconstruida: {len(signal_limpia)} muestras")
    
    # Normalizar señal limpia
    signal_limpia_norm = normalize_audio(signal_limpia)
    
    # Guardar señal limpia
    clean_path = os.path.join(output_dir, "signal_limpia.wav")
    scipy.io.wavfile.write(clean_path, SAMPLE_RATE, signal_limpia_norm)
    print(f"  ✓ Señal limpia guardada en: {clean_path}")
    
    # Mostrar señal limpia en el tiempo
    print("\n  Mostrando señal limpia en el dominio del tiempo...")
    plot_time_domain(signal_limpia_norm, SAMPLE_RATE,
                     "Señal Limpia (Demodulada) - Dominio del Tiempo")
    
    # -------- PASO 6: COMPARACIÓN FINAL --------
    print("\n[PASO 6] Comparación de señales...")
    
    # Crear comparación visual
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    samples = 1000
    t = np.arange(samples) / SAMPLE_RATE
    
    # Señal original deseada
    axes[0].plot(t, normalize_audio(signal_deseada)[:samples], 'g-', linewidth=0.5)
    axes[0].set_title("Señal Original Deseada (400 Hz)")
    axes[0].set_ylabel("Amplitud")
    axes[0].grid(True, alpha=0.3)
    
    # Señal mixta
    axes[1].plot(t, signal_mixta_norm[:samples], 'r-', linewidth=0.5)
    axes[1].set_title("Señal Mixta (400 Hz + 4000 Hz)")
    axes[1].set_ylabel("Amplitud")
    axes[1].grid(True, alpha=0.3)
    
    # Señal recuperada
    axes[2].plot(t, signal_limpia_norm[:samples], 'b-', linewidth=0.5)
    axes[2].set_title("Señal Recuperada (Demodulada)")
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].set_ylabel("Amplitud")
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # -------- RESUMEN FINAL --------
    print("\n" + "="*70)
    print("RESUMEN DE RESULTADOS")
    print("="*70)
    print(f"✓ Señal deseada generada: {FREQ_DESEADA} Hz")
    print(f"✓ Ruido agregado: {FREQ_RUIDO} Hz")
    print(f"✓ Filtrado exitoso usando FFT")
    print(f"✓ Señal recuperada mediante iFFT")
    print(f"\nArchivos generados en '{output_dir}/':")
    print(f"  - signal_mixta.wav (señal modulada)")
    print(f"  - signal_limpia.wav (señal demodulada)")
    print("\n¡Proceso completado exitosamente!")
    print("="*70)


if __name__ == "__main__":
    main()