"""
Pregunta 5: Modulación y Demodulación FSK usando FFT
Autor: Grupo de Trabajo
Fecha: 2025

Este script implementa un sistema completo de modulación FSK (Frequency Shift Keying)
y demodulación usando la clase FFT para recuperación de datos digitales.
"""

import numpy as np
import scipy.io.wavfile
from matplotlib import pyplot as plt
import tkinter as tk
from tkinter import simpledialog
import os
import fft

# ==================== PARÁMETROS DE CONFIGURACIÓN ====================
SAMPLE_RATE = 44100
FREQ_BIT_0 = 1000
FREQ_BIT_1 = 2000
SYMBOL_DURATION = 0.1

# ==================== FUNCIONES ====================

def generate_fsk_symbol(freq, duration, sample_rate):
    """Genera un símbolo FSK (un bit modulado)."""
    num_samples = int(sample_rate * duration)
    t = np.linspace(0, duration, num_samples, endpoint=False)
    symbol = np.sin(2 * np.pi * freq * t)
    return symbol


def modulate_fsk(data_bits, freq_0, freq_1, symbol_duration, sample_rate):
    """Modula bits usando FSK."""
    modulated_signal = np.array([])
    for bit in data_bits:
        freq = freq_1 if bit == 1 else freq_0
        symbol = generate_fsk_symbol(freq, symbol_duration, sample_rate)
        modulated_signal = np.concatenate([modulated_signal, symbol])
    return modulated_signal


def add_awgn_noise(signal, snr_db):
    """Agrega ruido Gaussiano blanco aditivo (AWGN)."""
    signal_power = np.mean(signal ** 2)
    snr_linear = 10 ** (snr_db / 10)
    noise_power = signal_power / snr_linear
    noise = np.random.normal(0, np.sqrt(noise_power), len(signal))
    signal_with_noise = signal + noise
    return signal_with_noise, noise


def demodulate_fsk_with_confidence(signal, freq_0, freq_1, symbol_duration, sample_rate):
    """Demodula una señal FSK usando FFT, retornando bits y confianza."""
    num_samples_per_symbol = int(sample_rate * symbol_duration)
    num_symbols = len(signal) // num_samples_per_symbol
    demodulated_bits = []
    confidence = []
    
    print(f"\n  Demodulando {num_symbols} símbolos...")
    
    for i in range(num_symbols):
        start_idx = i * num_samples_per_symbol
        end_idx = start_idx + num_samples_per_symbol
        symbol = signal[start_idx:end_idx]
        
        fft_analyzer = fft.FFT(symbol, sample_rate, mostrar_graficas=False)
        xf, yf = fft_analyzer.get_spectrum()
        magnitude = np.abs(yf)
        
        points_per_freq = len(xf) / (sample_rate / 2)
        idx_0 = int(points_per_freq * freq_0)
        idx_1 = int(points_per_freq * freq_1)
        
        mag_0 = np.sum(magnitude[max(0, idx_0-2):min(len(magnitude), idx_0+3)])
        mag_1 = np.sum(magnitude[max(0, idx_1-2):min(len(magnitude), idx_1+3)])
        
        bit = 1 if mag_1 > mag_0 else 0
        conf = max(mag_0, mag_1) / (mag_0 + mag_1 + 1e-10)
        
        demodulated_bits.append(bit)
        confidence.append(conf)
        
        if (i + 1) % max(1, num_symbols // 5) == 0:
            print(f"    {i + 1}/{num_symbols} símbolos procesados")
    
    return np.array(demodulated_bits), np.array(confidence)


def demodulated_bits_to_signal(bits, freq_0, freq_1, symbol_duration, sample_rate):
    """Convierte bits demodulados a señal de audio."""
    signal = np.array([])
    for bit in bits:
        freq = freq_1 if bit == 1 else freq_0
        symbol = generate_fsk_symbol(freq, symbol_duration, sample_rate)
        signal = np.concatenate([signal, symbol])
    return signal


def normalize_audio(signal, target_amplitude=32767):
    """Normaliza una señal a 16 bits."""
    if np.max(np.abs(signal)) == 0:
        return np.int16(signal)
    normalized = np.int16((signal / np.max(np.abs(signal))) * target_amplitude)
    return normalized


def solicitar_snr():
    """Solicita la SNR al usuario."""
    root = tk.Tk()
    root.withdraw()
    
    snr = simpledialog.askfloat(
        "Relación Señal-Ruido (SNR)",
        "Ingresa la SNR en dB:\n\n"
        "Ejemplos:\n"
        "  • 20 dB = Excelente\n"
        "  • 10 dB = Bueno\n"
        "  • 5 dB = Moderado\n"
        "  • 0 dB = Mucho ruido",
        initialvalue=10.0,
        minvalue=-10.0,
        maxvalue=30.0
    )
    
    root.destroy()
    return snr


def solicitar_numero_bits():
    """Solicita el número de bits al usuario."""
    root = tk.Tk()
    root.withdraw()
    
    num_bits = simpledialog.askinteger(
        "Número de Bits",
        "¿Cuántos bits deseas modular?",
        initialvalue=20,
        minvalue=1,
        maxvalue=1000
    )
    
    root.destroy()
    return num_bits


# ==================== MAIN ====================

def main():
    print("="*70)
    print("MODULACIÓN Y DEMODULACIÓN FSK (FREQUENCY SHIFT KEYING)")
    print("="*70)
    
    # Solicitar número de bits
    num_bits = solicitar_numero_bits()
    if num_bits is None:
        print("\nOperación cancelada.")
        return
    
    print(f"\n[CONFIGURACIÓN]")
    print(f"  - Número de bits: {num_bits}")
    print(f"  - Frecuencia bit 0: {FREQ_BIT_0} Hz")
    print(f"  - Frecuencia bit 1: {FREQ_BIT_1} Hz")
    print(f"  - Duración por símbolo: {SYMBOL_DURATION} s")
    print(f"  - Duración total: {num_bits * SYMBOL_DURATION:.2f} s")
    
    # PASO 1: Generar bits
    print("\n[PASO 1] Generando bits aleatorios...")
    data_bits = np.random.randint(0, 2, size=num_bits)
    bits_str = ''.join(map(str, data_bits))
    print(f"  Bits: {bits_str}")
    
    # PASO 2: Modular
    print("\n[PASO 2] Modulando con FSK...")
    modulated_signal = modulate_fsk(data_bits, FREQ_BIT_0, FREQ_BIT_1, SYMBOL_DURATION, SAMPLE_RATE)
    print(f"  Señal modulada: {len(modulated_signal)} muestras")
    
    # Graficar señal modulada limpia
    print("\n  Mostrando señal modulada (limpia)...")
    t_mod = np.arange(len(modulated_signal)) / SAMPLE_RATE
    plt.figure(figsize=(14, 4))
    plt.plot(t_mod[:int(SAMPLE_RATE*0.5)], modulated_signal[:int(SAMPLE_RATE*0.5)], linewidth=0.7, color='#2E86AB')
    plt.title("Señal Modulada FSK (Limpia)", fontsize=14, fontweight='bold')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Amplitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
    # Guardar señal limpia
    output_dir = "audio_output"
    os.makedirs(output_dir, exist_ok=True)
    modulated_norm = normalize_audio(modulated_signal)
    modulated_path = os.path.join(output_dir, "fsk_modulated_clean.wav")
    scipy.io.wavfile.write(modulated_path, SAMPLE_RATE, modulated_norm)
    print(f"  Guardada: {modulated_path}")
    
    # PASO 2.5: Agregar ruido
    print("\n[PASO 2.5] Configurando canal de transmisión...")
    snr_db = solicitar_snr()
    if snr_db is None:
        print("Operación cancelada.")
        return
    
    print(f"\n  Agregando ruido AWGN...")
    modulated_noisy, noise = add_awgn_noise(modulated_signal, snr_db)
    print(f" SNR: {snr_db} dB")
    
    # Graficar señal con ruido
    print("\n  Mostrando señal modulada (con ruido)...")
    plt.figure(figsize=(14, 4))
    plt.plot(t_mod[:int(SAMPLE_RATE*0.5)], modulated_noisy[:int(SAMPLE_RATE*0.5)], linewidth=0.7, color='#E63946')
    plt.title(f"Señal Modulada FSK + Ruido ({snr_db} dB SNR)", fontsize=14, fontweight='bold')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Amplitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
    # Guardar señal con ruido
    modulated_noisy_norm = normalize_audio(modulated_noisy)
    modulated_noisy_path = os.path.join(output_dir, "fsk_modulated_noisy.wav")
    scipy.io.wavfile.write(modulated_noisy_path, SAMPLE_RATE, modulated_noisy_norm)
    print(f"  Guardada: {modulated_noisy_path}")
    
    # PASO 3: FFT
    print("\n[PASO 3] Analizando espectro con FFT...")
    fft_analyzer = fft.FFT(modulated_noisy_norm, SAMPLE_RATE, mostrar_graficas=False)
    print(f"  FFT calculada")
    
    print("\n  Mostrando espectro...")
    fft_analyzer.mostrar_espectro()
    
    # PASO 4: Demodular
    print("\n[PASO 4] Demodulando...")
    demodulated_bits, confidence = demodulate_fsk_with_confidence(modulated_noisy, FREQ_BIT_0, FREQ_BIT_1, SYMBOL_DURATION, SAMPLE_RATE)
    demod_str = ''.join(map(str, demodulated_bits))
    print(f"  Bits recuperados: {demod_str}")
    
    # PASO 4.5: Reconstruir señal
    print("\n[PASO 4.5] Sintetizando señal desde bits...")
    demodulated_signal = demodulated_bits_to_signal(demodulated_bits, FREQ_BIT_0, FREQ_BIT_1, SYMBOL_DURATION, SAMPLE_RATE)
    print(f"  Señal sintetizada: {len(demodulated_signal)} muestras")
    
    # Graficar señal demodulada
    print("\n  Mostrando señal demodulada...")
    plt.figure(figsize=(14, 4))
    plt.plot(t_mod[:int(SAMPLE_RATE*0.5)], demodulated_signal[:int(SAMPLE_RATE*0.5)], linewidth=0.7, color='#A23B72')
    plt.title("Señal Demodulada (Reconstruida)", fontsize=14, fontweight='bold')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Amplitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
    # Guardar señal demodulada
    demodulated_norm = normalize_audio(demodulated_signal)
    demodulated_path = os.path.join(output_dir, "fsk_demodulated.wav")
    scipy.io.wavfile.write(demodulated_path, SAMPLE_RATE, demodulated_norm)
    print(f"  Guardada: {demodulated_path}")
    
    # PASO 5: Análisis de errores
    print("\n[PASO 5] Análisis de Desempeño...")
    errors = np.sum(data_bits != demodulated_bits)
    ber = errors / num_bits
    bit_errors = (data_bits != demodulated_bits).astype(int)
    
    print(f"  - Bits originales:    {bits_str}")
    print(f"  - Bits demodulados:   {demod_str}")
    print(f"  - Errores: {errors}/{num_bits}")
    print(f"  - BER: {ber*100:.2f}%")
    print(f"  - Confiabilidad: {(1-ber)*100:.2f}%")
    
    # Comparación de señales
    print("\n[COMPARACIÓN] Mostrando las 3 señales...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    samples = int(SAMPLE_RATE * 0.5)
    t = np.arange(samples) / SAMPLE_RATE
    
    axes[0].plot(t, modulated_signal[:samples], linewidth=0.7, color='#2E86AB')
    axes[0].set_title("Señal Modulada (Limpia)", fontsize=12, fontweight='bold')
    axes[0].set_ylabel("Amplitud")
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(t, modulated_noisy[:samples], linewidth=0.7, color='#E63946')
    axes[1].set_title(f"Señal Modulada + Ruido ({snr_db} dB SNR)", fontsize=12, fontweight='bold')
    axes[1].set_ylabel("Amplitud")
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(t, demodulated_signal[:samples], linewidth=0.7, color='#A23B72')
    axes[2].set_title("Señal Demodulada", fontsize=12, fontweight='bold')
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].set_ylabel("Amplitud")
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Comparación de bits
    print("\n[COMPARACIÓN] Mostrando las 3 secuencias de bits...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # Bits originales
    markerline1, stemlines1, baseline1 = axes[0].stem(range(len(data_bits)), data_bits, basefmt=' ')
    markerline1.set_color('#2E86AB')
    markerline1.set_markersize(8)
    stemlines1.set_color('#2E86AB')
    stemlines1.set_linewidth(1.5)
    axes[0].set_title("Bits Originales (Transmitidos)", fontsize=12, fontweight='bold')
    axes[0].set_ylabel("Valor")
    axes[0].set_ylim(-0.5, 1.5)
    axes[0].grid(True, alpha=0.3, axis='y')
    axes[0].set_xlim(-0.5, len(data_bits) - 0.5)
    
    # Bits de la señal con ruido (sin demodular correctamente)
    markerline2, stemlines2, baseline2 = axes[1].stem(range(len(demodulated_bits)), demodulated_bits, basefmt=' ')
    markerline2.set_color('#E63946')
    markerline2.set_markersize(8)
    stemlines2.set_color('#E63946')
    stemlines2.set_linewidth(1.5)
    axes[1].set_title(f"Bits Recuperados de Señal con Ruido (SNR={snr_db} dB)", fontsize=12, fontweight='bold')
    axes[1].set_ylabel("Valor")
    axes[1].set_ylim(-0.5, 1.5)
    axes[1].grid(True, alpha=0.3, axis='y')
    axes[1].set_xlim(-0.5, len(demodulated_bits) - 0.5)
    
    # Diferencia entre bits
    bit_errors = (data_bits != demodulated_bits).astype(int)
    markerline3, stemlines3, baseline3 = axes[2].stem(range(len(bit_errors)), bit_errors, basefmt=' ')
    markerline3.set_color('#A23B72')
    markerline3.set_markersize(8)
    stemlines3.set_color('#A23B72')
    stemlines3.set_linewidth(1.5)
    axes[2].set_title("Errores de Bit (1=Error, 0=Correcto)", fontsize=12, fontweight='bold')
    axes[2].set_xlabel("Índice de Bit")
    axes[2].set_ylabel("Error")
    axes[2].set_ylim(-0.5, 1.5)
    axes[2].grid(True, alpha=0.3, axis='y')
    axes[2].set_xlim(-0.5, len(bit_errors) - 0.5)
    
    plt.tight_layout()
    plt.show()
    
    # Resumen
    print("\n" + "="*70)
    print("RESUMEN DE RESULTADOS")
    print("="*70)
    print(f"Parámetros:")
    print(f"  • Freq bit 0: {FREQ_BIT_0} Hz")
    print(f"  • Freq bit 1: {FREQ_BIT_1} Hz")
    print(f"  • Bits transmitidos: {num_bits}")
    print(f"  • SNR: {snr_db} dB")
    print(f"\nResultados:")
    print(f"  • Bits correctos: {num_bits - errors}/{num_bits}")
    print(f"  • BER: {ber*100:.2f}%")
    print(f"  • Confiabilidad: {(1-ber)*100:.2f}%")
    print(f"\nArchivos guardados en '{output_dir}/':")
    print(f"  • fsk_modulated_clean.wav")
    print(f"  • fsk_modulated_noisy.wav")
    print(f"  • fsk_demodulated.wav")
    print("="*70)


if __name__ == "__main__":
    main()