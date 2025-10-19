"""
EXPERIMENTO DE MODULACIÓN Y DEMODULACIÓN FM USANDO FFT

Objetivo:
Aplicar técnicas de análisis espectral complejo (FFT) para observar y analizar
la modulación y demodulación FM. Se busca identificar visualmente la portadora,
las bandas laterales y el mensaje recuperado.
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import hilbert
import scipy.io.wavfile
from fft import FFT  # Clase FFT desarrollada en la etapa anterior

# =======================================================
# PARÁMETROS DE CONFIGURACIÓN
# =======================================================
SAMPLE_RATE = 44100      # Frecuencia de muestreo (Hz)
FC = 8000                # Frecuencia de portadora (Hz)
KF = 2500                # Sensibilidad de frecuencia
F_MESSAGE = 400          # Frecuencia del mensaje (Hz)
DURATION = 2.0           # Duración en segundos
AMPLITUD = 1.0           # Amplitud del mensaje

# =======================================================
# 1. GENERACIÓN DE SEÑAL DE MENSAJE
# =======================================================
t = np.linspace(0, DURATION, int(SAMPLE_RATE * DURATION), endpoint=False)
message = AMPLITUD * np.sin(2 * np.pi * F_MESSAGE * t)

# =======================================================
# 2. MODULACIÓN FM
# =======================================================
integral_message = np.cumsum(message) / SAMPLE_RATE
fm_signal = np.cos(2 * np.pi * (FC * t + KF * integral_message))

# =======================================================
# 3. ANÁLISIS ESPECTRAL DE LA SEÑAL FM (FFT)
# =======================================================
print("📈 Analizando espectro de la señal FM modulada...")
fft_fm = FFT(fm_signal, SAMPLE_RATE, mostrar_graficas=False)

plt.figure(figsize=(10, 5))
plt.plot(fft_fm.xf, 20 * np.log10(np.abs(fft_fm.yf) + 1e-8))
plt.title("Espectro de la señal FM modulada")
plt.xlabel("Frecuencia [Hz]")
plt.ylabel("Magnitud [dB]")
plt.xlim(0, 20000)
plt.grid(True)
plt.show()

# =======================================================
# 4. DEMODULACIÓN FM USANDO ANÁLISIS ESPECTRAL COMPLEJO
# =======================================================
print("🎧 Recuperando mensaje mediante análisis de fase instantánea (Hilbert)...")

analytic_signal = hilbert(fm_signal)
instantaneous_phase = np.unwrap(np.angle(analytic_signal))
instantaneous_freq = np.diff(instantaneous_phase) * SAMPLE_RATE / (2 * np.pi)
demodulated = instantaneous_freq - np.mean(instantaneous_freq)
demodulated /= np.max(np.abs(demodulated))

# =======================================================
# 5. ANÁLISIS FFT DEL MENSAJE DEMODULADO
# =======================================================
fft_demod = FFT(demodulated, SAMPLE_RATE, mostrar_graficas=False)

plt.figure(figsize=(10, 5))
plt.plot(fft_demod.xf, 20 * np.log10(np.abs(fft_demod.yf) + 1e-8))
plt.title("Espectro del mensaje demodulado")
plt.xlabel("Frecuencia [Hz]")
plt.ylabel("Magnitud [dB]")
plt.xlim(0, 2000)
plt.grid(True)
plt.show()

# =======================================================
# 6. COMPARACIÓN TEMPORAL ENTRE SEÑALES
# =======================================================
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t, message, color="steelblue")
plt.title("Mensaje Original (400 Hz)")
plt.ylabel("Amplitud")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t, fm_signal, color="darkred")
plt.title("Señal Modulada FM (Portadora 8 kHz)")
plt.ylabel("Amplitud")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t[:-1], demodulated, color="purple")
plt.title("Mensaje Demodulado (Recuperado)")
plt.xlabel("Tiempo [s]")
plt.ylabel("Amplitud")
plt.grid(True)

plt.tight_layout()
plt.show()

# =======================================================
# 7. GUARDAR RESULTADOS EN AUDIO (OPCIONAL)
# =======================================================
scipy.io.wavfile.write("mensaje_original.wav", SAMPLE_RATE, np.int16(message * 32767))
scipy.io.wavfile.write("fm_modulada.wav", SAMPLE_RATE, np.int16(fm_signal * 32767))
scipy.io.wavfile.write("fm_demodulada.wav", SAMPLE_RATE, np.int16(demodulated * 32767))

print("\n✅ EXPERIMENTO COMPLETADO")
print("Archivos generados:")
print(" - mensaje_original.wav")
print(" - fm_modulada.wav")
print(" - fm_demodulada.wav")
print("\nPuedes escuchar cómo la modulación FM transporta el mensaje original.\n")
