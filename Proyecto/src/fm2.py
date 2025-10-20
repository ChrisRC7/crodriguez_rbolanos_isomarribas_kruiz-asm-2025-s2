"""
EXPERIMENTO DE MODULACIÓN Y DEMODULACIÓN FM USANDO FFT

Este programa implementa:
1. Generación de señal de mensaje
2. Modulación FM
3. Análisis espectral usando FFT para identificar componentes
4. Sintonización y filtrado de la señal FM
5. Demodulación FM usando FFT (Hilbert)
6. Visualización de resultados espectrales y temporales
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile

class FMModulationExperiment:
    """
    Clase para realizar experimentos de modulación y demodulación FM
    con análisis espectral completo usando FFT
    """
    
    def __init__(self, fs=44100, fc=10000, kf=5000, duration=2.0):
        """
        Parámetros:
        - fs: Frecuencia de muestreo (Hz)
        - fc: Frecuencia portadora (Hz)
        - kf: Desviación de frecuencia por voltio (Hz/V)
        - duration: Duración de la señal (segundos)
        """
        self.fs = fs
        self.fc = fc
        self.kf = kf
        self.duration = duration
        self.t = np.linspace(0, duration, int(fs * duration), endpoint=False)

        # Directorio de salida en la raíz del proyecto: <Proyecto>/out_FM
        base_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(base_dir, ".."))
        self.output_dir = os.path.join(project_root, "out_FM")
        os.makedirs(self.output_dir, exist_ok=True)
        self.fm_max_est = None
        
    def generate_message_signal(self, frequencies=[440, 880], amplitudes=[1.0, 0.5]):
        """
        Genera señal de mensaje (suma de senoidales) y normaliza a |x|<=1
        """
        self.message = np.zeros_like(self.t, dtype=float)
        for freq, amp in zip(frequencies, amplitudes):
            self.message += amp * np.sin(2 * np.pi * freq * self.t)
        
        peak = np.max(np.abs(self.message)) + 1e-12
        self.message = self.message / peak
        return self.message
    
    def fm_modulate(self):
        """
        Modula la señal mensaje usando FM: cos(2π f_c t + 2π k_f ∫ m(t) dt)
        """
        phase = 2 * np.pi * self.kf * np.cumsum(self.message) / self.fs
        self.fm_signal = np.cos(2 * np.pi * self.fc * self.t + phase)
        return self.fm_signal
    
    def compute_fft(self, signal_data):
        """
        Calcula la FFT de una señal (solo frecuencias positivas)
        """
        N = len(signal_data)
        fft_result = np.fft.rfft(signal_data)
        fft_freq = np.fft.rfftfreq(N, d=1/self.fs)
        return fft_freq, fft_result
    
    def identify_carrier_in_spectrum(self):
        """
        Identifica la portadora y bandas laterales en el espectro
        Nota: el pico máximo puede ser una banda lateral, no necesariamente f_c.
        """
        freq, fft_fm = self.compute_fft(self.fm_signal)
        magnitude = np.abs(fft_fm)
        
        # Pico global (puede ser banda lateral)
        global_idx = np.argmax(magnitude)
        global_peak_freq = freq[global_idx]
        
        # Pico más cercano a la fc (ventana ±2*kf)
        window = (freq >= self.fc - 2*self.kf) & (freq <= self.fc + 2*self.kf)
        if np.any(window):
            local_idx = np.argmax(magnitude[window])
            carrier_freq_near = freq[window][local_idx]
        else:
            carrier_freq_near = global_peak_freq
        
        print(f"📡 Pico global en: {global_peak_freq:.2f} Hz")
        print(f"📡 Portadora teórica: {self.fc:.2f} Hz")
        print(f"📡 Pico cercano a f_c: {carrier_freq_near:.2f} Hz")
        
        return freq, magnitude, carrier_freq_near
    
    def bandpass_filter(self, signal_data, lowcut, highcut, order=5):
        """
        Aplica un filtro pasa-banda para sintonizar la señal
        """
        nyquist = 0.5 * self.fs
        
        # Ajustes seguros
        lowcut = max(1.0, float(lowcut))
        highcut = min(nyquist * 0.99, float(highcut))
        if not (lowcut < highcut):
            raise ValueError("lowcut debe ser menor que highcut.")
        
        low = lowcut / nyquist
        high = highcut / nyquist
        if not (0 < low < high < 1):
            raise ValueError("Frecuencias normalizadas inválidas para el filtro.")
        
        b, a = signal.butter(order, [low, high], btype='band')
        filtered_signal = signal.filtfilt(b, a, signal_data)
        return filtered_signal
    
    def tune_signal(self, bandwidth=None):
        """
        Sintoniza la señal FM usando filtro pasa-banda (regla de Carson por defecto)
        """
        # Estimar fm_max a partir del mensaje (frecuencia con mayor energía significativa)
        freq_msg, fft_msg = self.compute_fft(self.message)
        mag_msg = np.abs(fft_msg)
        # Ignorar DC
        mag_msg[0] = 0.0
        th = 0.05 * np.max(mag_msg) + 1e-12
        idxs = np.where(mag_msg >= th)[0]
        fm_max_est = freq_msg[idxs[-1]] if idxs.size > 0 else 3000.0
        self.fm_max_est = float(min(fm_max_est, 0.45*self.fs))
        
        # Regla de Carson: BW ≈ 2(Δf + f_máx)
        delta_f = self.kf * np.max(np.abs(self.message))  # ≈ kf si mensaje está en [-1,1]
        if bandwidth is None:
            bandwidth = 2*(delta_f + self.fm_max_est)
        
        # Calcular límites del filtro
        lowcut = max(1.0, self.fc - bandwidth / 2)
        highcut = min(self.fs/2 * 0.99, self.fc + bandwidth / 2)
        if lowcut >= highcut:
            raise ValueError("Rango de filtro inválido después del cálculo de bandwidth.")
        
        print(f"📻 Sintonizando señal FM:")
        print(f"   - Frecuencia portadora: {self.fc:.0f} Hz")
        print(f"   - Ancho de banda: {bandwidth:.0f} Hz")
        print(f"   - Rango del filtro: {lowcut:.0f} Hz - {highcut:.0f} Hz")
        
        self.tuned_signal = self.bandpass_filter(self.fm_signal, lowcut, highcut)
        return self.tuned_signal
    
    def fm_demodulate_fft(self):
        """
        Demodula la señal FM usando fase instantánea (Hilbert):
        f_inst = (1/2π) dφ/dt  ->  (f_inst - f_c)/k_f ≈ m(t)
        """
        if not hasattr(self, "tuned_signal"):
            raise RuntimeError("Primero sintoniza la señal con tune_signal().")
        
        analytic_signal = signal.hilbert(self.tuned_signal)
        instantaneous_phase = np.unwrap(np.angle(analytic_signal))
        
        # dφ/dt -> frecuencia instantánea
        inst_freq = np.diff(instantaneous_phase) * self.fs / (2 * np.pi)
        # Alinear tamaño con la señal original
        inst_freq = np.concatenate([inst_freq[:1], inst_freq])
        
        # Remover la portadora y escalar por kf para recuperar m(t)
        demod = (inst_freq - self.fc) / self.kf
        
        # Quitar DC
        demod = demod - np.mean(demod)
        
        # Filtrado pasa-bajos al ancho del mensaje (suaviza ruido de la derivada)
        cutoff = min(max(1.5 * (self.fm_max_est or 3000.0), 100.0), 0.45*self.fs)
        b, a = signal.butter(5, cutoff / (0.5*self.fs), btype='low')
        demod = signal.filtfilt(b, a, demod)
        
        # Normalizar nivel RMS al del mensaje original para que "suenen" parecido
        rms_msg = np.sqrt(np.mean(self.message**2)) + 1e-12
        rms_dem = np.sqrt(np.mean(demod**2)) + 1e-12
        demod *= (rms_msg / rms_dem)
        
        # Limitar a [-1,1] para audio
        demod = np.clip(demod, -1.0, 1.0)
        
        self.demodulated = demod
        return self.demodulated
    
    def plot_time_domain(self):
        """
        Grafica señales en el dominio del tiempo
        """
        fig, axes = plt.subplots(4, 1, figsize=(14, 10))
        
        nshow = min(2000, len(self.t))
        axes[0].plot(self.t[:nshow], self.message[:nshow], 'b-', linewidth=1)
        axes[0].set_title('Señal de Mensaje Original', fontsize=12, fontweight='bold')
        axes[0].set_ylabel('Amplitud')
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(self.t[:nshow], self.fm_signal[:nshow], 'r-', linewidth=0.5)
        axes[1].set_title('Señal FM Modulada', fontsize=12, fontweight='bold')
        axes[1].set_ylabel('Amplitud')
        axes[1].grid(True, alpha=0.3)
        
        axes[2].plot(self.t[:nshow], self.tuned_signal[:nshow], 'g-', linewidth=0.5)
        axes[2].set_title('Señal FM Sintonizada (Filtrada)', fontsize=12, fontweight='bold')
        axes[2].set_ylabel('Amplitud')
        axes[2].grid(True, alpha=0.3)
        
        axes[3].plot(self.t[:nshow], self.demodulated[:nshow], 'm-', linewidth=1)
        axes[3].set_title('Mensaje Demodulado (Recuperado)', fontsize=12, fontweight='bold')
        axes[3].set_xlabel('Tiempo (s)')
        axes[3].set_ylabel('Amplitud')
        axes[3].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/fm2_time_domain_signals.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_frequency_domain(self):
        """
        Grafica espectros de frecuencia usando FFT
        """
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        freq_msg, fft_msg = self.compute_fft(self.message)
        axes[0].plot(freq_msg, 20*np.log10(np.abs(fft_msg) + 1e-10), 'b-', linewidth=1)
        axes[0].set_title('Espectro del Mensaje Original (FFT)', fontsize=12, fontweight='bold')
        axes[0].set_ylabel('Magnitud (dB)')
        axes[0].set_xlim(0, 3000)
        axes[0].grid(True, alpha=0.3)
        
        freq_fm, fft_fm = self.compute_fft(self.fm_signal)
        axes[1].plot(freq_fm, 20*np.log10(np.abs(fft_fm) + 1e-10), 'r-', linewidth=1)
        axes[1].set_title('Espectro de la Señal FM Modulada (FFT)', fontsize=12, fontweight='bold')
        axes[1].set_ylabel('Magnitud (dB)')
        axes[1].axvline(self.fc, color='k', linestyle='--', label=f'Portadora: {self.fc} Hz')
        axes[1].set_xlim(0, min(25000, 0.5*self.fs))
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        freq_demod, fft_demod = self.compute_fft(self.demodulated)
        axes[2].plot(freq_demod, 20*np.log10(np.abs(fft_demod) + 1e-10), 'm-', linewidth=1)
        axes[2].set_title('Espectro del Mensaje Demodulado (FFT)', fontsize=12, fontweight='bold')
        axes[2].set_xlabel('Frecuencia (Hz)')
        axes[2].set_ylabel('Magnitud (dB)')
        axes[2].set_xlim(0, 3000)
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/fm2_frequency_domain_spectra.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_spectral_identification(self):
        """
        Grafica detallada para identificar componentes espectrales
        """
        freq, magnitude, carrier_freq = self.identify_carrier_in_spectrum()
        
        plt.figure(figsize=(14, 6))
        
        plt.subplot(1, 2, 1)
        plt.plot(freq, 20*np.log10(magnitude + 1e-10), 'r-', linewidth=1)
        plt.axvline(carrier_freq, color='k', linestyle='--', linewidth=2, 
                   label=f'Pico cercano a f_c: {carrier_freq:.0f} Hz')
        plt.axvline(self.fc - self.kf, color='b', linestyle=':', alpha=0.7, 
                   label=f'Banda lateral inferior')
        plt.axvline(self.fc + self.kf, color='b', linestyle=':', alpha=0.7, 
                   label=f'Banda lateral superior')
        plt.title('Identificación Espectral de Componentes FM', fontsize=12, fontweight='bold')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Magnitud (dB)')
        plt.xlim(0, min(25000, 0.5*self.fs))
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Zoom en la región de la portadora
        plt.subplot(1, 2, 2)
        plt.plot(freq, 20*np.log10(magnitude + 1e-10), 'r-', linewidth=1)
        plt.axvline(carrier_freq, color='k', linestyle='--', linewidth=2, 
                   label=f'Pico cercano a f_c: {carrier_freq:.0f} Hz')
        plt.title('Zoom: Región de la Portadora', fontsize=12, fontweight='bold')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Magnitud (dB)')
        plt.xlim(self.fc - 10000, self.fc + 10000)
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/fm2_spectral_identification.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def save_audio_files(self):
        """
        Guarda archivos de audio WAV
        """
        # Normalizar y convertir a int16
        def to_int16(x):
            x = np.clip(x, -1.0, 1.0)
            return np.int16(x * 32767)
        
        message_audio = to_int16(self.message)
        fm_audio = to_int16(self.fm_signal * 0.5)  # evitar clipping
        demodulated_audio = to_int16(self.demodulated)
        
        wavfile.write(f'{self.output_dir}/fm2_mensaje_original.wav', self.fs, message_audio)
        wavfile.write(f'{self.output_dir}/fm2_señal_modulada.wav', self.fs, fm_audio)
        wavfile.write(f'{self.output_dir}/fm2_mensaje_demodulado.wav', self.fs, demodulated_audio)
        
        print(f"\n✅ Archivos de audio guardados en: {self.output_dir}/")
        print(f"   - fm2_mensaje_original.wav")
        print(f"   - fm2_señal_modulada.wav")
        print(f"   - fm2_mensaje_demodulado.wav")
    
    def run_complete_experiment(self):
        """
        Ejecuta el experimento completo
        """
        print("="*60)
        print("EXPERIMENTO DE MODULACIÓN Y DEMODULACIÓN FM CON FFT")
        print("="*60)
        
        print("\n1️⃣  Generando señal de mensaje...")
        self.generate_message_signal(frequencies=[440, 880], amplitudes=[1.0, 0.5])
        
        print("2️⃣  Modulando señal FM...")
        self.fm_modulate()
        
        print("3️⃣  Analizando espectro con FFT...")
        self.identify_carrier_in_spectrum()
        
        print("4️⃣  Sintonizando señal FM...")
        self.tune_signal()
        
        print("5️⃣  Demodulando señal FM...")
        self.fm_demodulate_fft()
        
        print("\n6️⃣  Generando visualizaciones...")
        self.plot_time_domain()
        self.plot_frequency_domain()
        self.plot_spectral_identification()
        
        print("\n7️⃣  Guardando archivos de audio...")
        self.save_audio_files()
        
        print("\n" + "="*60)
        print("✅ EXPERIMENTO COMPLETADO CON ÉXITO")
        print("="*60)
        print(f"\nResultados guardados en: {self.output_dir}/")
        print("\nArchivos generados:")
        print("  📊 Gráficas:")
        print("     - fm2_time_domain_signals.png")
        print("     - fm2_frequency_domain_spectra.png")
        print("     - fm2_spectral_identification.png")
        print("  🔊 Audio:")
        print("     - fm2_mensaje_original.wav")
        print("     - fm2_señal_modulada.wav")
        print("     - fm2_mensaje_demodulado.wav")


# =======================================================
# EJECUCIÓN DEL EXPERIMENTO
# =======================================================
if __name__ == "__main__":
    # Crear instancia del experimento
    experiment = FMModulationExperiment(
        fs=44100,          # Frecuencia de muestreo
        fc=10000,          # Frecuencia portadora (10 kHz)
        kf=5000,           # Desviación de frecuencia (5 kHz)
        duration=2.0       # Duración (2 segundos)
    )
    
    # Ejecutar experimento completo
    experiment.run_complete_experiment()