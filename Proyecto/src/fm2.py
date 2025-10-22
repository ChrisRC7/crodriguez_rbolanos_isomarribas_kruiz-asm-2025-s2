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
from fft import FFT

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
        # Variables de resumen
        self.global_peak_freq = None
        self.carrier_freq_observed = None
        self.bandwidth_used = None
        self.lowcut = None
        self.highcut = None
        self.delta_f = None
        self.demod_lpf_cutoff = None
        
    def generate_message_signal(self, frequency=440, amplitude=1.0):
        """
        Genera señal de mensaje senoidal pura y normaliza a |x|<=1
        Parámetros:
        - frequency: Frecuencia del seno en Hz (por defecto 440 Hz = La musical)
        - amplitude: Amplitud antes de normalizar
        """
        self.message = amplitude * np.sin(2 * np.pi * frequency * self.t)
        
        # Normalizar a [-1, 1]
        peak = np.max(np.abs(self.message)) + 1e-12
        self.message = self.message / peak
        
        print(f"   ✓ Señal senoidal generada: {frequency} Hz")
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
        Calcula la FFT usando la clase FFT (misma forma que fft.py).
        No aplica ventana ni remueve DC para que los resultados coincidan.
        """
        xf, yf = FFT(signal_data, self.fs, mostrar_graficas=False).get_spectrum()
        return xf, yf
    
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

        # Guardar para el resumen
        self.global_peak_freq = float(global_peak_freq)
        self.carrier_freq_observed = float(carrier_freq_near)

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
        self.delta_f = float(delta_f)
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

        # Guardar para el resumen
        self.bandwidth_used = float(bandwidth)
        self.lowcut = float(lowcut)
        self.highcut = float(highcut)
        
        self.tuned_signal = self.bandpass_filter(self.fm_signal, lowcut, highcut)
        return self.tuned_signal
    
    def fm_demodulate_fft(self):
        """
        Demodula FM usando fase instantánea (Hilbert).
        Mejoras: gradient, filtro DC-block, LPF más selectivo.
        """
        if not hasattr(self, "tuned_signal"):
            raise RuntimeError("Primero sintoniza la señal con tune_signal().")
        
        # Señal analítica y fase instantánea
        analytic_signal = signal.hilbert(self.tuned_signal)
        instantaneous_phase = np.unwrap(np.angle(analytic_signal))
        
        # Frecuencia instantánea (gradient evita desfase de 1/2 muestra)
        inst_freq = np.gradient(instantaneous_phase) * self.fs / (2 * np.pi)
        
        # Quitar portadora y normalizar por kf
        demod = (inst_freq - self.fc) / self.kf
        
        # 1) Remover DC residual
        demod = demod - np.mean(demod)
        
        # 2) Filtro pasa-altos (DC-block) para eliminar pedestal en baja frecuencia
        hp_cutoff = 20.0  # Hz, ajustar según tu señal
        sos_hp = signal.butter(4, hp_cutoff / (0.5*self.fs), btype='high', output='sos')
        demod = signal.sosfilt(sos_hp, demod)
        
        # 3) Filtro pasa-bajos (recuperar solo el mensaje, rechazar ruido)
        cutoff = min(max(1.5 * (self.fm_max_est or 1000.0), 100.0), 0.45*self.fs)
        sos_lp = signal.butter(6, cutoff / (0.5*self.fs), btype='low', output='sos')  # orden 6
        demod = signal.sosfiltfilt(sos_lp, demod)  # filtfilt para fase cero
        
        self.demod_lpf_cutoff = float(cutoff)
        
        # 4) Normalizar RMS para que coincida con el mensaje original
        rms_msg = np.sqrt(np.mean(self.message**2)) + 1e-12
        rms_dem = np.sqrt(np.mean(demod**2)) + 1e-12
        demod *= (rms_msg / rms_dem)
        
        # 5) Clip para evitar saturación en el WAV
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
        Identificación de portadora y bandas laterales en el espectro FM
        """
        freq, magnitude, carrier_freq = self.identify_carrier_in_spectrum()

        plt.figure(figsize=(14, 6))

        # Vista completa
        plt.subplot(1, 2, 1)
        plt.plot(freq, 20*np.log10(magnitude + 1e-12), 'r-', linewidth=1)
        plt.axvline(self.fc, color='k', linestyle='--', linewidth=1, label=f'fc teórica: {self.fc:.0f} Hz')
        plt.axvline(carrier_freq, color='g', linestyle='--', linewidth=1.5, label=f'fc observada: {carrier_freq:.0f} Hz')
        plt.axvline(self.fc - self.kf, color='b', linestyle=':', alpha=0.7, label='≈ fc - kf')
        plt.axvline(self.fc + self.kf, color='b', linestyle=':', alpha=0.7, label='≈ fc + kf')
        plt.title('Identificación Espectral de FM (Vista completa)')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Magnitud (dB)')
        plt.xlim(0, 0.5*self.fs)
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Zoom alrededor de fc
        plt.subplot(1, 2, 2)
        plt.plot(freq, 20*np.log10(magnitude + 1e-12), 'r-', linewidth=1)
        bw = self.bandwidth_used if self.bandwidth_used else 2*(self.kf + (self.fm_max_est or 3000))
        left = max(0.0, self.fc - 0.75*bw)
        right = min(0.5*self.fs, self.fc + 0.75*bw)
        plt.axvline(self.fc, color='k', linestyle='--', linewidth=1, label=f'fc teórica: {self.fc:.0f} Hz')
        plt.axvline(carrier_freq, color='g', linestyle='--', linewidth=1.5, label=f'fc observada: {carrier_freq:.0f} Hz')
        plt.title('Zoom alrededor de la portadora')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Magnitud (dB)')
        plt.xlim(left, right)
        plt.grid(True, alpha=0.3)
        plt.legend()

        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/fm2_spectral_identification.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_mag_phase_comparison(self, fmax_view=None, modo='raw'):
        """
        Comparación de magnitud y fase entre el mensaje original y el demodulado
        usando fft.py. Corrige retardo temporal y offset de fase para que la fase sea comparable.
        """
        if not hasattr(self, "message") or not hasattr(self, "demodulated"):
            raise RuntimeError("Faltan señales para comparar. Ejecuta generación y demodulación primero.")

        msg = self.message
        dem = self.demodulated

        # FFT con la clase FFT (idéntico a fft.py)
        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        if fmax_view is None:
            fmax_view = 3000.0
        mask_msg = xf_msg <= fmax_view
        mask_dem = xf_dem <= fmax_view

        # Fase cruda
        phi_msg = np.unwrap(np.angle(yf_msg[mask_msg]))
        phi_dem = np.unwrap(np.angle(yf_dem[mask_dem]))
        f_msg = xf_msg[mask_msg]
        f_dem = xf_dem[mask_dem]

        # Re-muestrear fase del demod en las mismas frecuencias (mismo vector)
        # rfft devuelve mismas freq si N es igual; asumimos N igual ⇒ usar f_msg
        # Estimar retardo τ con regresión lineal sobre la diferencia de fase (ponderada por magnitud)
        mag_w = np.minimum(np.abs(yf_msg[mask_msg]), np.abs(yf_dem[mask_dem]))
        th = 0.1 * np.max(mag_w) + 1e-12
        use = mag_w >= th
        if np.any(use):
            phi_diff = np.unwrap(phi_dem - phi_msg)
            # Ajuste: phi_diff ≈ -2π f τ + φ0
            X = np.vstack([f_msg[use], np.ones(np.sum(use))]).T
            w = mag_w[use]
            coef, _, _, _ = np.linalg.lstsq((X * w[:, None]), (phi_diff[use] * w), rcond=None)
            slope, phi0 = coef[0], coef[1]
            tau = -slope / (2*np.pi)
            # Corregir fase del demod
            phi_dem_corr = np.unwrap(phi_dem + 2*np.pi*f_msg*tau - phi0)
        else:
            tau = 0.0
            phi_dem_corr = phi_dem

        plt.figure(figsize=(14, 8))

        # Magnitud (lineal, como fft.py)
        plt.subplot(2, 1, 1)
        plt.plot(f_msg, np.abs(yf_msg[mask_msg]), label='Mensaje', color='b')
        plt.plot(f_msg, np.abs(yf_dem[mask_dem]), label='Demodulado', color='m', alpha=0.85)
        plt.title('Comparación de Magnitud (fft.py)')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Magnitud |Y(f)|')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Fase corregida (quita retardo y offset)
        plt.subplot(2, 1, 2)
        plt.plot(f_msg, phi_msg, label='Mensaje', color='b')
        plt.plot(f_msg, phi_dem_corr, label='Demodulado (fase corregida)', color='m', alpha=0.85)
        plt.title('Comparación de Fase (fft.py) - corregida por retardo/offset')
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('Fase (rad)')
        plt.grid(True, alpha=0.3)
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'fm2_mag_phase_comparison.png'), dpi=300, bbox_inches='tight')
        plt.show()

        print(f"↪ Corrección de fase: τ ≈ {tau*1e3:.3f} ms (estimado)")
    
    # NUEVO: comparación EXACTA como fft.py (magnitud y fase crudas)
    def plot_mag_phase_fft_like(self):
        """
        Espectros superpuestos (Mensaje vs Demodulado) usando fft.py:
        - Magnitud |Y(f)| lineal
        - Fase ∠Y(f) cruda (np.angle), sin unwrap ni dB
        """
        # Asegurar mismo N
        N = min(len(self.message), len(self.demodulated))
        msg = self.message[:N]
        dem = self.demodulated[:N]

        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        plt.figure(figsize=(12, 8))
        # Magnitud
        plt.subplot(2, 1, 1)
        plt.plot(xf_msg, np.abs(yf_msg), label='Mensaje', color='b')
        plt.plot(xf_dem, np.abs(yf_dem), label='Demodulado', color='m', alpha=0.85)
        plt.title("Espectro de Magnitud (fft.py - comparación)")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.legend()
        # Fase cruda
        plt.subplot(2, 1, 2)
        plt.plot(xf_msg, np.angle(yf_msg), label='Mensaje', color='b')
        plt.plot(xf_dem, np.angle(yf_dem), label='Demodulado', color='m', alpha=0.85)
        plt.title("Espectro de Fase (fft.py - comparación)")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (rad)")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'fm2_mag_phase_fft_like.png'), dpi=300, bbox_inches='tight')
        plt.show()
    
    # Estima τ (retardo) y φ0 (offset de fase) al estilo fft.py
    def estimate_phase_alignment_fft_like(self, fmax_view=3000.0, rel_thr=0.01):
        N = min(len(self.message), len(self.demodulated))
        msg = self.message[:N]
        dem = self.demodulated[:N]

        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        amp_msg = np.abs(yf_msg)
        amp_dem = np.abs(yf_dem)
        mask_f = (xf_msg > 0) & (xf_msg <= float(fmax_view))
        thr = rel_thr * (amp_msg.max() + 1e-12)
        mask = mask_f & (amp_msg >= thr) & (amp_dem >= thr)

        if not np.any(mask):
            return 0.0, 0.0, 0

        f = xf_msg[mask]
        phi_diff = np.unwrap(np.angle(yf_dem[mask]) - np.angle(yf_msg[mask]))

        w = np.minimum(amp_msg[mask], amp_dem[mask])
        X = np.vstack([f, np.ones_like(f)]).T
        coef, _, _, _ = np.linalg.lstsq((X * w[:, None]), (phi_diff * w), rcond=None)
        slope, phi0 = coef[0], coef[1]
        tau = -slope / (2*np.pi)
        return float(tau), float(phi0), int(np.count_nonzero(mask))

    # Graficar magnitud y fase "fft.py-like" con fase alineada
    def plot_mag_phase_fft_like_aligned(self, fmax_view=3000.0, rel_thr=0.01):
        N = min(len(self.message), len(self.demodulated))
        msg = self.message[:N]
        dem = self.demodulated[:N]

        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        tau, phi0, used = self.estimate_phase_alignment_fft_like(fmax_view=fmax_view, rel_thr=rel_thr)

        # Corregir fase del demodulado en frecuencia
        phase_rot = np.exp(1j * (2*np.pi*xf_dem*tau - phi0))
        yf_dem_corr = yf_dem * phase_rot

        mask_msg = xf_msg <= float(fmax_view)
        mask_dem = xf_dem <= float(fmax_view)

        plt.figure(figsize=(12, 8))
        # Magnitud
        plt.subplot(2, 1, 1)
        plt.plot(xf_msg[mask_msg], np.abs(yf_msg[mask_msg]), label='Mensaje', color='b')
        plt.plot(xf_dem[mask_dem], np.abs(yf_dem[mask_dem]), label='Demodulado', color='m', alpha=0.6)
        plt.plot(xf_dem[mask_dem], np.abs(yf_dem_corr[mask_dem]), label='Demod (alineado)', color='g', alpha=0.9)
        plt.title("Espectro de Magnitud (fft.py) con demod alineado")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud |Y(f)|")
        plt.grid(True); plt.legend()

        # Fase cruda, demod corregido
        plt.subplot(2, 1, 2)
        plt.plot(xf_msg[mask_msg], np.angle(yf_msg[mask_msg]), label='Mensaje', color='b')
        plt.plot(xf_dem[mask_dem], np.angle(yf_dem[mask_dem]), label='Demod (sin alinear)', color='m', alpha=0.6)
        plt.plot(xf_dem[mask_dem], np.angle(yf_dem_corr[mask_dem]), label='Demod (alineado)', color='g', alpha=0.9)
        plt.title(f"Espectro de Fase (fft.py) alineado | τ≈{tau*1e3:.3f} ms, φ0≈{phi0:.3f} rad, bins={used}")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase ∠Y(f) [rad]")
        plt.grid(True); plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'fm2_mag_phase_fft_like_aligned.png'),
                    dpi=300, bbox_inches='tight')
        plt.show()

    # Gráfica limpia: solo muestra fase donde la magnitud es significativa
    def plot_mag_phase_fft_like_clean(self, fmax_view=2000.0, mag_threshold_rel=0.02):
        N = min(len(self.message), len(self.demodulated))
        msg = self.message[:N]
        dem = self.demodulated[:N]

        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        amp_msg = np.abs(yf_msg)
        amp_dem = np.abs(yf_dem)
        
        mask_freq = xf_msg <= fmax_view
        thr = mag_threshold_rel * amp_msg.max()
        mask_mag_msg = amp_msg >= thr
        mask_mag_dem = amp_dem >= thr

        plt.figure(figsize=(12, 8))
        
        # Magnitud
        plt.subplot(2, 1, 1)
        plt.plot(xf_msg[mask_freq], amp_msg[mask_freq], label='Mensaje', color='b', linewidth=1.5)
        plt.plot(xf_dem[mask_freq], amp_dem[mask_freq], label='Demodulado', color='m', alpha=0.8, linewidth=1.5)
        plt.title("Magnitud (fft.py)")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud |Y(f)|")
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Fase (solo donde magnitud es suficiente)
        plt.subplot(2, 1, 2)
        mask_plot_msg = mask_freq & mask_mag_msg
        plt.plot(xf_msg[mask_plot_msg], np.angle(yf_msg[mask_plot_msg]), 
                 'o', color='b', markersize=4, label='Mensaje (fase válida)')
        
        mask_plot_dem = mask_freq & mask_mag_dem
        plt.plot(xf_dem[mask_plot_dem], np.angle(yf_dem[mask_plot_dem]), 
                 'x', color='m', markersize=4, alpha=0.8, label='Demodulado (fase válida)')
        
        plt.axhline(-np.pi/2, color='gray', linestyle='--', linewidth=0.8, label='Fase esperada sin(2πft)')
        plt.title(f"Fase (fft.py) - filtrada por magnitud > {mag_threshold_rel:.3f}·max")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase ∠Y(f) [rad]")
        plt.ylim(-np.pi - 0.5, np.pi + 0.5)
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'fm2_mag_phase_clean.png'), dpi=300, bbox_inches='tight')
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
    
    def print_summary(self):
        print("\n---------------- RESUMEN DEL ANÁLISIS FM ----------------")
        print(f"Portadora teórica (fc):          {self.fc:.2f} Hz")
        if self.carrier_freq_observed is not None:
            print(f"Portadora observada (FFT):       {self.carrier_freq_observed:.2f} Hz")
            print(f"Pico global del espectro FM:     {self.global_peak_freq:.2f} Hz")
        if self.fm_max_est is not None:
            print(f"fmax estimada (mensaje):         {self.fm_max_est:.2f} Hz")
        if self.delta_f is not None:
            print(f"Desviación máx. Δf ≈ kf·|m|max:   {self.delta_f:.2f} Hz")
        if self.bandwidth_used is not None:
            print(f"BW usado (Carson):               {self.bandwidth_used:.2f} Hz")
        if self.lowcut is not None and self.highcut is not None:
            print(f"Filtro pasa-banda:               {self.lowcut:.2f} Hz  →  {self.highcut:.2f} Hz")
        if self.demod_lpf_cutoff is not None:
            print(f"LPF demodulación (cutoff):       {self.demod_lpf_cutoff:.2f} Hz")
        print("---------------------------------------------------------\n")
    
    def print_phase_spectrum(self, fmax_view=3000.0, top_n=8, rel_thr=0.01):
        """
        Imprime fase cruda (np.angle) al estilo fft.py para picos dominantes,
        filtrando bins de baja magnitud en ambas señales.
        Muestra: f, |Y_msg|, |Y_dem|, φ_msg, φ_dem y Δφ envuelta = angle(Ydem * conj(Ymsg)).
        Además, resume Δφ medio y desviación estándar en grados.
        """
        if not hasattr(self, "message") or not hasattr(self, "demodulated"):
            print("No hay señales para comparar. Ejecuta generación y demodulación primero.")
            return

        # Asegurar mismo N en ambas FFT
        N = min(len(self.message), len(self.demodulated))
        msg = self.message[:N]
        dem = self.demodulated[:N]

        xf_msg, yf_msg = FFT(msg, self.fs, mostrar_graficas=False).get_spectrum()
        xf_dem, yf_dem = FFT(dem, self.fs, mostrar_graficas=False).get_spectrum()

        amp_msg = np.abs(yf_msg)
        amp_dem = np.abs(yf_dem)
        phi_msg = np.angle(yf_msg)  # fase cruda (fft.py)
        phi_dem = np.angle(yf_dem)

        # Rango y umbral relativo (respecto al máximo del mensaje)
        mask_f = (xf_msg > 0) & (xf_msg <= float(fmax_view))
        thr = rel_thr * (amp_msg.max() + 1e-12)
        mask_mag = (amp_msg >= thr) & (amp_dem >= thr)
        mask = mask_f & mask_mag

        idx_cands = np.where(mask)[0]
        if idx_cands.size == 0:
            print(f"Sin componentes útiles en (0, {fmax_view}] Hz con umbral rel={rel_thr}.")
            return

        # Elegir top_n por magnitud del mensaje
        mags = amp_msg[idx_cands]
        take = int(min(top_n, idx_cands.size))
        top_rel = np.argpartition(mags, -take)[-take:]
        idx = np.sort(idx_cands[top_rel])

        print("\n---- Fase vs Frecuencia (fft.py-like, fase cruda filtrada) ----")
        print(f"Rango: 0–{fmax_view:.0f} Hz | picos listados: {take} | umbral rel: {rel_thr:.3f}")
        print("f(Hz)       |   |Ymsg|        |Ydem|        φ_msg(rad)     φ_dem(rad)     Δφ env.(rad)   Δφ(deg)")
        print("------------+---------------------------------------------------------------------------------------")

        dphis = []
        for i in idx:
            f = xf_msg[i]
            am = amp_msg[i]
            ad = amp_dem[i]
            pm = phi_msg[i]
            pd = phi_dem[i]
            dphi = np.angle(yf_dem[i] * np.conj(yf_msg[i]))  # (-π,π]
            dphis.append(dphi)
            print(f"{f:10.2f} | {am:11.6f}  {ad:11.6f}   {pm:12.6f}   {pd:12.6f}   {dphi:12.6f}   {np.degrees(dphi):9.3f}")

        dphis = np.array(dphis)
        mean_deg = float(np.degrees(np.angle(np.mean(np.exp(1j*dphis)))) )  # media circular
        std_deg = float(np.degrees(np.sqrt(np.mean(np.angle(np.exp(1j*(dphis - np.angle(np.mean(np.exp(1j*dphis))))))**2 + 1e-12))))
        print("----------------------------------------------------------------------------------------------------")
        print(f"Resumen Δφ: media ≈ {mean_deg:.2f}° | dispersión ≈ {std_deg:.2f}°")
        if abs(abs(mean_deg) - 180.0) < 15.0:
            print("Nota: Δφ ≈ 180° sugiere inversión de signo (señal demodulada invertida).")
        print("----------------------------------------------------------------------------------------------------\n")
    
    def run_complete_experiment(self):
        """
        Ejecuta el experimento completo: generación, modulación, análisis espectral,
        sintonización, demodulación y visualización de resultados.
        """
        print("🔄 Ejecutando experimento completo de modulación/demodulación FM...")
        print("1️⃣  Generando señal de mensaje...")
        self.generate_message_signal(frequency=440)  # <- Cambiado: ahora genera un solo seno
        print("2️⃣  Modulando señal mensaje a FM...")
        self.fm_modulate()
        print("3️⃣  Analizando espectro de la señal FM modulada...")
        self.compute_fft(self.fm_signal)
        print("4️⃣  Sintonizando señal FM (filtrado pasa-banda)...")
        self.tune_signal()
        print("5️⃣  Demodulando señal FM...")
        self.fm_demodulate_fft()

        # Imprimir tablas de fase antes de abrir ventanas de matplotlib
        print("\n8️⃣  Imprimiendo espectro de fase vs frecuencia (fft.py-like)...")
        self.print_phase_spectrum(fmax_view=3000.0, top_n=8)

        print("\n6️⃣  Generando visualizaciones...")
        self.plot_time_domain()
        self.plot_frequency_domain()
        self.plot_spectral_identification()
        
        # Gráficas de comparación magnitud/fase (fft.py-like)
        self.plot_mag_phase_fft_like_aligned(fmax_view=3000.0, rel_thr=0.01)  # fase alineada
        self.plot_mag_phase_fft_like_clean(fmax_view=2000.0, mag_threshold_rel=0.02)  # solo picos

        self.save_audio_files()
        self.print_summary()

        print("  📊 Gráficas:")
        print("     - fm2_time_domain_signals.png")
        print("     - fm2_frequency_domain_spectra.png")
        print("     - fm2_spectral_identification.png")
        print("     - fm2_mag_phase_fft_like_aligned.png")
        print("     - fm2_mag_phase_clean.png")
        print("  🎵 Audios:")
        print("     - fm2_mensaje_original.wav")
        print("     - fm2_señal_modulada.wav")
        print("     - fm2_mensaje_demodulado.wav")
        print("\n✅ Experimento completo.")

# Agregar bloque de ejecución directa
if __name__ == "__main__":
    experiment = FMModulationExperiment(fs=44100, fc=10000, kf=5000, duration=2.0)
    experiment.run_complete_experiment()