import numpy as np
from matplotlib import pyplot as plt
from scipy.fft import rfft, rfftfreq, irfft
from scipy.signal import butter, filtfilt
import sounddevice as sd
import soundfile as sf
import os

class FSK:
    def __init__(self, SAMPLE_RATE=8000, duration=2.0, carpeta_salida='SENfsk'):
        """
        Clase para modulación y demodulación FSK (Frequency Shift Keying).
        """
        self.SAMPLE_RATE = SAMPLE_RATE
        self.duration = duration  # duración para señal original (portadora)
        self.t = np.linspace(0, duration, int(SAMPLE_RATE * duration), endpoint=False)
        self.carpeta_salida = carpeta_salida
        
        # Nuevos: tiempos y duraciones separadas
        self.duration_fsk = 0.0
        self.t_fsk = np.array([], dtype=float)
        self.duration_dem = 0.0
        self.t_dem = np.array([], dtype=float)
        
        # Crear la carpeta si no existe
        if not os.path.exists(carpeta_salida):
            os.makedirs(carpeta_salida)
            print(f"Carpeta '{carpeta_salida}' creada.")
        
    def generar_datos_binarios(self, num_bits=None, datos=None):
        """
        Genera o acepta datos binarios para modular.
        
        Parámetros:
            num_bits: Número de bits aleatorios a generar
            datos: Array de datos binarios [0, 1, 0, 1, ...]
        
        Retorna:
            Array de datos binarios
        """
        if datos is not None:
            self.datos = np.array(datos)
        elif num_bits is not None:
            self.datos = np.random.randint(0, 2, num_bits)
        else:
            # Por defecto, genera 16 bits
            self.datos = np.random.randint(0, 2, 16)
        
        print(f"Datos binarios: {self.datos}")
        return self.datos
    
    def generar_tono_original(self, frecuencia=1000, amplitud=0.8):
        """
        Genera un tono senoidal original de frecuencia específica.
        Usa self.duration (no lo modifica).
        """
        # Guardar frecuencia para referencia
        self.frecuencia_original = frecuencia
        
        # Generar tono con la duración original (2s)
        self.senal_original = amplitud * np.sin(2 * np.pi * frecuencia * self.t)
        
        # Guardar archivo WAV
        archivo_salida = os.path.join(self.carpeta_salida, 'senal_original.wav')
        sf.write(archivo_salida, self.senal_original, self.SAMPLE_RATE)
        
        print(f"Tono original generado: {frecuencia}Hz | Duración: {self.duration}s")
        print(f"Señal original guardada en: {archivo_salida}")
        return self.senal_original
    
    def mostrar_senal_original(self):
        """Muestra la señal senoidal original en tiempo y frecuencia."""
        fig = plt.figure(figsize=(15, 10))
        
        # Señal en el tiempo (completa)
        plt.subplot(3, 2, 1)
        plt.plot(self.t, self.senal_original, 'b-', linewidth=1)
        plt.title("Señal Senoidal Original - Vista Completa")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        
        # Señal en el tiempo (zoom primeros 0.05s)
        plt.subplot(3, 2, 2)
        mask = self.t <= 0.05
        plt.plot(self.t[mask], self.senal_original[mask], 'b-', linewidth=2)
        plt.title("Señal Senoidal Original - Zoom (primeros 50ms)")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        
        # FFT - Espectro de magnitud
        N = len(self.senal_original)
        yf = rfft(self.senal_original)
        xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
        
        plt.subplot(3, 2, 3)
        plt.plot(xf, np.abs(yf), 'b-', linewidth=1)
        plt.title("Espectro de Magnitud - Señal Original")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.xlim(0, 3000)
        
        # FFT - Espectro de magnitud (zoom)
        plt.subplot(3, 2, 4)
        mask_freq = xf <= 2000
        plt.plot(xf[mask_freq], np.abs(yf[mask_freq]), 'b-', linewidth=2)
        plt.title("Espectro de Magnitud - Zoom (0-2000 Hz)")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.axvline(self.frecuencia_original, color='r', linestyle='--', label=f'f={self.frecuencia_original}Hz')
        plt.legend()
        
        # FFT - Espectro de fase
        plt.subplot(3, 2, 5)
        plt.plot(xf, np.angle(yf), 'g-', linewidth=1)
        plt.title("Espectro de Fase - Señal Original")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (rad)")
        plt.grid(True)
        plt.xlim(0, 3000)
        
        # Espectrograma
        plt.subplot(3, 2, 6)
        plt.specgram(self.senal_original, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='viridis')
        plt.title("Espectrograma - Señal Original")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, 3000)
        plt.colorbar(label='Intensidad (dB)')
        
        plt.tight_layout()
        plt.show()
    
    def modular_fsk(self, f0=1000, f1=2000, bits_per_second=125, amplitud=1.0):
        """
        Modula los datos binarios usando FSK.
        Genera señal del largo exacto necesario para los bits (sin padding).
        """
        self.f0 = f0
        self.f1 = f1
        self.bits_per_second = bits_per_second

        # Duración y muestras por bit (forza coherencia con Fs/bps)
        samples_per_bit = int(round(self.SAMPLE_RATE / bits_per_second))
        bit_duration = samples_per_bit / float(self.SAMPLE_RATE)

        # Generar señal modulada sin padding de ceros
        bloques = []
        for bit in self.datos:
            t_bit = np.linspace(0, bit_duration, samples_per_bit, endpoint=False)
            senal_bit = amplitud * np.sin(2 * np.pi * (f1 if bit == 1 else f0) * t_bit)
            bloques.append(senal_bit)
        senal_fsk = np.concatenate(bloques) if bloques else np.array([], dtype=float)

        # Ajustar duración y eje de tiempo de la señal modulada (NO tocar self.duration ni self.t)
        self.senal_fsk = senal_fsk
        self.duration_fsk = len(self.senal_fsk) / float(self.SAMPLE_RATE)
        self.t_fsk = np.arange(len(self.senal_fsk)) / float(self.SAMPLE_RATE)

        print(f"Señal FSK modulada: f0={f0}Hz (0), f1={f1}Hz (1) | "
              f"sps={samples_per_bit} | bits={len(self.datos)} | dur={self.duration_fsk:.4f}s")

        # Guardar el archivo exacto (sin relleno)
        archivo_salida = os.path.join(self.carpeta_salida, 'fsk_modulado.wav')
        sf.write(archivo_salida, self.senal_fsk, self.SAMPLE_RATE)
        print(f"Señal modulada guardada en: {archivo_salida}")
        return self.senal_fsk
    
    def mostrar_senal_modulada(self):
        """Muestra análisis completo de la señal FSK modulada."""
        bit_duration = 1.0 / self.bits_per_second
        
        fig = plt.figure(figsize=(15, 12))
        
        # Datos binarios superpuestos con la señal modulada
        plt.subplot(4, 2, 1)
        t_senal = np.arange(len(self.senal_fsk)) / self.SAMPLE_RATE
        
        # Señal modulada
        plt.plot(t_senal, self.senal_fsk, 'r-', linewidth=1, alpha=0.7)
        
        # Datos binarios como fondo
        ax2 = plt.gca().twinx()
        t_bits = np.arange(len(self.datos)) * bit_duration
        ax2.step(t_bits, self.datos, where='post', linewidth=2, color='blue', alpha=0.5)
        ax2.set_ylabel("Bit (0/1)", color='blue')
        ax2.set_ylim(-0.5, 1.5)
        ax2.tick_params(axis='y', labelcolor='blue')
        
        plt.title(f"Señal FSK Modulada con Bits Superpuestos")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud", color='red')
        plt.grid(True, alpha=0.3)
        plt.xlim(0, len(self.datos) * bit_duration)
        
        # Señal modulada (zoom primeros bits) con bits
        plt.subplot(4, 2, 2)
        zoom_time = min(4 * bit_duration, self.duration)
        mask = t_senal <= zoom_time
        mask_bits = t_bits <= zoom_time
        
        plt.plot(t_senal[mask], self.senal_fsk[mask], 'r-', linewidth=2, alpha=0.7)
        
        ax2 = plt.gca().twinx()
        ax2.step(t_bits[mask_bits], self.datos[mask_bits], where='post', linewidth=2, color='blue', alpha=0.5)
        ax2.set_ylabel("Bit (0/1)", color='blue')
        ax2.set_ylim(-0.5, 1.5)
        ax2.tick_params(axis='y', labelcolor='blue')
        
        plt.title("Señal FSK Modulada - Zoom (primeros bits)")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud", color='red')
        plt.grid(True, alpha=0.3)
        
        # FFT - Espectro de magnitud
        N = len(self.senal_fsk)
        yf = rfft(self.senal_fsk)
        xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
        
        plt.subplot(4, 2, 3)
        plt.plot(xf, np.abs(yf), 'r-', linewidth=1)
        plt.title("Espectro de Magnitud - Señal Modulada")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.xlim(0, max(self.f0, self.f1) * 2.5)
        
        # FFT - Espectro de magnitud (zoom con marcadores)
        plt.subplot(4, 2, 4)
        mask_freq = xf <= max(self.f0, self.f1) * 2
        plt.plot(xf[mask_freq], np.abs(yf[mask_freq]), 'r-', linewidth=2)
        plt.axvline(self.f0, color='b', linestyle='--', label=f'f0={self.f0}Hz (bit 0)')
        plt.axvline(self.f1, color='g', linestyle='--', label=f'f1={self.f1}Hz (bit 1)')
        plt.title("Espectro de Magnitud - Zoom con Frecuencias FSK")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.legend()
        
        # FFT - Espectro de fase
        plt.subplot(4, 2, 5)
        plt.plot(xf, np.angle(yf), 'orange', linewidth=1)
        plt.title("Espectro de Fase - Señal Modulada")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (rad)")
        plt.grid(True)
        plt.xlim(0, max(self.f0, self.f1) * 2.5)
        
        # Datos binarios solamente
        plt.subplot(4, 2, 6)
        plt.step(t_bits, self.datos, where='post', linewidth=2.5, color='blue')
        plt.fill_between(t_bits, 0, self.datos, step='post', alpha=0.3, color='blue')
        plt.title("Secuencia de Bits Original")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Bit")
        plt.ylim(-0.2, 1.2)
        plt.grid(True, alpha=0.3)
        plt.yticks([0, 1])
        
        # Añadir etiquetas de bits
        for i, bit in enumerate(self.datos):
            plt.text(t_bits[i] + bit_duration/2, 0.5, f'{bit}', 
                    ha='center', va='center', fontsize=10, fontweight='bold')
        
        # Espectrograma
        plt.subplot(4, 2, 7)
        plt.specgram(self.senal_fsk, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='hot')
        plt.title("Espectrograma - Señal FSK Modulada")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, max(self.f0, self.f1) * 2.5)
        plt.colorbar(label='Intensidad (dB)')
        plt.axhline(self.f0, color='cyan', linestyle='--', linewidth=1, alpha=0.7, label=f'f0={self.f0}Hz')
        plt.axhline(self.f1, color='lime', linestyle='--', linewidth=1, alpha=0.7, label=f'f1={self.f1}Hz')
        plt.legend(loc='upper right')
        
        # Espectrograma con bits superpuestos
        plt.subplot(4, 2, 8)
        plt.specgram(self.senal_fsk, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='hot')
        
        # Superponer los bits en el espectrograma
        for i, bit in enumerate(self.datos):
            t_center = t_bits[i] + bit_duration/2
            if bit == 0:
                plt.text(t_center, self.f0, '0', ha='center', va='center',
                        fontsize=12, fontweight='bold', color='cyan',
                        bbox=dict(boxstyle='circle', facecolor='black', alpha=0.7))
            else:
                plt.text(t_center, self.f1, '1', ha='center', va='center',
                        fontsize=12, fontweight='bold', color='lime',
                        bbox=dict(boxstyle='circle', facecolor='black', alpha=0.7))
        
        plt.title("Espectrograma con Bits Identificados")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, max(self.f0, self.f1) * 2.5)
        plt.colorbar(label='Intensidad (dB)')
        plt.axhline(self.f0, color='cyan', linestyle='--', linewidth=1, alpha=0.5)
        plt.axhline(self.f1, color='lime', linestyle='--', linewidth=1, alpha=0.5)
        
        plt.tight_layout()
        plt.show()
    
    def analizar_espectro(self, senal, titulo="Espectro de Frecuencia"):
        """
        Analiza y muestra el espectro de frecuencia usando FFT.
        
        Parámetros:
            senal: Señal a analizar
            titulo: Título de la gráfica
        
        Retorna:
            xf: Vector de frecuencias
            yf: Vector de coeficientes FFT
        """
        N = len(senal)
        yf = rfft(senal)
        xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
        
        plt.figure(figsize=(12, 8))
        
        # Espectro de magnitud
        plt.subplot(2, 1, 1)
        plt.plot(xf, np.abs(yf))
        plt.title(f"{titulo} - Magnitud")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.xlim(0, max(self.f0, self.f1) * 2)
        
        # Espectro de fase
        plt.subplot(2, 1, 2)
        plt.plot(xf, np.angle(yf))
        plt.title(f"{titulo} - Fase")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (rad)")
        plt.grid(True)
        plt.xlim(0, max(self.f0, self.f1) * 2)
        
        plt.tight_layout()
        plt.show()
        
        return xf, yf
    
    def demodular_fsk(self, senal_fsk=None, metodo='filtro'):
        """
        Demodula la señal FSK para recuperar los datos binarios.
        
        Parámetros:
            senal_fsk: Señal FSK a demodular (usa self.senal_fsk si es None)
            metodo: 'filtro' o 'fft' (default: 'filtro')
        
        Retorna:
            Array de datos binarios recuperados
        """
        if senal_fsk is None:
            senal_fsk = self.senal_fsk
        
        bit_duration = 1.0 / self.bits_per_second
        samples_per_bit = int(self.SAMPLE_RATE * bit_duration)
        num_bits = len(self.datos)
        
        datos_recuperados = []
        
        if metodo == 'filtro':
            # Demodulación usando filtros pasabanda
            for i in range(num_bits):
                inicio = i * samples_per_bit
                fin = inicio + samples_per_bit
                
                if fin > len(senal_fsk):
                    break
                
                segmento = senal_fsk[inicio:fin]
                
                # Calcular energía en f0 y f1 usando FFT
                N = len(segmento)
                yf = rfft(segmento)
                xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
                
                # Encontrar índices cercanos a f0 y f1
                idx_f0 = np.argmin(np.abs(xf - self.f0))
                idx_f1 = np.argmin(np.abs(xf - self.f1))
                
                # Calcular energía en un rango alrededor de cada frecuencia
                rango = 5  # bins a cada lado
                energia_f0 = np.sum(np.abs(yf[max(0, idx_f0-rango):idx_f0+rango+1])**2)
                energia_f1 = np.sum(np.abs(yf[max(0, idx_f1-rango):idx_f1+rango+1])**2)
                
                # Decidir bit basado en mayor energía
                bit = 1 if energia_f1 > energia_f0 else 0
                datos_recuperados.append(bit)
        
        elif metodo == 'fft':
            # Demodulación usando análisis FFT completo
            for i in range(num_bits):
                inicio = i * samples_per_bit
                fin = inicio + samples_per_bit
                
                if fin > len(senal_fsk):
                    break
                
                segmento = senal_fsk[inicio:fin]
                
                # FFT del segmento
                N = len(segmento)
                yf = rfft(segmento)
                xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
                
                # Encontrar pico de frecuencia
                magnitud = np.abs(yf)
                idx_max = np.argmax(magnitud)
                freq_pico = xf[idx_max]
                
                # Decidir bit basado en frecuencia del pico
                if abs(freq_pico - self.f0) < abs(freq_pico - self.f1):
                    bit = 0
                else:
                    bit = 1
                
                datos_recuperados.append(bit)
        
        self.datos_recuperados = np.array(datos_recuperados)
        print(f"Datos recuperados: {self.datos_recuperados}")
        
        # Calcular tasa de error
        if len(self.datos_recuperados) == len(self.datos):
            errores = np.sum(self.datos != self.datos_recuperados)
            tasa_error = errores / len(self.datos) * 100
            print(f"Tasa de error: {tasa_error:.2f}% ({errores}/{len(self.datos)} bits)")
        
        # Reconstruir señal demodulada
        self.reconstruir_senal_demodulada()
        
        return self.datos_recuperados
    
    def reconstruir_senal_demodulada(self):
        """
        Reconstruye la señal portadora a partir de los bits demodulados.
        Usa la frecuencia de la portadora original (f0), no el promedio.
        """
        if not hasattr(self, 'datos_recuperados') or self.datos_recuperados is None or len(self.datos_recuperados) == 0:
            print("Error: No hay bits demodulados. Ejecuta demodular_fsk primero.")
            return None

        # Usar frecuencia de la portadora base (f0=1000 Hz), NO el promedio
        frecuencia = self.f0
        print(f"\nReconstruyendo señal demodulada con frecuencia portadora: {frecuencia:.2f} Hz")

        # Generar tono continuo demodulado con el mismo largo que la FSK
        self.duration_dem = self.duration_fsk
        self.t_dem = np.arange(len(self.senal_fsk)) / float(self.SAMPLE_RATE)
        self.senal_demodulada = 0.8 * np.sin(2 * np.pi * frecuencia * self.t_dem)

        # Guardar
        archivo_salida = os.path.join(self.carpeta_salida, 'fsk_demodulado.wav')
        sf.write(archivo_salida, self.senal_demodulada, self.SAMPLE_RATE)
        print(f"Señal demodulada guardada en: {archivo_salida}")
        return self.senal_demodulada
    
    def mostrar_senal_demodulada(self):
        """Muestra análisis completo de la señal demodulada."""
        fig = plt.figure(figsize=(15, 10))
        # Señal demodulada en el tiempo (completa)
        plt.subplot(3, 2, 1)
        plt.plot(self.t_dem, self.senal_demodulada, 'm-', linewidth=1)  # <-- usar t_dem
        
        # Señal demodulada (zoom)
        plt.subplot(3, 2, 2)
        mask = self.t_dem <= 0.05                                      # <-- usar t_dem
        plt.plot(self.t_dem[mask], self.senal_demodulada[mask], 'm-', linewidth=2)
        plt.title("Señal Demodulada - Zoom (primeros 50ms)")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        
        # FFT - Espectro de magnitud
        N = len(self.senal_demodulada)
        yf = rfft(self.senal_demodulada)
        xf = rfftfreq(N, 1 / self.SAMPLE_RATE)
        
        plt.subplot(3, 2, 3)
        plt.plot(xf, np.abs(yf), 'm-', linewidth=1)
        plt.title("Espectro de Magnitud - Señal Demodulada")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        plt.xlim(0, 3000)
        
        # FFT - Espectro de magnitud (zoom)
        plt.subplot(3, 2, 4)
        mask_freq = xf <= 2000
        plt.plot(xf[mask_freq], np.abs(yf[mask_freq]), 'm-', linewidth=2)
        plt.title("Espectro de Magnitud - Zoom (0-2000 Hz)")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)
        if hasattr(self, 'frecuencia_original'):
            plt.axvline(self.frecuencia_original, color='r', linestyle='--', 
                       label=f'f={self.frecuencia_original}Hz')
        plt.legend()
        
        # FFT - Espectro de fase
        plt.subplot(3, 2, 5)
        plt.plot(xf, np.angle(yf), 'cyan', linewidth=1)
        plt.title("Espectro de Fase - Señal Demodulada")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (rad)")
        plt.grid(True)
        plt.xlim(0, 3000)
        
        # Espectrograma
        plt.subplot(3, 2, 6)
        plt.specgram(self.senal_demodulada, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='plasma')
        plt.title("Espectrograma - Señal Demodulada")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, 3000)
        plt.colorbar(label='Intensidad (dB)')
        
        plt.tight_layout()
        plt.show()
    
    def comparar_senales(self):
        """Compara las tres señales: original, modulada y demodulada."""
        fig = plt.figure(figsize=(16, 12))
        bit_duration = 1.0 / self.bits_per_second
        t_senal = self.t_fsk                                           # <-- tiempo FSK

        # Señales en el tiempo
        plt.subplot(4, 3, 1)
        plt.plot(self.t, self.senal_original, 'b-', linewidth=1, label='Original')
        plt.title("Señal Original")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        plt.legend()
        
        plt.subplot(4, 3, 2)
        plt.plot(t_senal, self.senal_fsk, 'r-', linewidth=1, label='Modulada')
        plt.title("Señal Modulada (FSK)")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        plt.xlim(0, len(self.datos) * bit_duration)
        plt.legend()
        
        plt.subplot(4, 3, 3)
        plt.plot(self.t_dem, self.senal_demodulada, 'm-', linewidth=1, label='Demodulada')
        plt.title("Señal Demodulada")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        plt.legend()
        
        # Zoom (50 ms)
        zoom_time = 0.05
        mask_zoom = self.t <= zoom_time
        mask_zoom_fsk = t_senal <= zoom_time
        mask_zoom_dem = self.t_dem <= zoom_time

        plt.subplot(4, 3, 4)
        plt.plot(self.t[mask_zoom], self.senal_original[mask_zoom], 'b-', linewidth=2)
        plt.subplot(4, 3, 5)
        plt.plot(t_senal[mask_zoom_fsk], self.senal_fsk[mask_zoom_fsk], 'r-', linewidth=2)
        plt.subplot(4, 3, 6)
        plt.plot(self.t_dem[mask_zoom_dem], self.senal_demodulada[mask_zoom_dem], 'm-', linewidth=2)

        # Espectros de magnitud (cada uno con su N)
        N_orig = len(self.senal_original)
        xf_orig = rfftfreq(N_orig, 1 / self.SAMPLE_RATE)
        yf_orig = rfft(self.senal_original)

        N_mod = len(self.senal_fsk)
        xf_mod = rfftfreq(N_mod, 1 / self.SAMPLE_RATE)
        yf_mod = rfft(self.senal_fsk)

        N_dem = len(self.senal_demodulada)
        xf_dem = rfftfreq(N_dem, 1 / self.SAMPLE_RATE)
        yf_dem = rfft(self.senal_demodulada)

        plt.subplot(4, 3, 7);  plt.plot(xf_orig, np.abs(yf_orig), 'b-', linewidth=1);  plt.xlim(0, 3000)
        plt.subplot(4, 3, 8);  plt.plot(xf_mod,  np.abs(yf_mod),  'r-', linewidth=1);  plt.axvline(self.f0, color='b', ls='--', alpha=0.5); plt.axvline(self.f1, color='g', ls='--', alpha=0.5); plt.xlim(0, 3000)
        plt.subplot(4, 3, 9);  plt.plot(xf_dem,  np.abs(yf_dem),  'm-', linewidth=1);  plt.xlim(0, 3000)

        # Espectrogramas (sin cambios)
        plt.subplot(4, 3, 10)
        plt.specgram(self.senal_original, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='viridis')
        plt.title("Espectrograma Original")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, 3000)
        
        plt.subplot(4, 3, 11)
        plt.specgram(self.senal_fsk, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='hot')
        plt.title("Espectrograma Modulado")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, 3000)
        
        plt.subplot(4, 3, 12)
        plt.specgram(self.senal_demodulada, NFFT=1024, Fs=self.SAMPLE_RATE, 
                     noverlap=512, cmap='plasma')
        plt.title("Espectrograma Demodulado")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Frecuencia (Hz)")
        plt.ylim(0, 3000)
        
        plt.tight_layout()
        plt.show()
    
    def mostrar_senales(self):
        """Muestra las señales en el dominio del tiempo."""
        bit_duration = 1.0 / self.bits_per_second
        samples_per_bit = int(self.SAMPLE_RATE * bit_duration)
        
        plt.figure(figsize=(14, 10))
        
        # Datos binarios
        plt.subplot(3, 1, 1)
        t_bits = np.arange(len(self.datos)) * bit_duration
        plt.step(t_bits, self.datos, where='post', linewidth=2)
        plt.title("Datos Binarios Originales")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Bit")
        plt.ylim(-0.5, 1.5)
        plt.grid(True)
        
        # Señal FSK modulada
        plt.subplot(3, 1, 2)
        t_senal = np.arange(len(self.senal_fsk)) / self.SAMPLE_RATE
        plt.plot(t_senal, self.senal_fsk)
        plt.title(f"Señal FSK Modulada (f0={self.f0}Hz, f1={self.f1}Hz)")
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Amplitud")
        plt.grid(True)
        plt.xlim(0, len(self.datos) * bit_duration)
        
        # Datos recuperados
        if hasattr(self, 'datos_recuperados'):
            plt.subplot(3, 1, 3)
            t_bits_rec = np.arange(len(self.datos_recuperados)) * bit_duration
            plt.step(t_bits_rec, self.datos_recuperados, where='post', linewidth=2, color='orange')
            plt.title("Datos Binarios Recuperados")
            plt.xlabel("Tiempo (s)")
            plt.ylabel("Bit")
            plt.ylim(-0.5, 1.5)
            plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def reproducir_audio(self, senal=None):
        """
        Reproduce la señal FSK como audio.
        
        Parámetros:
            senal: Señal a reproducir (usa self.senal_fsk si es None)
        """
        if senal is None:
            senal = self.senal_fsk
        
        # Normalizar la señal
        senal_normalizada = senal / np.max(np.abs(senal))
        
        print("Reproduciendo señal FSK...")
        sd.play(senal_normalizada, self.SAMPLE_RATE)
        sd.wait()
        print("Reproducción completada.")


# Ejemplo de uso
if __name__ == "__main__":
    # Crear instancia de FSK equivalente al ESP32
    fsk = FSK(SAMPLE_RATE=8000, duration=2.0, carpeta_salida='SENfsk')

    # Generar tono senoidal original (coherente con F0)
    print("\n=== GENERANDO SEÑAL ORIGINAL ===")
    tono_original = fsk.generar_tono_original(frecuencia=1000, amplitud=0.8)
    fsk.mostrar_senal_original()

    # Usar el mismo patrón del ESP32: {1, 0, 1, 1, 0, 1, 0, 0}
    # Patrón base del ESP32 (Prueba.ino: data_bits)
    patron_esp32 = [1, 0, 1, 1, 0, 1, 0, 0]
    
    # Repetir para llenar ~2 segundos (2s * 125 bps = 250 bits)
    num_repeticiones = (250 // len(patron_esp32)) + 1
    datos_extendidos = (patron_esp32 * num_repeticiones)[:250]
    
    print(f"\nPatron ESP32 (8 bits): {patron_esp32}")
    print(f"Repetido {num_repeticiones} veces para 250 bits (2s a 125 bps)")
    
    fsk.generar_datos_binarios(datos=datos_extendidos)

    # Modular en FSK con los mismos parámetros del micro
    print("\n=== MODULANDO SEÑAL FSK ===")
    senal_modulada = fsk.modular_fsk(f0=1000, f1=2000, bits_per_second=125, amplitud=0.8)
    fsk.mostrar_senal_modulada()

    # Demodular la señal FSK
    print("\n=== DEMODULANDO SEÑAL FSK ===")
    datos_recuperados = fsk.demodular_fsk(metodo='filtro')
    fsk.mostrar_senal_demodulada()

    # Comparar señales
    print("\n=== COMPARACION DE SEÑALES ===")
    fsk.comparar_senales()

    # Verificar que los primeros 8 bits coincidan con el ESP32
    print("\n*** VERIFICACION vs ESP32 ***")
    print(f"Patron ESP32:        {patron_esp32}")
    print(f"Python (primeros 8): {datos_extendidos[:8]}")
    if datos_extendidos[:8] == patron_esp32:
        print("[OK] Los datos coinciden con el ESP32")
    else:
        print("[ERROR] Los datos NO coinciden")

    plt.show()