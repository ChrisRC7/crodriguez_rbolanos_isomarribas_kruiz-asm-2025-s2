import numpy as np
import wave
import struct
import os
from datetime import datetime

class GeneradorSonidos:
    def __init__(self, SAMPLE_RATE=44100, carpeta_salida="sonidos_prueba"):
        """
        Clase para generar archivos WAV de prueba.
        
        Parámetros:
            SAMPLE_RATE: Frecuencia de muestreo en Hz (por defecto 44100 Hz)
            carpeta_salida: Carpeta donde se guardarán los archivos WAV
        """
        self.SAMPLE_RATE = SAMPLE_RATE
        self.carpeta_salida = carpeta_salida
        
        # Crear carpeta si no existe
        if not os.path.exists(carpeta_salida):
            os.makedirs(carpeta_salida)
            print(f"Carpeta '{carpeta_salida}' creada.")
    
    def generar_tono_senoidal(self, frecuencia, duracion, amplitud=0.5, nombre_archivo=None):
        """
        Genera un tono senoidal puro.
        
        Parámetros:
            frecuencia: Frecuencia del tono en Hz
            duracion: Duración en segundos
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo (si es None, se genera automáticamente)
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        senal = amplitud * np.sin(2 * np.pi * frecuencia * t)
        
        if nombre_archivo is None:
            nombre_archivo = f"tono_senoidal_{frecuencia}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Tono senoidal generado: {frecuencia} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_tono_cuadrado(self, frecuencia, duracion, amplitud=0.5, nombre_archivo=None):
        """
        Genera un tono de onda cuadrada.
        
        Parámetros:
            frecuencia: Frecuencia del tono en Hz
            duracion: Duración en segundos
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        senal = amplitud * np.sign(np.sin(2 * np.pi * frecuencia * t))
        
        if nombre_archivo is None:
            nombre_archivo = f"tono_cuadrado_{frecuencia}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Tono cuadrado generado: {frecuencia} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_tono_triangular(self, frecuencia, duracion, amplitud=0.5, nombre_archivo=None):
        """
        Genera un tono de onda triangular.
        
        Parámetros:
            frecuencia: Frecuencia del tono en Hz
            duracion: Duración en segundos
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        # Onda triangular usando arcsin(sin(x))
        senal = amplitud * (2 / np.pi) * np.arcsin(np.sin(2 * np.pi * frecuencia * t))
        
        if nombre_archivo is None:
            nombre_archivo = f"tono_triangular_{frecuencia}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Tono triangular generado: {frecuencia} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_tono_diente_sierra(self, frecuencia, duracion, amplitud=0.5, nombre_archivo=None):
        """
        Genera un tono de onda diente de sierra.
        
        Parámetros:
            frecuencia: Frecuencia del tono en Hz
            duracion: Duración en segundos
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        # Onda diente de sierra
        senal = amplitud * 2 * (t * frecuencia - np.floor(t * frecuencia + 0.5))
        
        if nombre_archivo is None:
            nombre_archivo = f"tono_diente_sierra_{frecuencia}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Tono diente de sierra generado: {frecuencia} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_chirp(self, freq_inicial, freq_final, duracion, amplitud=0.5, nombre_archivo=None):
        """
        Genera un chirp (barrido de frecuencia lineal).
        
        Parámetros:
            freq_inicial: Frecuencia inicial en Hz
            freq_final: Frecuencia final en Hz
            duracion: Duración en segundos
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        # Chirp lineal
        fase = 2 * np.pi * (freq_inicial * t + (freq_final - freq_inicial) * t**2 / (2 * duracion))
        senal = amplitud * np.sin(fase)
        
        if nombre_archivo is None:
            nombre_archivo = f"chirp_{freq_inicial}Hz_a_{freq_final}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Chirp generado: {freq_inicial} Hz a {freq_final} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_multitono(self, frecuencias, duracion, amplitudes=None, nombre_archivo=None):
        """
        Genera un tono con múltiples frecuencias simultáneas.
        
        Parámetros:
            frecuencias: Lista de frecuencias en Hz
            duracion: Duración en segundos
            amplitudes: Lista de amplitudes (si es None, todas son iguales)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        if amplitudes is None:
            amplitudes = [1.0 / len(frecuencias)] * len(frecuencias)
        
        t = np.linspace(0, duracion, int(self.SAMPLE_RATE * duracion), endpoint=False)
        senal = np.zeros_like(t)
        
        for freq, amp in zip(frecuencias, amplitudes):
            senal += amp * np.sin(2 * np.pi * freq * t)
        
        # Normalizar para evitar clipping
        senal = senal / np.max(np.abs(senal)) * 0.8
        
        if nombre_archivo is None:
            freqs_str = "_".join([str(f) for f in frecuencias])
            nombre_archivo = f"multitono_{freqs_str}Hz_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Multitono generado: {frecuencias} Hz, {duracion} s")
        
        return ruta_archivo, senal
    
    def generar_fsk_prueba(self, datos_binarios, freq_0, freq_1, duracion_bit, 
                          tipo_onda='senoidal', amplitud=0.5, nombre_archivo=None):
        """
        Genera una señal FSK de prueba con diferentes tipos de onda.
        
        Parámetros:
            datos_binarios: Lista de bits (0s y 1s)
            freq_0: Frecuencia para bit 0 en Hz
            freq_1: Frecuencia para bit 1 en Hz
            duracion_bit: Duración de cada bit en segundos
            tipo_onda: 'senoidal', 'cuadrada', 'triangular', 'diente_sierra'
            amplitud: Amplitud de la señal (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        muestras_por_bit = int(self.SAMPLE_RATE * duracion_bit)
        senal = np.array([])
        
        for bit in datos_binarios:
            t = np.linspace(0, duracion_bit, muestras_por_bit, endpoint=False)
            freq = freq_0 if bit == 0 else freq_1
            
            if tipo_onda == 'senoidal':
                segmento = amplitud * np.sin(2 * np.pi * freq * t)
            elif tipo_onda == 'cuadrada':
                segmento = amplitud * np.sign(np.sin(2 * np.pi * freq * t))
            elif tipo_onda == 'triangular':
                segmento = amplitud * (2 / np.pi) * np.arcsin(np.sin(2 * np.pi * freq * t))
            elif tipo_onda == 'diente_sierra':
                segmento = amplitud * 2 * (t * freq - np.floor(t * freq + 0.5))
            else:
                raise ValueError(f"Tipo de onda '{tipo_onda}' no reconocido")
            
            senal = np.concatenate([senal, segmento])
        
        if nombre_archivo is None:
            bits_str = "".join([str(b) for b in datos_binarios])
            nombre_archivo = f"fsk_{tipo_onda}_{bits_str}_{freq_0}Hz_{freq_1}Hz.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        duracion_total = len(datos_binarios) * duracion_bit
        print(f"FSK {tipo_onda} generado: {datos_binarios}, f0={freq_0} Hz, f1={freq_1} Hz, duración={duracion_total:.2f}s")
        
        return ruta_archivo, senal
    
    def generar_ruido_blanco(self, duracion, amplitud=0.1, nombre_archivo=None):
        """
        Genera ruido blanco gaussiano.
        
        Parámetros:
            duracion: Duración en segundos
            amplitud: Amplitud del ruido (0.0 a 1.0)
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
            senal: Array numpy con la señal generada
        """
        num_muestras = int(self.SAMPLE_RATE * duracion)
        senal = amplitud * np.random.randn(num_muestras)
        
        if nombre_archivo is None:
            nombre_archivo = f"ruido_blanco_{duracion}s.wav"
        
        ruta_archivo = self._guardar_wav(senal, nombre_archivo)
        print(f"Ruido blanco generado: {duracion} s")
        
        return ruta_archivo, senal
    
    def _guardar_wav(self, senal, nombre_archivo):
        """
        Guarda una señal como archivo WAV.
        
        Parámetros:
            senal: Array numpy con la señal
            nombre_archivo: Nombre del archivo
            
        Retorna:
            ruta_archivo: Ruta completa del archivo guardado
        """
        ruta_archivo = os.path.join(self.carpeta_salida, nombre_archivo)
        
        # Normalizar señal a rango [-1, 1]
        senal_normalizada = senal / np.max(np.abs(senal)) if np.max(np.abs(senal)) > 0 else senal
        
        # Convertir a 16-bit PCM
        senal_int16 = np.int16(senal_normalizada * 32767)
        
        # Guardar archivo WAV
        with wave.open(ruta_archivo, 'w') as wav_file:
            # Parámetros: nchannels, sampwidth, framerate, nframes, comptype, compname
            wav_file.setparams((1, 2, self.SAMPLE_RATE, len(senal_int16), 'NONE', 'not compressed'))
            wav_file.writeframes(senal_int16.tobytes())
        
        return ruta_archivo
    
    def generar_conjunto_pruebas_fsk(self):
        """
        Genera un conjunto completo de archivos de prueba para FSK de ~5 segundos cada uno.
        """
        print("\n=== Generando conjunto de pruebas FSK (5 segundos) ===\n")
        
        # Parámetros FSK para obtener ~5 segundos
        freq_0 = 1000  # 1 kHz para bit 0
        freq_1 = 2000  # 2 kHz para bit 1
        duracion_bit = 0.25  # 250 ms por bit (20 bits = 5 segundos)
        
        # Datos de prueba (20 bits cada uno para 5 segundos)
        datos_simple = [1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1]
        datos_alternado = [1, 0] * 10
        datos_todos_ceros = [0] * 20
        datos_todos_unos = [1] * 20
        datos_grupos_ceros = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1]
        
        # Generar FSK con diferentes tipos de onda
        tipos_onda = ['senoidal', 'cuadrada', 'triangular', 'diente_sierra']
        
        print("Generando señales FSK con diferentes tipos de onda...")
        for tipo in tipos_onda:
            self.generar_fsk_prueba(datos_simple, freq_0, freq_1, duracion_bit, 
                                   tipo_onda=tipo, nombre_archivo=f"fsk_{tipo}_5seg.wav")
        
        # Generar tonos individuales para calibración (5 segundos cada uno)
        print("\nGenerando tonos de calibración (5 segundos)...")
        self.generar_tono_senoidal(freq_0, 5.0, nombre_archivo="calibracion_freq0_1000Hz_5seg.wav")
        self.generar_tono_senoidal(freq_1, 5.0, nombre_archivo="calibracion_freq1_2000Hz_5seg.wav")
        
        # Generar patrones especiales (5 segundos cada uno)
        print("\nGenerando patrones especiales (5 segundos)...")
        self.generar_fsk_prueba(datos_alternado, freq_0, freq_1, duracion_bit, 
                               nombre_archivo="fsk_patron_alternado_5seg.wav")
        self.generar_fsk_prueba(datos_todos_ceros, freq_0, freq_1, duracion_bit, 
                               nombre_archivo="fsk_patron_ceros_5seg.wav")
        self.generar_fsk_prueba(datos_todos_unos, freq_0, freq_1, duracion_bit, 
                               nombre_archivo="fsk_patron_unos_5seg.wav")
        self.generar_fsk_prueba(datos_grupos_ceros, freq_0, freq_1, duracion_bit, 
                               nombre_archivo="fsk_patron_grupos_5seg.wav")
        
        # Generar chirp entre las frecuencias FSK (5 segundos)
        print("\nGenerando chirp de prueba (5 segundos)...")
        self.generar_chirp(freq_0, freq_1, 5.0, nombre_archivo="chirp_1000_2000Hz_5seg.wav")
        
        # Generar multitono (5 segundos)
        print("\nGenerando multitono de prueba (5 segundos)...")
        self.generar_multitono([freq_0, freq_1], 5.0, nombre_archivo="multitono_fsk_5seg.wav")
        self.generar_multitono([freq_0, freq_1, 1500], 5.0, nombre_archivo="multitono_tres_freqs_5seg.wav")
        
        # Generar ruido blanco (5 segundos)
        print("\nGenerando ruido blanco de prueba (5 segundos)...")
        self.generar_ruido_blanco(5.0, amplitud=0.1, nombre_archivo="ruido_blanco_5seg.wav")
        
        # Generar tonos adicionales de diferentes tipos (5 segundos cada uno)
        print("\nGenerando tonos de diferentes formas (5 segundos)...")
        self.generar_tono_cuadrado(1000, 5.0, nombre_archivo="onda_cuadrada_1000Hz_5seg.wav")
        self.generar_tono_triangular(1000, 5.0, nombre_archivo="onda_triangular_1000Hz_5seg.wav")
        self.generar_tono_diente_sierra(1000, 5.0, nombre_archivo="onda_diente_sierra_1000Hz_5seg.wav")
        
        print(f"\n=== Conjunto de pruebas de 5 segundos generado en '{self.carpeta_salida}' ===")
        print(f"Total de archivos: Se generaron múltiples archivos WAV de ~5 segundos")


# Ejemplo de uso
if __name__ == "__main__":
    print("=== Generador de Sonidos de Prueba para FSK (5 segundos) ===\n")
    
    # Crear generador
    generador = GeneradorSonidos(SAMPLE_RATE=44100, carpeta_salida="sonidos_prueba_fsk")
    
    # Generar conjunto completo de pruebas de 5 segundos
    generador.generar_conjunto_pruebas_fsk()
    
    # Ejemplos individuales adicionales
    print("\n--- Ejemplos adicionales personalizados ---\n")
    
    # Tono senoidal de 5 segundos
    generador.generar_tono_senoidal(440, 5.0, nombre_archivo="la_440Hz_5seg.wav")
    
    # FSK personalizado de 5 segundos (25 bits a 0.2s cada uno)
    datos_personalizados = [1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0]
    generador.generar_fsk_prueba(
        datos_personalizados, 
        freq_0=800, 
        freq_1=1600, 
        duracion_bit=0.2,
        tipo_onda='senoidal',
        nombre_archivo="fsk_personalizado_5seg.wav"
    )
    
    # Chirp extendido de 5 segundos
    generador.generar_chirp(500, 3000, 5.0, nombre_archivo="chirp_500_3000Hz_5seg.wav")
    
    print("\n=== Generación completada ===")
    print("Todos los archivos tienen aproximadamente 5 segundos de duración")