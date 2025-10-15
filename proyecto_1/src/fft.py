import scipy, numpy
from matplotlib import pyplot as plt

class FFT:
    def __init__(self, normalized_tone, SAMPLE_RATE, mostrar_graficas=True):
        """
        Clase para calcular y visualizar la FFT de una señal.
        
        Parámetros:
            normalized_tone: Señal de audio normalizada
            SAMPLE_RATE: Frecuencia de muestreo en Hz
            mostrar_graficas: Si True, muestra las gráficas automáticamente
        """
        self.normalized_tone = normalized_tone
        self.SAMPLE_RATE = SAMPLE_RATE

        # Número de muestras
        N = len(normalized_tone)

        # Aplicar la FFT
        self.yf = scipy.fft.rfft(normalized_tone)
        self.xf = scipy.fft.rfftfreq(N, 1 / SAMPLE_RATE)
        
        # Mostrar gráficas si se solicita
        if mostrar_graficas:
            self.mostrar_espectro()
    
    def mostrar_espectro(self):
        """Muestra las gráficas de magnitud y fase del espectro."""
        print("Mostrando las gráficas de magnitud y fase. Ciérralas para continuar.")

        # Crear una figura para ambas gráficas
        plt.figure(figsize=(12, 8))

        # Gráfica de Magnitud
        plt.subplot(2, 1, 1)
        plt.plot(self.xf, numpy.abs(self.yf))
        plt.title("Espectro de Magnitud")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True)

        # Gráfica de Fase
        plt.subplot(2, 1, 2)
        plt.plot(self.xf, numpy.angle(self.yf))
        plt.title("Espectro de Fase")
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Fase (radianes)")
        plt.grid(True)

        # Ajustar el espaciado y mostrar las gráficas
        plt.tight_layout()
        plt.show()
    
    def get_spectrum(self):
        """
        Retorna el espectro calculado.
        
        Retorna:
            xf: Vector de frecuencias
            yf: Vector de coeficientes FFT (complejos)
        """
        return self.xf, self.yf
    
    def filtrar_frecuencia(self, frecuencia_objetivo, ancho_banda=1):
        """
        Filtra una frecuencia específica del espectro.
        
        Parámetros:
            frecuencia_objetivo: Frecuencia en Hz a filtrar
            ancho_banda: Número de bins a cada lado del objetivo a filtrar
        
        Retorna:
            yf_filtrado: Espectro filtrado
        """
        # Calcular el índice correspondiente a la frecuencia objetivo
        points_per_freq = len(self.xf) / (self.SAMPLE_RATE / 2)
        target_idx = int(points_per_freq * frecuencia_objetivo)
        
        # Crear copia del espectro
        yf_filtrado = self.yf.copy()
        
        # Anular la componente objetivo
        inicio = max(0, target_idx - ancho_banda)
        fin = min(len(yf_filtrado), target_idx + ancho_banda + 1)
        yf_filtrado[inicio:fin] = 0
        
        print(f"Frecuencia {frecuencia_objetivo} Hz filtrada (bins {inicio} a {fin-1})")
        
        return yf_filtrado
    
    def reconstruir_senal(self, yf_filtrado):
        """
        Reconstruye la señal en el dominio del tiempo usando iFFT.
        
        Parámetros:
            yf_filtrado: Espectro filtrado
        
        Retorna:
            señal reconstruida
        """
        return scipy.fft.irfft(yf_filtrado)
    
    def mostrar_comparacion_espectros(self, yf_filtrado, titulo_original="Espectro Original", 
                                     titulo_filtrado="Espectro Filtrado"):
        """
        Muestra una comparación entre el espectro original y el filtrado.
        
        Parámetros:
            yf_filtrado: Espectro filtrado
            titulo_original: Título para el espectro original
            titulo_filtrado: Título para el espectro filtrado
        """
        plt.figure(figsize=(12, 8))
        
        # Espectro original
        plt.subplot(2, 1, 1)
        plt.plot(self.xf, numpy.abs(self.yf))
        plt.title(titulo_original)
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True, alpha=0.3)
        
        # Espectro filtrado
        plt.subplot(2, 1, 2)
        plt.plot(self.xf, numpy.abs(yf_filtrado))
        plt.title(titulo_filtrado)
        plt.xlabel("Frecuencia (Hz)")
        plt.ylabel("Magnitud")
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()