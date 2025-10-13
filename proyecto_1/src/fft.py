import scipy, numpy
from matplotlib import pyplot as plt
class FFT:
        def __init__(self, normalized_tone, SAMPLE_RATE):
            self.normalized_tone = normalized_tone
            self.SAMPLE_RATE = SAMPLE_RATE

            # Número de muestras
            N = len(normalized_tone)

            # Aplicar la FFT
            yf = scipy.fft.rfft(normalized_tone)
            xf = scipy.fft.rfftfreq(N, 1 / SAMPLE_RATE)


            # --- 4. Interpretar Magnitud y Fase (graficando) ---

            print("Mostrando las gráficas de magnitud y fase. Ciérralas para continuar.")

            # Crear una figura para ambas gráficas
            plt.figure(figsize=(12, 8))

            # Gráfica de Magnitud
            plt.subplot(2, 1, 1)
            plt.plot(xf, numpy.abs(yf))
            plt.title("Espectro de Magnitud")
            plt.xlabel("Frecuencia (Hz)")
            plt.ylabel("Magnitud")
            plt.grid(True)

            # Gráfica de Fase
            plt.subplot(2, 1, 2)
            plt.plot(xf, numpy.angle(yf))
            plt.title("Espectro de Fase")
            plt.xlabel("Frecuencia (Hz)")
            plt.ylabel("Fase (radianes)")
            plt.grid(True)

            # Ajustar el espaciado y mostrar las gráficas
            plt.tight_layout()
            plt.show()