import fft
import tkinter as tk
from tkinter import filedialog
from scipy.io import wavfile
from matplotlib import pyplot as plt

class Gui():

    def __init__(self):

        self.root = tk.Tk()
        self.root.withdraw()

    def abrir_explorador(self):  

        # Abrir el explorador de archivos para que el usuario seleccione un archivo .wav
        print("Por favor, selecciona un archivo de audio (.wav) en la ventana que aparecerá.")
        file_path = filedialog.askopenfilename(
            title="Selecciona tu pista de audio",
            filetypes=[("Archivos WAV", "*.wav"), ("Todos los archivos", "*.*")]
        )

        # Si el usuario no selecciona un archivo, el programa termina
        if not file_path:
            print("No se seleccionó ningún archivo. Saliendo del programa.")
            exit()

        print(f"Archivo seleccionado: {file_path}")

        # --- 2. Cargar el archivo de audio seleccionado ---
        try:
            SAMPLE_RATE, audio_data = wavfile.read(file_path)
        except Exception as e:
            print(f"Error al leer el archivo de audio: {e}")
            exit()

        # Si el audio es estéreo, trabajamos solo con el primer canal
        if audio_data.ndim > 1:
            print("El audio es estéreo, se utilizará solo el primer canal.")
            normalized_tone = audio_data[:, 0]
        else:
            normalized_tone = audio_data

        fft.FFT(normalized_tone, SAMPLE_RATE)

if __name__ == "__main__":
    gui = Gui()
    gui.abrir_explorador()

        