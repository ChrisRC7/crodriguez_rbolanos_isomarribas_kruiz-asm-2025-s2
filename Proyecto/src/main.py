"""
Main.py - Sistema de Análisis FFT y Modulación FSK
Menú principal con interfaz gráfica para acceder a las diferentes funcionalidades
"""

import fft
import tkinter as tk
from tkinter import filedialog, messagebox
from scipy.io import wavfile
from matplotlib import pyplot as plt
import subprocess
import sys
import os

class MainApp:
    def __init__(self):
        # Crear ventana principal
        self.root = tk.Tk()
        self.root.title("Sistema de Procesamiento Digital de Señales")
        self.root.geometry("800x500")
        self.root.minsize(800, 500)
        self.root.resizable(False, False)
        
        # Configurar estilo
        self.root.configure(bg='#2c3e50')
        
        # Crear interfaz
        self.crear_interfaz()
        
    def crear_interfaz(self):
        """Crea todos los elementos de la interfaz gráfica"""
        
        # Título principal
        titulo = tk.Label(
            self.root,
            text="Sistema de Procesamiento Digital de Señales",
            font=("Helvetica", 18, "bold"),
            bg='#2c3e50',
            fg='white',
            pady=20
        )
        titulo.pack()
        
        # Subtítulo
        subtitulo = tk.Label(
            self.root,
            text="Análisis FFT y Modulación FSK",
            font=("Helvetica", 12),
            bg='#2c3e50',
            fg='#ecf0f1',
            pady=5
        )
        subtitulo.pack()
        
        # Frame para los botones
        frame_botones = tk.Frame(self.root, bg="#2c3f52")
        frame_botones.pack(pady=40)
        
        # Botón Pregunta 2 (FFT)
        btn_pregunta2 = tk.Button(
            frame_botones,
            text="Análisis FFT\nde Audio",
            command=self.ejecutar_pregunta2,
            font=("Helvetica", 14, "bold"),
            bg='#3498db',
            fg='white',
            activebackground='#2980b9',
            activeforeground='white',
            width=20,
            height=4,
            relief=tk.RAISED,
            bd=3,
            cursor="hand2"
        )
        btn_pregunta2.grid(row=0, column=0, padx=30)
        
        # Botón FSK
        btn_fsk = tk.Button(
            frame_botones,
            text="Modulación FSK\n(Frequency Shift Keying)",
            command=self.ejecutar_fsk,
            font=("Helvetica", 14, "bold"),
            bg='#e74c3c',
            fg='white',
            activebackground='#c0392b',
            activeforeground='white',
            width=20,
            height=4,
            relief=tk.RAISED,
            bd=3,
            cursor="hand2"
        )
        btn_fsk.grid(row=0, column=1, padx=30)
        
        # Descripción de funcionalidades
        frame_descripciones = tk.Frame(self.root, bg='#2c3e50')
        frame_descripciones.pack(pady=20)
        
        desc_pregunta2 = tk.Label(
            frame_descripciones,
            text="• Carga un archivo WAV\n• Muestra espectro de magnitud y fase\n• Análisis FFT completo",
            font=("Helvetica", 10),
            bg='#2c3e50',
            fg='#bdc3c7',
            justify=tk.LEFT
        )
        desc_pregunta2.grid(row=0, column=0, padx=40)
        
        desc_fsk = tk.Label(
            frame_descripciones,
            text="• Genera señal senoidal\n• Modula en FSK\n• Demodula y compara señales",
            font=("Helvetica", 10),
            bg='#2c3e50',
            fg='#bdc3c7',
            justify=tk.LEFT
        )
        desc_fsk.grid(row=0, column=1, padx=40)
        
        # Botón Salir
        btn_salir = tk.Button(
            self.root,
            text="Salir",
            command=self.salir,
            font=("Helvetica", 10),
            bg='#95a5a6',
            fg='white',
            activebackground='#7f8c8d',
            width=15,
            height=1,
            cursor="hand2"
        )
        btn_salir.pack(pady=10)
        
        # Footer
        footer = tk.Label(
            self.root,
            text="Instituto Tecnológico de Costa Rica - Escuela de Ingeniería en Computadores",
            font=("Helvetica", 8),
            bg='#2c3e50',
            fg='#7f8c8d'
        )
        footer.pack(side=tk.BOTTOM, pady=10)
    
    def ejecutar_pregunta2(self):
        """Ejecuta la funcionalidad de la Pregunta 2 (Análisis FFT)"""
        print("\n" + "="*60)
        print("ANÁLISIS FFT DE AUDIO")
        print("="*60)
        
        # Abrir explorador de archivos
        print("Por favor, selecciona un archivo de audio (.wav)")
        file_path = filedialog.askopenfilename(
            title="Selecciona tu pista de audio",
            filetypes=[("Archivos WAV", "*.wav"), ("Todos los archivos", "*.*")]
        )
        
        # Si no se selecciona archivo
        if not file_path:
            messagebox.showwarning("Advertencia", "No se seleccionó ningún archivo.")
            return
        
        print(f"Archivo seleccionado: {file_path}")
        
        try:
            # Cargar el archivo de audio
            SAMPLE_RATE, audio_data = wavfile.read(file_path)
            
            # Si es estéreo, usar solo el primer canal
            if audio_data.ndim > 1:
                print("El audio es estéreo, se utilizará solo el primer canal.")
                normalized_tone = audio_data[:, 0]
            else:
                normalized_tone = audio_data
            
            # Ejecutar análisis FFT usando la clase FFT
            print("Calculando FFT...")
            fft.FFT(normalized_tone, SAMPLE_RATE)
            
            print("Análisis completado exitosamente")
            messagebox.showinfo("Éxito", "Análisis FFT completado. Las gráficas se han mostrado.")
            
        except Exception as e:
            print(f"Error al procesar el archivo: {e}")
            messagebox.showerror("Error", f"No se pudo procesar el archivo:\n{e}")
    
    def ejecutar_fsk(self):
        """Ejecuta la funcionalidad de Modulación FSK"""
        print("\n" + "="*60)
        print("MODULACIÓN FSK (FREQUENCY SHIFT KEYING)")
        print("="*60)
        
        try:
            # Obtener el directorio donde está main.py
            script_dir = os.path.dirname(os.path.abspath(__file__))
            fsk_path = os.path.join(script_dir, "FSK.py")
            
            # Verificar si existe el archivo FSK.py
            if not os.path.exists(fsk_path):
                print(f"Directorio del script: {script_dir}")
                print(f"Buscando: {fsk_path}")
                print(f"Archivos en el directorio: {os.listdir(script_dir)}")
                messagebox.showerror(
                    "Error", 
                    f"No se encontró el archivo 'FSK.py'.\n\n"
                    f"Directorio actual:\n{script_dir}\n\n"
                    f"Asegúrate de que 'FSK.py' esté en el mismo directorio que main.py"
                )
                return
            
            print("Ejecutando FSK.py...")
            print("Se generarán señales y gráficas de análisis FSK.\n")
            
            # Ejecutar FSK.py como subproceso desde el directorio correcto
            resultado = subprocess.run(
                [sys.executable, fsk_path],
                capture_output=True,
                text=True,
                cwd=script_dir  # Ejecutar desde el directorio del script
            )
            
            # Mostrar salida en consola
            if resultado.stdout:
                print(resultado.stdout)
            
            if resultado.stderr:
                print("Errores:", resultado.stderr)
            
            if resultado.returncode == 0:
                print(" Proceso completado exitosamente")
                messagebox.showinfo(
                    "Éxito",
                    "Modulación FSK completada.\n"
                    "Revisa la carpeta 'SENfsk' para los archivos de audio generados:\n\n"
                    "• seno_original.wav\n"
                    "• fsk_modulado.wav\n"
                    "• fsk_demodulado.wav"
                )
            else:
                messagebox.showerror(
                    "Error",
                    f"Hubo un error al ejecutar FSK.py\n"
                    f"Código de retorno: {resultado.returncode}"
                )
                
        except Exception as e:
            print(f"Error al ejecutar FSK.py: {e}")
            messagebox.showerror("Error", f"No se pudo ejecutar el proceso:\n{e}")
    
    def salir(self):
        """Cierra la aplicación"""
        respuesta = messagebox.askyesno(
            "Salir",
            "¿Estás seguro de que deseas salir?"
        )
        if respuesta:
            print("\nCerrando aplicación...")
            self.root.destroy()
    
    def run(self):
        """Inicia el loop principal de la aplicación"""
        self.root.mainloop()


if __name__ == "__main__":
    print("="*60)
    print("SISTEMA DE PROCESAMIENTO DIGITAL DE SEÑALES")
    print("Análisis FFT y Modulación FSK")
    print("="*60)
    print("\nIniciando interfaz gráfica...")
    
    app = MainApp()
    app.run()
    
    print("\nAplicación cerrada.")