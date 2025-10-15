"""
Pregunta 5: Implementación de Modulación y Demodulación usando FFT

Este script implementa un ejemplo completo de modulación y demodulación
usando la clase FFT del archivo fft.py para filtrado en el dominio
de la frecuencia.
"""

import numpy as np
import scipy.io.wavfile
from matplotlib import pyplot as plt
import tkinter as tk
from tkinter import filedialog, messagebox
import os
import fft  # Importar la clase FFT

# ==================== PARÁMETROS DE CONFIGURACIÓN ====================
SAMPLE_RATE = 44100  # Hz - Frecuencia de muestreo
DURATION = 5         # Segundos - Duración de las señales
FREQ_DESEADA = 400   # Hz - Frecuencia de la señal deseada
FREQ_RUIDO = 4000    # Hz - Frecuencia del ruido a filtrar
AMPLITUD_RUIDO = 0.3 # Factor de amplitud del ruido (30%)

# ==================== FUNCIONES AUXILIARES ====================

def generate_sine_wave(freq, sample_rate, duration):
    """
    Genera una onda sinusoidal.
    
    Parámetros:
        freq: Frecuencia de la onda en Hz
        sample_rate: Frecuencia de muestreo en Hz
        duration: Duración de la señal en segundos
    
    Retorna:
        t: Vector de tiempo
        y: Vector de amplitudes de la señal
    """
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    y = np.sin(2 * np.pi * freq * t)
    return t, y


def normalize_audio(signal, target_amplitude=32767):
    """
    Normaliza una señal de audio a 16 bits.
    
    Parámetros:
        signal: Señal a normalizar
        target_amplitude: Amplitud objetivo (32767 para 16 bits)
    
    Retorna:
        Signal normalizada como entero de 16 bits
    """
    normalized = np.int16((signal / np.max(np.abs(signal))) * target_amplitude)
    return normalized


def plot_time_domain(signal, sample_rate, title="Señal en el Dominio del Tiempo", samples=1000):
    """
    Grafica una señal en el dominio del tiempo.
    
    Parámetros:
        signal: Señal a graficar
        sample_rate: Frecuencia de muestreo
        title: Título de la gráfica
        samples: Número de muestras a mostrar
    """
    t = np.arange(min(samples, len(signal))) / sample_rate
    plt.figure(figsize=(12, 4))
    plt.plot(t, signal[:samples], linewidth=0.5)
    plt.title(title)
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Amplitud")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def seleccionar_modo():
    """
    Muestra un diálogo para seleccionar el modo de operación.
    
    Retorna:
        'generar': Para generar señales sintéticas
        'cargar': Para cargar un archivo WAV
        None: Si se cancela
    """
    root = tk.Tk()
    root.withdraw()
    
    respuesta = messagebox.askyesnocancel(
        "Modo de Operación",
        "¿Deseas generar señales sintéticas?\n\n"
        "• SÍ: Generar señales de 400 Hz y 4000 Hz\n"
        "• NO: Cargar un archivo WAV existente\n"
        "• CANCELAR: Salir"
    )
    
    root.destroy()
    
    if respuesta is None:  # Cancelar
        return None
    elif respuesta:  # Sí
        return 'generar'
    else:  # No
        return 'cargar'


def cargar_archivo_wav():
    """
    Abre un diálogo para seleccionar un archivo WAV.
    
    Retorna:
        sample_rate: Frecuencia de muestreo del archivo
        audio_data: Datos del audio
        file_path: Ruta del archivo seleccionado
    """
    root = tk.Tk()
    root.withdraw()
    
    print("\nPor favor, selecciona un archivo de audio (.wav)")
    file_path = filedialog.askopenfilename(
        title="Selecciona un archivo WAV para demodular",
        filetypes=[("Archivos WAV", "*.wav"), ("Todos los archivos", "*.*")]
    )
    
    root.destroy()
    
    if not file_path:
        return None, None, None
    
    try:
        sample_rate, audio_data = scipy.io.wavfile.read(file_path)
        
        # Si es estéreo, usar solo el primer canal
        if audio_data.ndim > 1:
            print("El audio es estéreo, se utilizará solo el primer canal.")
            audio_data = audio_data[:, 0]
        
        return sample_rate, audio_data, file_path
    
    except Exception as e:
        print(f"Error al cargar el archivo: {e}")
        return None, None, None


def solicitar_frecuencia_filtrar():
    """
    Muestra un diálogo para ingresar la frecuencia a filtrar.
    
    Retorna:
        frecuencia: Frecuencia en Hz o None si se cancela
    """
    root = tk.Tk()
    root.withdraw()
    
    # Usar simpledialog en lugar de crear ventana personalizada
    from tkinter import simpledialog
    
    frecuencia = simpledialog.askfloat(
        "Frecuencia a Filtrar",
        "Ingresa la frecuencia (en Hz) que deseas filtrar:",
        initialvalue=4000.0,
        minvalue=0.1,
        maxvalue=22050.0
    )
    
    root.destroy()
    
    return frecuencia


# ==================== PROCESO PRINCIPAL ====================

def main():
    print("="*70)
    print("SISTEMA DE MODULACIÓN Y DEMODULACIÓN USANDO FFT")
    print("="*70)
    
    # Seleccionar modo de operación
    modo = seleccionar_modo()
    
    if modo is None:
        print("\nOperación cancelada por el usuario.")
        return
    
    signal_mixta_norm = None
    sample_rate = SAMPLE_RATE
    modo_nombre = ""
    
    if modo == 'generar':
        modo_nombre = "GENERACIÓN DE SEÑALES SINTÉTICAS"
        print(f"\n[MODO] {modo_nombre}")
        
        # -------- PASO 1: GENERACIÓN DE SEÑALES --------
        print("\n[PASO 1] Generando señales...")
        print(f"  - Señal deseada: {FREQ_DESEADA} Hz")
        print(f"  - Señal de ruido: {FREQ_RUIDO} Hz (amplitud {AMPLITUD_RUIDO*100}%)")
        
        t, signal_deseada = generate_sine_wave(FREQ_DESEADA, SAMPLE_RATE, DURATION)
        _, signal_ruido = generate_sine_wave(FREQ_RUIDO, SAMPLE_RATE, DURATION)
        
        # -------- PASO 2: MODULACIÓN (MEZCLA DE SEÑALES) --------
        print("\n[PASO 2] Mezclando señales (Modulación)...")
        signal_ruido = signal_ruido * AMPLITUD_RUIDO
        signal_mixta = signal_deseada + signal_ruido
        
        # Normalizar para guardar
        signal_mixta_norm = normalize_audio(signal_mixta)
        
        # Guardar señal mixta
        output_dir = "audio_output"
        os.makedirs(output_dir, exist_ok=True)
        mixed_path = os.path.join(output_dir, "signal_mixta.wav")
        scipy.io.wavfile.write(mixed_path, SAMPLE_RATE, signal_mixta_norm)
        print(f"  ✓ Señal mixta guardada en: {mixed_path}")
        
        # Frecuencia a filtrar
        freq_filtrar = FREQ_RUIDO
        
    else:  # modo == 'cargar'
        modo_nombre = "CARGA DE ARCHIVO WAV"
        print(f"\n[MODO] {modo_nombre}")
        
        # Cargar archivo
        sample_rate, audio_data, file_path = cargar_archivo_wav()
        
        if sample_rate is None:
            print("\nNo se seleccionó ningún archivo. Operación cancelada.")
            return
        
        print(f"\n[PASO 1] Archivo cargado exitosamente")
        print(f"  - Archivo: {file_path}")
        print(f"  - Frecuencia de muestreo: {sample_rate} Hz")
        print(f"  - Número de muestras: {len(audio_data)}")
        print(f"  - Duración: {len(audio_data)/sample_rate:.2f} segundos")
        
        signal_mixta_norm = audio_data
        
        # Solicitar frecuencia a filtrar
        print("\n[PASO 2] Solicitar frecuencia a filtrar...")
        freq_filtrar = solicitar_frecuencia_filtrar()
        
        if freq_filtrar is None:
            print("\nOperación cancelada por el usuario.")
            return
        
        print(f"  ✓ Frecuencia a filtrar: {freq_filtrar} Hz")
    
    # Mostrar señal mixta en el tiempo
    print(f"\n  Mostrando señal {'mixta' if modo == 'generar' else 'original'} en el dominio del tiempo...")
    plot_time_domain(signal_mixta_norm, sample_rate, 
                     f"Señal {'Mixta' if modo == 'generar' else 'Original'} - Dominio del Tiempo")
    
    # -------- PASO 3: APLICAR FFT USANDO LA CLASE FFT --------
    print("\n[PASO 3] Aplicando FFT usando la clase FFT...")
    
    # Crear instancia de la clase FFT (sin mostrar gráficas automáticamente)
    fft_analyzer = fft.FFT(signal_mixta_norm, sample_rate, mostrar_graficas=False)
    
    # Obtener el espectro
    xf, yf = fft_analyzer.get_spectrum()
    
    print(f"  ✓ FFT calculada: {len(yf)} bins de frecuencia")
    print(f"  ✓ Resolución frecuencial: {sample_rate/len(signal_mixta_norm):.2f} Hz/bin")
    
    # Mostrar espectro de magnitud y fase
    print("\n  Mostrando espectro de magnitud y fase...")
    fft_analyzer.mostrar_espectro()
    
    # -------- PASO 4: FILTRADO (DEMODULACIÓN) --------
    print("\n[PASO 4] Filtrando frecuencia usando la clase FFT...")
    
    # Filtrar la frecuencia objetivo
    yf_filtrado = fft_analyzer.filtrar_frecuencia(freq_filtrar, ancho_banda=1)
    
    print(f"  ✓ Componente de {freq_filtrar} Hz filtrada exitosamente")
    
    # Mostrar comparación de espectros
    print("\n  Mostrando comparación de espectros...")
    fft_analyzer.mostrar_comparacion_espectros(
        yf_filtrado,
        titulo_original="Espectro ANTES del Filtrado",
        titulo_filtrado="Espectro DESPUÉS del Filtrado"
    )
    
    # -------- PASO 5: APLICAR FFT INVERSA --------
    print("\n[PASO 5] Aplicando FFT Inversa (iFFT) usando la clase FFT...")
    
    # Reconstruir la señal usando la clase FFT
    signal_limpia = fft_analyzer.reconstruir_senal(yf_filtrado)
    
    print(f"  ✓ Señal reconstruida: {len(signal_limpia)} muestras")
    
    # Normalizar señal limpia
    signal_limpia_norm = normalize_audio(signal_limpia)
    
    # Guardar señal limpia
    output_dir = "audio_output"
    os.makedirs(output_dir, exist_ok=True)
    clean_path = os.path.join(output_dir, "signal_limpia.wav")
    scipy.io.wavfile.write(clean_path, sample_rate, signal_limpia_norm)
    print(f"  ✓ Señal limpia guardada en: {clean_path}")
    
    # Mostrar señal limpia en el tiempo
    print("\n  Mostrando señal limpia en el dominio del tiempo...")
    plot_time_domain(signal_limpia_norm, sample_rate,
                     "Señal Limpia (Demodulada) - Dominio del Tiempo")
    
    # -------- PASO 6: COMPARACIÓN FINAL --------
    print("\n[PASO 6] Comparación de señales...")
    
    # Crear comparación visual
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    samples = min(1000, len(signal_mixta_norm))
    t = np.arange(samples) / sample_rate
    
    # Señal original/mixta
    axes[0].plot(t, signal_mixta_norm[:samples], 'r-', linewidth=0.5)
    axes[0].set_title(f"Señal {'Mixta' if modo == 'generar' else 'Original'}")
    axes[0].set_ylabel("Amplitud")
    axes[0].grid(True, alpha=0.3)
    
    # Señal recuperada
    axes[1].plot(t, signal_limpia_norm[:samples], 'b-', linewidth=0.5)
    axes[1].set_title("Señal Recuperada (Demodulada)")
    axes[1].set_xlabel("Tiempo (s)")
    axes[1].set_ylabel("Amplitud")
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # -------- RESUMEN FINAL --------
    print("\n" + "="*70)
    print("RESUMEN DE RESULTADOS")
    print("="*70)
    print(f"Modo de operación: {modo_nombre}")
    print(f"✓ Frecuencia de muestreo: {sample_rate} Hz")
    print(f"✓ Frecuencia filtrada: {freq_filtrar} Hz")
    print(f"✓ Filtrado exitoso usando la clase FFT")
    print(f"✓ Señal recuperada mediante iFFT")
    print(f"\nArchivos generados en '{output_dir}/':")
    if modo == 'generar':
        print(f"  - signal_mixta.wav (señal modulada)")
    print(f"  - signal_limpia.wav (señal demodulada)")
    print("\n¡Proceso completado exitosamente!")
    print("="*70)


if __name__ == "__main__":
    main()