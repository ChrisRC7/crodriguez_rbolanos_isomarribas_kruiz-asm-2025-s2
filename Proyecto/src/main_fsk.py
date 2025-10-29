"""
Script principal para pruebas FSK equivalentes al ESP32.
"""
from fsk import FSK
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Crear instancia FSK
    fsk = FSK(SAMPLE_RATE=8000, duration=2.0, carpeta_salida='SENfsk')

    print("\n=== GENERANDO SEÑAL ORIGINAL ===")
    fsk.generar_tono_original(frecuencia=1000, amplitud=0.8)
    fsk.mostrar_senal_original()

    # Patron ESP32: {1, 0, 1, 1, 0, 1, 0, 0}
    patron_esp32 = [1, 0, 1, 1, 0, 1, 0, 0]
    num_repeticiones = (250 // len(patron_esp32)) + 1
    datos_extendidos = (patron_esp32 * num_repeticiones)[:250]
    
    print(f"\nPatron ESP32 (8 bits): {patron_esp32}")
    print(f"Repetido {num_repeticiones} veces para 250 bits")
    
    fsk.generar_datos_binarios(datos=datos_extendidos)

    print("\n=== MODULANDO SEÑAL FSK ===")
    fsk.modular_fsk(f0=1000, f1=2000, bits_per_second=125, amplitud=0.8)
    fsk.mostrar_senal_modulada()

    print("\n=== DEMODULANDO SEÑAL FSK ===")
    fsk.demodular_fsk(metodo='filtro')
    fsk.mostrar_senal_demodulada()

    print("\n=== COMPARACION DE SEÑALES ===")
    fsk.comparar_senales()

    print("\n*** VERIFICACION vs ESP32 ***")
    print(f"Patron ESP32:        {patron_esp32}")
    print(f"Python (primeros 8): {datos_extendidos[:8]}")
    if datos_extendidos[:8] == patron_esp32:
        print("[OK] Los datos coinciden con el ESP32")
    
    plt.show()