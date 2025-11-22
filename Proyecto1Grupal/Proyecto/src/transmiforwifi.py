import socket
import time
import numpy as np
import struct

# --- Parámetros de la Señal y Red ---
HOST = '0.0.0.0'  # Escuchar en todas las interfaces de red
PORT = 8080       # Puerto para que el ESP32 se conecte

# --- Parámetros FSK (deben coincidir con el ESP32) ---
FREQ_0 = 1000
FREQ_1 = 2000
SAMPLING_RATE = 8000
BIT_DURATION_SAMPLES = 64  # 8 ms * 8000 Hz / 1000 ms
AMPLITUDE = 16383  # Amplitud para audio de 16 bits (la mitad de 32767)

# --- Mensaje a Transmitir ---
MESSAGE_BITS = [1, 0, 1, 0, 1, 0, 1, 0]

def generate_fsk_signal(bits):
    """Genera la onda de audio FSK completa como un array de bytes."""
    signal = np.array([], dtype=np.int16)
    num_samples_total = len(bits) * BIT_DURATION_SAMPLES
    t = np.linspace(0, num_samples_total / SAMPLING_RATE, num_samples_total, endpoint=False)
    
    current_sample = 0
    for bit in bits:
        freq = FREQ_1 if bit == 1 else FREQ_0
        # Genera la onda para este bit
        time_segment = t[current_sample : current_sample + BIT_DURATION_SAMPLES]
        wave_segment = AMPLITUDE * np.sin(2 * np.pi * freq * time_segment)
        signal = np.append(signal, wave_segment)
        current_sample += BIT_DURATION_SAMPLES
        
    # Convertir la señal a formato de bytes de 16 bits (signed short)
    return struct.pack('<' + 'h' * len(signal), *signal.astype(np.int16))

# --- Lógica Principal del Servidor ---
print("Generando la señal FSK modulada...")
fsk_data = generate_fsk_signal(MESSAGE_BITS)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"=========================================")
    print(f"   Servidor de Audio Socket en {HOST}:{PORT}")
    print("   Esperando conexión del ESP32...")
    print(f"=========================================")
    
    conn, addr = s.accept()
    with conn:
        print(f"Conexión establecida desde {addr}")
        while True:
            try:
                print(f"Transmitiendo {len(fsk_data)} bytes de audio modulado...")
                conn.sendall(fsk_data)
                time.sleep(5)  # Esperar 5 segundos antes de retransmitir
            except (ConnectionResetError, BrokenPipeError):
                print(f"Cliente {addr} desconectado. Esperando nueva conexión...")
                conn, addr = s.accept() # Espera a que se reconecte
                print(f"Conexión re-establecida desde {addr}")
                
