/**
 * @file receptor_final_definitivo.ino
 * @brief Versión final y depurada del receptor FSK por Wi-Fi.
 * @details Utiliza un flujo de datos lineal y simplificado para el análisis
 * de FFT, eliminando funciones separadas para máxima robustez.
 */
#include <WiFi.h>
#include "arduinoFFT.h"

// Cambiar esto por los datos de tu red Wi-Fi
const char* ssid = "EL_NOMBRE_DE_TU_WIFI";
const char* password = "LA_CONTRASENA_DE_TU_WIFI";
const char* host = "XXX.XXX.XX.XXX"; // IP del la PC

const uint16_t port = 8080;

// --- Parámetros de la Señal (deben coincidir con Python) ---
#define SAMPLES 512
#define SAMPLING_FREQ 8000
#define FREQ_0 1000
#define FREQ_1 2000
#define BIT_DURATION 64
#define SYM_FFT_N 64

// --- Mensaje Original para Verificación ---
const uint8_t data_bits[] = {1, 0, 1, 0, 1, 0, 1, 0};
const int num_bits = sizeof(data_bits) / sizeof(data_bits[0]);

// --- Buffers ---
double vRealSymbol[SYM_FFT_N];
double vImagSymbol[SYM_FFT_N];

// --- Cliente Wi-Fi ---
WiFiClient client;

// --- Función de Ayuda para Imprimir ---
static void printBitsLine(const char* label, const uint8_t* bits, int n) {
  Serial.print(label); Serial.print(": ");
  for (int i = 0; i < n; ++i) Serial.print(bits[i]);
  Serial.println();
}

void connectToSocket() {
  if (client.connected()) return;
  Serial.println("\nIntentando conectar al servidor de audio...");
  while (!client.connect(host, port)) {
    Serial.println("Conexión fallida. Reintentando en 3 segundos...");
    delay(3000);
  }
  Serial.println("¡Conectado al servidor de audio!");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nConectando a Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado. IP: " + WiFi.localIP().toString());
  connectToSocket();
}

void loop() {
  if (!client.connected()) {
    connectToSocket();
  }

  if (client.available() >= SAMPLES * 2) {
    Serial.println("\n>>> Stream de audio recibido! Analizando...");
    
    uint8_t byte_buffer[SAMPLES * 2];
    client.readBytes(byte_buffer, sizeof(byte_buffer));

    // Reinterpretar el buffer de bytes como un array de audio de 16 bits
    int16_t* audio_samples = (int16_t*)byte_buffer;
    
    // --- Bucle de Análisis Principal ---
    static uint8_t demod_bits[128];
    static double f0_mags[128], f1_mags[128];
    int errors = 0;

    // Analizar el stream símbolo por símbolo (bit a bit)
    for (int k = 0; k < num_bits; ++k) {
      // 1. Copiar el segmento de 64 muestras para el bit actual a los buffers de la FFT
      int start_index = k * BIT_DURATION;
      for(int i = 0; i < SYM_FFT_N; i++) {
        vRealSymbol[i] = (double)audio_samples[start_index + i];
        vImagSymbol[i] = 0.0;
      }
      
      // 2. Crear un objeto FFT para este símbolo específico
      ArduinoFFT<double> FFT = ArduinoFFT<double>(vRealSymbol, vImagSymbol, SYM_FFT_N, SAMPLING_FREQ);
      
      // 3. Realizar el análisis
      FFT.dcRemoval(); // Usar la función DC Removal de la librería
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();
      
      // 4. Obtener magnitudes y tomar la decisión
      int b0 = (int)round((double)FREQ_0 * SYM_FFT_N / SAMPLING_FREQ);
      int b1 = (int)round((double)FREQ_1 * SYM_FFT_N / SAMPLING_FREQ);
      f0_mags[k] = vRealSymbol[b0];
      f1_mags[k] = vRealSymbol[b1];
      demod_bits[k] = (f1_mags[k] > f0_mags[k]) ? 1 : 0;
      
      if (demod_bits[k] != data_bits[k]) errors++;
    }

    // --- Imprimir Resultados ---
    Serial.println("--- ANALISIS BIT A BIT ---");
    Serial.println("Bit | F0 Mag | F1 Mag | Decision");
    Serial.println("----|--------|--------|----------");
    for (int k = 0; k < num_bits; k++) {
      Serial.printf(" %d  | %6.1f | %6.1f |    %d\n", k, f0_mags[k], f1_mags[k], demod_bits[k]);
    }
    Serial.println("\n--- COMPARACION DEL MENSAJE ---");
    printBitsLine("Original", data_bits, num_bits);
    printBitsLine("Recibido", demod_bits, num_bits);
    float ber = (float)errors / num_bits;
    Serial.printf("\nBER (Bit Error Rate): %d/%d = %.2f%%\n", errors, num_bits, ber * 100.0f);
  }
}