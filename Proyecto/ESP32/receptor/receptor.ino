/**
 * @file esp32_receptor_final.ino
 * @brief Receptor FSK con FFT para ESP32.
 * @details Esta versión final solo imprime en la terminal cuando detecta un
 * mensaje que no es todo ceros. Cuando lo hace, muestra un análisis
 * detallado bit a bit con las magnitudes de frecuencia (F0 y F1)
 * y la comparación final.
 */

#include "arduinoFFT.h"

// --- Parámetros de Hardware y Señal ---
#define ADC_PIN 34
#define SAMPLES 512
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Parámetros FSK ---
#define FREQ_0 1000
#define FREQ_1 2000
#define BIT_DURATION 64 // Muestras por símbolo (bit)
#define SYM_FFT_N 64    // Tamaño de la FFT para cada símbolo

// --- Mensaje Original (para verificar errores) ---
const uint8_t data_bits[] = {1, 0, 1, 0, 1, 0, 1, 0};
const int num_bits = sizeof(data_bits) / sizeof(data_bits[0]);

// --- Buffers y Objeto FFT ---
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// --- Máquina de Estados ---
enum State { WAITING_FOR_SIGNAL, SAMPLING, COMPUTING, DISPLAYING };
State current_state = WAITING_FOR_SIGNAL;
const double SIGNAL_THRESHOLD = 500.0; // Umbral para iniciar la captura

// --- Buffers para Resultados ---
static uint8_t demod_bits[128];
static double f0_mags[128]; // Para guardar las magnitudes de F0
static double f1_mags[128]; // Para guardar las magnitudes de F1

// -------------------- INICIO DE FUNCIONES DE AYUDA --------------------

/**
 * @brief Realiza una FFT en un pequeño segmento de la señal para demodular un bit.
 * @param in Puntero al inicio de las muestras del símbolo.
 * @param bit_out El bit resultante (0 o 1).
 * @param mag0 La magnitud calculada para la frecuencia F0.
 * @param mag1 La magnitud calculada para la frecuencia F1.
 */
static void demodulateSymbolFFT(const double* in, int& bit_out, double& mag0, double& mag1) {
  static double r[SYM_FFT_N];
  static double im[SYM_FFT_N];

  // Copia el segmento de la señal al buffer local.
  // El offset de DC ya fue removido previamente.
  for (int i = 0; i < SYM_FFT_N; i++) {
    r[i] = in[i];
    im[i] = 0.0;
  }

  ArduinoFFT<double> f(r, im, SYM_FFT_N, SAMPLING_FREQ);
  
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  f.compute(FFTDirection::Forward);
  f.complexToMagnitude();

  int b0 = (int)round((double)FREQ_0 * SYM_FFT_N / SAMPLING_FREQ); // Bin para 1000 Hz
  int b1 = (int)round((double)FREQ_1 * SYM_FFT_N / SAMPLING_FREQ); // Bin para 2000 Hz

  mag0 = r[b0];
  mag1 = r[b1];

  bit_out = (mag1 > mag0) ? 1 : 0;
}

/**
 * @brief Imprime una línea de bits en el monitor serie.
 */
static void printBitsLine(const char* label, const uint8_t* bits, int n) {
  Serial.print(label);
  Serial.print(": ");
  for (int i = 0; i < n; ++i) Serial.print(bits[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  Serial.println("\n\n========================================");
  Serial.println("    Receptor FSK con Desglose Bit a Bit");
  Serial.println("========================================");
  Serial.println("Verifique la conexion GND-GND y del pin de señal.");
  Serial.println("Esperando mensajes validos...");
}

void loop() {
  switch (current_state) {
    case WAITING_FOR_SIGNAL:
      if (abs((double)analogRead(ADC_PIN) - 2048.0) > SIGNAL_THRESHOLD) {
        current_state = SAMPLING;
      }
      break;

    case SAMPLING: {
      //Serial.println("\n>>> Señal detectada! Iniciando muestreo...");
      unsigned long next_t = micros();
      for (int i = 0; i < SAMPLES; i++) {
        while (micros() < next_t) { /* esperar */ }
        vReal[i] = (double)analogRead(ADC_PIN); // Leer valor crudo
        vImag[i] = 0.0;
        next_t += SAMPLING_PERIOD_US;
      }
      current_state = COMPUTING;
      break;
    }

    case COMPUTING: {
      double mean = 0;
      for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
      mean /= SAMPLES;
      for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;
      current_state = DISPLAYING;
      break;
    }

    case DISPLAYING: {
      const int sps = BIT_DURATION;
      const int total_symbols = min(num_bits, (int)(SAMPLES / sps));
      int errors = 0;
      bool received_all_zeros = true;

      // 1. Decodificar todos los bits y guardar los resultados y magnitudes
      for (int k = 0; k < total_symbols; ++k) {
        int start_idx = k * sps;
        int bit_est = 0;
        
        demodulateSymbolFFT(&vReal[start_idx], bit_est, f0_mags[k], f1_mags[k]);
        
        demod_bits[k] = (uint8_t)bit_est;
        
        if (bit_est == 1) {
          received_all_zeros = false;
        }
        
        if (k < num_bits && bit_est != data_bits[k]) errors++;
      }

      // 2. Si el mensaje NO fue todo ceros, imprimir todo el análisis
      if (!received_all_zeros) {
        Serial.println("\n--- MENSAJE VALIDO RECIBIDO ---");
        Serial.println("--- ANALISIS BIT A BIT ---");
        Serial.println("Bit | F0 Mag | F1 Mag | Decision");
        Serial.println("----|--------|--------|----------");

        for (int k = 0; k < total_symbols; k++) {
          Serial.printf(" %d  | %6.1f | %6.1f |    %d\n", k, f0_mags[k], f1_mags[k], demod_bits[k]);
        }

        Serial.println("\n--- COMPARACION DEL MENSAJE ---");
        printBitsLine("Original", data_bits, total_symbols);
        printBitsLine("Recibido", demod_bits, total_symbols);
        
        float ber = (total_symbols > 0) ? (float)errors / total_symbols : 0;
        Serial.printf("\nBER (Bit Error Rate): %d/%d = %.2f%%\n", errors, total_symbols, ber * 100.0f);
      }

      // 3. Volver a esperar la siguiente señal
      current_state = WAITING_FOR_SIGNAL;
      break;
    }
  }
}