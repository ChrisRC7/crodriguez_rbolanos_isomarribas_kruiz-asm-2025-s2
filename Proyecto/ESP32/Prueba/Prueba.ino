/*
 * Modulador y Demodulador FSK con FFT - ESP32
 * - Genera señal FSK interna (seno) y analiza con FFT
 * - Demodulación símbolo a símbolo con FFT (sin jack, usando el buffer interno)
 * - Muestra resultados por Monitor Serial
 */

#include "arduinoFFT.h"

// Parámetros de adquisición/FFT (frame)
#define SAMPLES 512              // antes 256; 8 bits * 64 sps = 512
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// Parámetros FSK
#define FREQ_0 1000
#define FREQ_1 2000
#define BIT_DURATION 64          // antes 50; coherente con Fs=8k y F0/F1

// Demod interno
#define MODE_DEMODULATOR 1
#define SYM_FFT_N 64             // mantener 64 para que el bin de F0/F1 sea exacto

// Opcional: desactivar ventana en demod por símbolo cuando es coherente
#define SYM_USE_WINDOW 0
#define SYM_LOCK_TO_BINS 1   // nuevo: usa exactamente los bins de F0/F1 al demodular

// Datos a modular (ejemplo)
const uint8_t data_bits[] = {1, 0, 1, 1, 0, 1, 0, 0};
const int num_bits = sizeof(data_bits) / sizeof(data_bits[0]);

// Buffers FFT (frame)
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// Copia del frame en dominio del tiempo (para demod por símbolo)
double tReal[SAMPLES];

// Control de ejecución
unsigned long last_analysis_time = 0;
unsigned long analysis_interval = 2000000; // 2 s entre análisis

// Máquina de estados
enum State { GENERATING, COMPUTING, DISPLAYING, WAITING };
State current_state = GENERATING;

// Buffer para almacenar bits demodulados del último análisis
static uint8_t demod_bits[128];

// Espera breve del puerto serie usando micros (sin delay)
static inline void waitForSerialMicros(uint32_t us) {
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < us) {
    if (Serial && Serial.availableForWrite() > 0) break;
    yield();
  }
}

// Helpers de impresión de líneas de bits y de Hz
static void printBitsLine(const char* label, const uint8_t* bits, int n) {
  Serial.print(label); Serial.print(": ");
  for (int i = 0; i < n; ++i) Serial.print(bits[i]);
  Serial.println();
}

static void printHzLine(const char* label, const uint8_t* bits, int n) {
  Serial.print(label); Serial.print(": ");
  for (int i = 0; i < n; ++i) {
    int hz = bits[i] ? FREQ_1 : FREQ_0;
    Serial.print(hz);
    if (i < n - 1) Serial.print(' ');
  }
  Serial.println();
}
// -------------------- Utilidades --------------------
static inline void printBarGraph(const char* label, float freq, float magnitude, float max_mag) {
  Serial.printf("%s (%.0fHz): ", label, freq);
  int bar_length = (max_mag > 0) ? (int)((magnitude / max_mag) * 40) : 0;
  for (int i = 0; i < bar_length; i++) Serial.print("█");
  Serial.printf(" %.1f\n", magnitude);
}

static inline void findPeakNear(int targetBin, int window, int& outIdx, double& outMag) {
  int start = max(2, targetBin - window);
  int end   = min((SAMPLES/2) - 1, targetBin + window);
  double bestMag = 0.0;
  int bestIdx = targetBin;
  for (int i = start; i <= end; i++) {
    if (vReal[i] > bestMag) { bestMag = vReal[i]; bestIdx = i; }
  }
  outIdx = bestIdx;
  outMag = bestMag;
}

static void printTopN_frame(int N) {
  const int half = SAMPLES / 2;
  static double mags[(SAMPLES/2)];
  for (int i = 0; i < half; ++i) mags[i] = vReal[i];

  for (int rank = 0; rank < N; ++rank) {
    double max_mag = 0.0; int max_idx = 0;
    for (int i = 2; i < half; i++) {
      if (mags[i] > max_mag) { max_mag = mags[i]; max_idx = i; }
    }
    if (max_mag <= 0) break;
    double freq = (max_idx * 1.0 * SAMPLING_FREQ) / SAMPLES;
    Serial.printf("  %d. %.2f Hz → Magnitud: %.2f\n", rank + 1, freq, max_mag);
    for (int j = max(2, max_idx - 1); j <= min(half - 1, max_idx + 1); ++j) mags[j] = 0.0;
  }
}

// FFT por símbolo (demod interno)
static inline int binFromFreq(double f, int nfft, int fs) {
  return (int)round((f * nfft) / fs);
}

static void demodulateSymbolFFT(const double* in, int sps, int fs,
                                int& bit_out, double& f0_det, double& f1_det,
                                double& mag0, double& mag1) {
  static double r[SYM_FFT_N];
  static double im[SYM_FFT_N];

  int ncopy = min(sps, SYM_FFT_N);
  for (int i = 0; i < ncopy; ++i) { r[i] = in[i]; im[i] = 0.0; }
  for (int i = ncopy; i < SYM_FFT_N; ++i) { r[i] = 0.0; im[i] = 0.0; }

  // Remover DC del símbolo
  double mean = 0.0; for (int i = 0; i < ncopy; ++i) mean += r[i];
  if (ncopy > 0) mean /= ncopy;
  for (int i = 0; i < ncopy; ++i) r[i] -= mean;

  ArduinoFFT<double> f(r, im, SYM_FFT_N, fs);
#if SYM_USE_WINDOW
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward);
#endif
  f.compute(FFTDirection::Forward);
  f.complexToMagnitude();

  int b0 = binFromFreq(FREQ_0, SYM_FFT_N, fs);
  int b1 = binFromFreq(FREQ_1, SYM_FFT_N, fs);
  int half = SYM_FFT_N / 2;

#if SYM_LOCK_TO_BINS
  // Tomar exactamente los bins de F0 y F1 (coherente con sps=64, Fs=8k)
  if (b0 < 2 || b0 >= half) b0 = max(2, min(half - 1, b0));
  if (b1 < 2 || b1 >= half) b1 = max(2, min(half - 1, b1));
  mag0 = r[b0];
  mag1 = r[b1];
  f0_det = FREQ_0;
  f1_det = FREQ_1;
#else
  // Búsqueda local (útil si no hay coherencia exacta)
  int w = 1;
  mag0 = 0.0; mag1 = 0.0; int i0 = b0, i1 = b1;
  for (int i = max(2, b0 - w); i <= min(half - 1, b0 + w); ++i) if (r[i] > mag0) { mag0 = r[i]; i0 = i; }
  for (int i = max(2, b1 - w); i <= min(half - 1, b1 + w); ++i) if (r[i] > mag1) { mag1 = r[i]; i1 = i; }
  f0_det = (i0 * 1.0 * fs) / SYM_FFT_N;
  f1_det = (i1 * 1.0 * fs) / SYM_FFT_N;
#endif

  bit_out = (mag1 > mag0) ? 1 : 0;
}

// -------------------- Setup/Loop --------------------
void setup() {
  Serial.begin(115200);
  // delay(1500);  // eliminado
  waitForSerialMicros(1500000UL);  // espera basada en micros

  Serial.println("\n\n========================================");
  Serial.println("    MOD/Demod FSK con FFT - ESP32");
  Serial.println("========================================");
  Serial.printf("Fs: %d Hz | N(frame): %d | Res(frame): %.2f Hz/bin\n", SAMPLING_FREQ, SAMPLES, (float)SAMPLING_FREQ/SAMPLES);
  Serial.printf("F0: %d Hz | F1: %d Hz | sps(bit): %d\n", FREQ_0, FREQ_1, BIT_DURATION);
  Serial.printf("Demod: %s | N(simbolo): %d | Res(simbolo): %.2f Hz/bin\n",
                MODE_DEMODULATOR ? "ON" : "OFF", SYM_FFT_N, (float)SAMPLING_FREQ/SYM_FFT_N);
  Serial.println("========================================\n");

  last_analysis_time = micros();
}

void loop() {
  unsigned long now = micros();

  switch (current_state) {
    case GENERATING:
      // Genera el frame FSK en vReal/vImag
      Serial.println("Generando señal FSK...");
      Serial.print("Patrón: ");
      for (int i = 0; i < num_bits; i++) Serial.print(data_bits[i]);
      Serial.println();
      // Mostrar cómo quedará el mapeo a portadoras (modulado)
      printHzLine("Modulado (Hz)", data_bits, num_bits);

      // Limpiar buffers
      for (int i = 0; i < SAMPLES; i++) {
        tReal[i] = 0;               // limpiar buffer de tiempo
        vReal[i] = 0; vImag[i] = 0; // opcional: limpiar también buffers FFT
      }

      // Muestreo temporal preciso
      {
        unsigned long t0 = micros();
        unsigned long next_t = t0;
        int idx = 0;
        for (int b = 0; b < num_bits && idx < SAMPLES; b++) {
          float f = (data_bits[b] ? FREQ_1 : FREQ_0);
          for (int s = 0; s < BIT_DURATION && idx < SAMPLES; s++) {
            while ((long)(micros() - next_t) < 0) { /* busy-wait */ }
            float t = (micros() - t0) / 1000000.0f;
            tReal[idx] = sinf(2.0f * PI * f * t);   // guardar señal en TIEMPO
            idx++;
            next_t += SAMPLING_PERIOD_US;
          }
        }
        unsigned long dt = micros() - t0;
        Serial.printf("Muestras: %d | Tiempo: %lu us | Esperado: %lu us\n\n",
                      idx, dt, (unsigned long)(SAMPLES * SAMPLING_PERIOD_US));
      }
      current_state = COMPUTING;
      break;

    case COMPUTING:
      Serial.println("Calculando FFT (frame)...");
      {
        // Copiar el frame de TIEMPO a los buffers de FFT
        for (int i = 0; i < SAMPLES; ++i) { vReal[i] = tReal[i]; vImag[i] = 0.0; }

        unsigned long t0 = micros();
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude(); // vReal contiene magnitudes
        unsigned long dt = micros() - t0;
        Serial.printf("FFT calculada en %lu us\n\n", dt);
      }
      current_state = DISPLAYING;
      break;

    case DISPLAYING:
      if (MODE_DEMODULATOR) {
        // Vista de frame + demod por símbolo
        Serial.println("========================================");
        Serial.println("           ANÁLISIS DE FRAME");
        Serial.println("========================================");
        // Picos cerca de F0/F1
        int b0est = (int)round((double)FREQ_0 * SAMPLES / SAMPLING_FREQ);
        int b1est = (int)round((double)FREQ_1 * SAMPLES / SAMPLING_FREQ);
        int i0=0, i1=0; double m0=0, m1=0;
        findPeakNear(b0est, 3, i0, m0);
        findPeakNear(b1est, 3, i1, m1);
        double f0 = (i0 * 1.0 * SAMPLING_FREQ) / SAMPLES;
        double f1 = (i1 * 1.0 * SAMPLING_FREQ) / SAMPLES;

        Serial.println("FRECUENCIAS (cercanas a portadoras):");
        Serial.println("------------------------------------");
        Serial.printf("  F0 ~ %d Hz → %.2f Hz (Mag: %.2f)\n", FREQ_0, f0, m0);
        Serial.printf("  F1 ~ %d Hz → %.2f Hz (Mag: %.2f)\n\n", FREQ_1, f1, m1);

        Serial.println("TOP 8 COMPONENTES (frame):");
        Serial.println("--------------------------");
        printTopN_frame(8);
        Serial.println();

        double max_mag = (m0 > m1) ? m0 : m1;
        Serial.println("ESPECTRO (Portadoras):");
        Serial.println("----------------------");
        printBarGraph("F0", f0, m0, max_mag);
        printBarGraph("F1", f1, m1, max_mag);
        Serial.println("----------------------\n");

        // Demod símbolo a símbolo usando el buffer en TIEMPO
        const int sps = BIT_DURATION;
        const int total_symbols = min(num_bits, (int)(SAMPLES / sps));
        Serial.println("========================================");
        Serial.println("        DEMODULACIÓN POR SÍMBOLO");
        Serial.println("========================================");
        Serial.printf("Símbolos: %d | NFFT símbolo: %d | Res: %.2f Hz/bin\n\n",
                      total_symbols, SYM_FFT_N, (float)SAMPLING_FREQ/SYM_FFT_N);

        Serial.println("Idx  Bit  F0_det(Hz)  F1_det(Hz)   Mag0    Mag1");
        Serial.println("---- ---  ----------  ----------  ------  ------");

        int errors = 0;
        for (int k = 0; k < total_symbols; ++k) {
          int start = k * sps;
          int bit_est = 0; double fd0=0, fd1=0, mm0=0, mm1=0;
          demodulateSymbolFFT(&tReal[start], sps, SAMPLING_FREQ, bit_est, fd0, fd1, mm0, mm1); // <-- tReal
          demod_bits[k] = (uint8_t)bit_est;
          Serial.printf("%3d  %d    %8.2f    %8.2f   %6.2f  %6.2f\n", k, bit_est, fd0, fd1, mm0, mm1);
          if (k < num_bits && bit_est != data_bits[k]) errors++;
        }

        if (total_symbols > 0 && total_symbols <= num_bits) {
          float ber = (float)errors / total_symbols;
          Serial.printf("\nBER (vs patrón): %d/%d = %.2f%%\n", errors, total_symbols, ber * 100.0f);
        }

        // Comparación compacta de mensajes (solo el subconjunto presente en el frame)
        Serial.println("\nCOMPARACIÓN DEL MENSAJE (subset en frame):");
        printBitsLine("Original (bits)", data_bits, total_symbols);
        printHzLine("Modulado (Hz)   ", data_bits, total_symbols);
        printBitsLine("Demod (bits)    ", demod_bits, total_symbols);

        Serial.printf("\nResolución frame: %.2f Hz/bin | Resolución símbolo: %.2f Hz/bin\n\n",
                      (float)SAMPLING_FREQ/SAMPLES, (float)SAMPLING_FREQ/SYM_FFT_N);
      } else {
        // Solo análisis de frame (sin demod por símbolo)
        Serial.println("========================================");
        Serial.println("         ANÁLISIS DE FRECUENCIAS");
        Serial.println("========================================");

        int b0est = (int)round((double)FREQ_0 * SAMPLES / SAMPLING_FREQ);
        int b1est = (int)round((double)FREQ_1 * SAMPLES / SAMPLING_FREQ);

        int i0=0, i1=0; double m0=0, m1=0;
        findPeakNear(b0est, 3, i0, m0);
        findPeakNear(b1est, 3, i1, m1);

        double f0 = (i0 * 1.0 * SAMPLING_FREQ) / SAMPLES;
        double f1 = (i1 * 1.0 * SAMPLING_FREQ) / SAMPLES;

        Serial.println("FRECUENCIAS DETECTADAS (FSK):");
        Serial.println("-----------------------------");
        Serial.printf("  F0 ~ %d Hz → %.2f Hz (Mag: %.2f)\n", FREQ_0, f0, m0);
        Serial.printf("  F1 ~ %d Hz → %.2f Hz (Mag: %.2f)\n\n", FREQ_1, f1, m1);

        float res = (float)SAMPLING_FREQ / SAMPLES;
        Serial.printf("Resolución FFT: %.2f Hz/bin\n\n", res);

        Serial.println("TOP 8 COMPONENTES:");
        Serial.println("------------------");
        printTopN_frame(8);

        Serial.println("\nESPECTRO (Portadoras):");
        Serial.println("----------------------");
        double max_mag = (m0 > m1) ? m0 : m1;
        printBarGraph("F0", f0, m0, max_mag);
        printBarGraph("F1", f1, m1, max_mag);
        Serial.println("----------------------\n");
      }

      current_state = WAITING;
      last_analysis_time = now;
      break;

    case WAITING:
      if ((now - last_analysis_time) >= analysis_interval) current_state = GENERATING;
      break;
  }
}