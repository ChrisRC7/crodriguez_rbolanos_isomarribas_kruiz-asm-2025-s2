/**
 * @file receptor_deteccion_energia.ino
 * @brief Receptor FSK con detección de energía y filtrado avanzado ESP-DSP.
 * @details Usa detección de energía en lugar de piloto, con filtrado IIR optimizado.
 */
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "esp_dsp.h"

// --- Parámetros de Hardware y Señal ---
#define ADC_PIN 34
#define ADC_CHANNEL ADC1_CHANNEL_6  // GPIO34 = ADC1_CH6
#define SAMPLES 4608  // ✅ 16 símbolos * 288 = 4608 (para bits de 16ms)
#define SAMPLING_FREQ 18000  // 18 kHz
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)  // ~55.5 us

// --- Parámetros FSK ---
#define FREQ_0 800
#define FREQ_1 2400
#define BIT_DURATION 288  // ✅ 16ms * 18000Hz = 288 muestras por bit (mejor SNR)
#define SYM_FFT_N 512  // ✅ Cero-relleno a 512 para mejor resolución con 288 muestras

// --- Detección de Energía ---
#define ENERGY_WINDOW_SIZE 512  // Ventana para calcular energía RMS
#define ENERGY_THRESHOLD 80.0   // Umbral de RMS para detectar señal
#define SILENCE_SAMPLES 256     // Muestras de silencio antes de captura

// --- Filtrado Avanzado con ESP-DSP ---
// Usamos un filtro Biquad en cascada (2 etapas) para mejor rechazo de ruido
#define NUM_BIQUAD_STAGES 2

// Estructura de estado del filtro biquad
typedef struct {
  float b[3];  // coeficientes feedforward b0, b1, b2
  float a[3];  // coeficientes feedback a0=1, a1, a2
  float w[2];  // estados del filtro (Direct Form II)
} biquad_state_t;

static biquad_state_t lpf_stages[NUM_BIQUAD_STAGES];

// --- Control de Verificación ---
//#define VERIFY_CHECKSUM

// --- Mensaje Original ---
const uint8_t confirmation_pattern[] = {1, 0, 1, 0};
const int MESSAGE_LEN = 4;
const int PACKET_LEN = 12;
const int SYNC_LEN = 2;
const uint8_t sync_pattern[SYNC_LEN] = {1, 0};

// --- Buffers ---
double vReal[SAMPLES];
double vImag[SAMPLES];
static uint8_t demod_bits[128];
static double f0_mags[128];
static double f1_mags[128];

// --- Estados ---
enum State { LISTENING, CAPTURING, PROCESSING };
State current_state = LISTENING;

// Buffer circular para detección de energía
static float energy_buffer[ENERGY_WINDOW_SIZE];
static int energy_idx = 0;

// --- Inicializar filtro Butterworth de 2do orden ---
void init_biquad_lpf(biquad_state_t* bq, float fs, float fc, float Q) {
  float omega = 2.0f * M_PI * fc / fs;
  float sn = sinf(omega);
  float cs = cosf(omega);
  float alpha = sn / (2.0f * Q);
  
  float b0 = (1.0f - cs) * 0.5f;
  float b1 = 1.0f - cs;
  float b2 = (1.0f - cs) * 0.5f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cs;
  float a2 = 1.0f - alpha;
  
  // Normalizar
  bq->b[0] = b0 / a0;
  bq->b[1] = b1 / a0;
  bq->b[2] = b2 / a0;
  bq->a[0] = 1.0f;
  bq->a[1] = a1 / a0;
  bq->a[2] = a2 / a0;
  bq->w[0] = 0.0f;
  bq->w[1] = 0.0f;
}

// Procesar muestra a través de un biquad (Direct Form II)
inline float process_biquad(biquad_state_t* bq, float x) {
  float w0 = x - bq->a[1] * bq->w[0] - bq->a[2] * bq->w[1];
  float y = bq->b[0] * w0 + bq->b[1] * bq->w[0] + bq->b[2] * bq->w[1];
  bq->w[1] = bq->w[0];
  bq->w[0] = w0;
  return y;
}

// Procesar muestra a través de cascada de filtros
inline float process_lpf_cascade(float x) {
  float y = x;
  for (int i = 0; i < NUM_BIQUAD_STAGES; i++) {
    y = process_biquad(&lpf_stages[i], y);
  }
  return y;
}

// --- Calcular RMS de ventana ---
float calculate_rms(const float* buf, int len) {
  float sum_sq = 0.0f;
  for (int i = 0; i < len; i++) {
    sum_sq += buf[i] * buf[i];
  }
  return sqrtf(sum_sq / len);
}

// --- Demodulación FSK con ESP-DSP ---
static void demodulateSymbolFFT(const double* in, int valid_n, int& bit_out, double& mag0, double& mag1) {
  static bool hann_init = false;
  static float win_hann[SYM_FFT_N];
  static float y_cf[2 * SYM_FFT_N];

  if (!hann_init) {
    dsps_wind_hann_f32(win_hann, SYM_FFT_N);
    hann_init = true;
  }

  int ncopy = (valid_n < SYM_FFT_N) ? valid_n : SYM_FFT_N;
  for (int i = 0; i < ncopy; i++) {
    float v = (float)(in[i]);
    y_cf[2 * i + 0] = v * win_hann[i];
    y_cf[2 * i + 1] = 0.0f;
  }
  for (int i = ncopy; i < SYM_FFT_N; i++) {
    y_cf[2 * i + 0] = 0.0f;
    y_cf[2 * i + 1] = 0.0f;
  }
  
  dsps_fft2r_fc32(y_cf, SYM_FFT_N);
  dsps_bit_rev_fc32(y_cf, SYM_FFT_N);

  double fbin0 = (double)FREQ_0 * SYM_FFT_N / SAMPLING_FREQ;
  double fbin1 = (double)FREQ_1 * SYM_FFT_N / SAMPLING_FREQ;
  int bin_f0 = (int)floor(fbin0 + 0.5);
  int bin_f1 = (int)floor(fbin1 + 0.5);

  auto band_mag = [&](int k) {
    double acc = 0.0;
    int k0 = (k - 1 < 0) ? 0 : k - 1;
    int k1 = (k + 1 >= SYM_FFT_N/2) ? SYM_FFT_N/2 - 1 : k + 1;
    for (int i = k0; i <= k1; i++) {
      float re = y_cf[2 * i + 0];
      float im = y_cf[2 * i + 1];
      acc += sqrtf(re * re + im * im);
    }
    return acc;
  };

  mag0 = band_mag(bin_f0);
  mag1 = band_mag(bin_f1);
  bit_out = (mag1 > mag0) ? 1 : 0;
}

// --- Lectura rápida del ADC ---
inline int fastAnalogRead() {
  return adc1_get_raw(ADC_CHANNEL);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  
  // Configurar ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  
  // Inicializar filtros en cascada (2 etapas Butterworth @ 3.5 kHz, Q=0.7071)
  // Frecuencia de corte por debajo de FREQ_1 pero lo suficientemente alta para no atenuar
  for (int i = 0; i < NUM_BIQUAD_STAGES; i++) {
    init_biquad_lpf(&lpf_stages[i], (float)SAMPLING_FREQ, 3500.0f, 0.7071f);
  }
  
  // Inicializar buffer de energía
  for (int i = 0; i < ENERGY_WINDOW_SIZE; i++) {
    energy_buffer[i] = 0.0f;
  }
  
  // Inicializar ESP-DSP FFT
  dsps_fft2r_init_fc32(NULL, SYM_FFT_N);
  
  Serial.println("\n========================================");
  Serial.println("  Receptor FSK con Deteccion de Energia");
  Serial.println("  Filtrado en Cascada + ESP-DSP");
  Serial.println("========================================");
  Serial.println("Esperando señal FSK...");
  Serial.printf("Frecuencia de muestreo: %d Hz\n", SAMPLING_FREQ);
  Serial.printf("Umbral de energia RMS: %.1f\n", ENERGY_THRESHOLD);
  Serial.printf("Filtro: Butterworth 4to orden (cascada), fc=3.5kHz\n");
  Serial.println("========================================\n");
}

void loop() {
  switch (current_state) {
    case LISTENING: {
      // Leer y filtrar muestras continuamente, calcular energía
      unsigned long next_t = micros();
      int raw = fastAnalogRead();
      float centered = (float)(raw - 2048);
      
      // Aplicar filtro en cascada
      float filtered = process_lpf_cascade(centered);
      
      // Actualizar buffer circular de energía
      energy_buffer[energy_idx] = filtered;
      energy_idx = (energy_idx + 1) % ENERGY_WINDOW_SIZE;
      
      // Calcular RMS de la ventana
      float rms = calculate_rms(energy_buffer, ENERGY_WINDOW_SIZE);
      
      // Si detectamos energía suficiente, pasar a captura
      if (rms > ENERGY_THRESHOLD) {
        Serial.printf("[ENERGIA] RMS=%.1f > %.1f, iniciando captura...\n", rms, ENERGY_THRESHOLD);
        current_state = CAPTURING;
      }
      
      // Mantener timing
      while (micros() < next_t + SAMPLING_PERIOD_US) {}
      break;
    }

    case CAPTURING: {
      // Capturar SAMPLES muestras filtradas
      unsigned long next_t = micros();
      for (int i = 0; i < SAMPLES; i++) {
        while (micros() < next_t) {}
        int raw = fastAnalogRead();
        float centered = (float)(raw - 2048);
        vReal[i] = (double)process_lpf_cascade(centered);
        vImag[i] = 0.0;
        next_t += SAMPLING_PERIOD_US;
      }
      current_state = PROCESSING;
      break;
    }

    case PROCESSING: {
      // Remover DC
      double mean = 0;
      for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
      mean /= SAMPLES;
      for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;
      
      // Búsqueda de sync con métrica normalizada
      const int sps = BIT_DURATION;
      const int available_symbols = (int)(SAMPLES / sps);
      const int needed_symbols = SYNC_LEN + PACKET_LEN;
      
      if (available_symbols >= needed_symbols) {
        int best_base_sym = 0;
        int best_offset = 0;
        double best_score = -1e12;
        int stepOffset = max(1, sps / 32); // ✅ Ajustado: 288/32 = 9 muestras
        int max_base_sym = available_symbols - needed_symbols;
        
        Serial.printf("[DEBUG] Buscando sync en %d simbolos, step=%d\n", max_base_sym, stepOffset);
        
        const double eps = 1e-6;
        for (int base_sym = 0; base_sym <= max_base_sym; ++base_sym) {
          int base_idx = base_sym * sps;
          for (int off = 0; off < sps; off += stepOffset) {
            if (base_idx + off + needed_symbols * sps > SAMPLES) break;
            double m0_f0, m0_f1, m1_f0, m1_f1; int tmp;
            demodulateSymbolFFT(&vReal[base_idx + off + 0 * sps], sps, tmp, m0_f0, m0_f1);
            double s0 = (m0_f1 - m0_f0) / (m0_f1 + m0_f0 + eps);
            demodulateSymbolFFT(&vReal[base_idx + off + 1 * sps], sps, tmp, m1_f0, m1_f1);
            double s1 = (m1_f0 - m1_f1) / (m1_f0 + m1_f1 + eps);
            double score = s0 + s1;
            if (score > best_score) { best_score = score; best_base_sym = base_sym; best_offset = off; }
          }
        }
        
        // Búsqueda fina
        {
          int base_idx = best_base_sym * sps;
          int off_start = max(0, best_offset - stepOffset);
          int off_end = min(sps - 1, best_offset + stepOffset);
          for (int off = off_start; off <= off_end; ++off) {
            if (base_idx + off + needed_symbols * sps > SAMPLES) break;
            double m0_f0, m0_f1, m1_f0, m1_f1; int tmp;
            demodulateSymbolFFT(&vReal[base_idx + off + 0 * sps], sps, tmp, m0_f0, m0_f1);
            demodulateSymbolFFT(&vReal[base_idx + off + 1 * sps], sps, tmp, m1_f0, m1_f1);
            double s0 = (m0_f1 - m0_f0) / (m0_f1 + m0_f0 + eps);
            double s1 = (m1_f0 - m1_f1) / (m1_f0 + m1_f1 + eps);
            double score = s0 + s1;
            if (score > best_score) { best_score = score; best_offset = off; }
          }
        }
        
        Serial.printf("[DEBUG] Mejor sync: base_sym=%d, offset=%d, score=%.3f\n", 
                      best_base_sym, best_offset, best_score);
        
        // Verificar sync
        int sync_start = best_base_sym * sps + best_offset;
        double s0_m0, s0_m1, s1_m0, s1_m1; int s0_bit, s1_bit;
        demodulateSymbolFFT(&vReal[sync_start + 0 * sps], sps, s0_bit, s0_m0, s0_m1);
        demodulateSymbolFFT(&vReal[sync_start + 1 * sps], sps, s1_bit, s1_m0, s1_m1);
        Serial.printf("[DEBUG] Sync: [%d,%d] (esperado [1,0])\n", s0_bit, s1_bit);
        
        // Demodular payload
        int payload_start = best_base_sym * sps + best_offset + SYNC_LEN * sps;
        for (int k = 0; k < PACKET_LEN; ++k) {
          int start_idx = payload_start + k * sps;
          int bit_est = 0;
          demodulateSymbolFFT(&vReal[start_idx], sps, bit_est, f0_mags[k], f1_mags[k]);
          demod_bits[k] = (uint8_t)bit_est;
        }
        
        // Verificación
        uint8_t received_data[MESSAGE_LEN];
        uint8_t received_confirmation[MESSAGE_LEN];
        uint8_t received_checksum[MESSAGE_LEN];
        for (int i = 0; i < MESSAGE_LEN; i++) {
          received_data[i] = demod_bits[i];
          received_confirmation[i] = demod_bits[i + MESSAGE_LEN];
          received_checksum[i] = demod_bits[i + 2 * MESSAGE_LEN];
        }
        
        bool confirmation_ok = true;
        for (int i = 0; i < MESSAGE_LEN; i++) {
          if (received_confirmation[i] != confirmation_pattern[i]) {
            confirmation_ok = false;
            break;
          }
        }
        
        uint8_t calculated_checksum[MESSAGE_LEN];
        for (int i = 0; i < MESSAGE_LEN; i++) {
          calculated_checksum[i] = received_data[i] ^ confirmation_pattern[i];
        }
        
        bool checksum_ok = true;
        for (int i = 0; i < MESSAGE_LEN; i++) {
          if (received_checksum[i] != calculated_checksum[i]) {
            checksum_ok = false;
            break;
          }
        }
        
        #ifdef VERIFY_CHECKSUM
          bool should_print = (confirmation_ok && checksum_ok);
        #else
          bool should_print = true;
        #endif
        
        if (should_print) {
          Serial.println("\n========================================");
          Serial.println("   MENSAJE RECIBIDO");
          Serial.println("========================================");
          Serial.print("Recibido: ");
          for (int k = 0; k < PACKET_LEN; k++) Serial.print(demod_bits[k]);
          Serial.println();
          Serial.print("Datos:        ");
          for (int i = 0; i < MESSAGE_LEN; i++) Serial.print(received_data[i]);
          Serial.println();
          Serial.print("Confirmacion: ");
          for (int i = 0; i < MESSAGE_LEN; i++) Serial.print(received_confirmation[i]);
          Serial.printf(" [Esperado: 1010] %s\n", confirmation_ok ? "✓" : "✗");
          Serial.print("Checksum:     ");
          for (int i = 0; i < MESSAGE_LEN; i++) Serial.print(received_checksum[i]);
          Serial.print(" [Calc: ");
          for (int i = 0; i < MESSAGE_LEN; i++) Serial.print(calculated_checksum[i]);
          Serial.printf("] %s\n", checksum_ok ? "✓" : "✗");
          Serial.println("========================================\n");
        }
      }
      
      // Volver a escuchar
      current_state = LISTENING;
      break;
    }
  }
}
