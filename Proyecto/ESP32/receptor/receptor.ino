/**
 * @file esp32_receptor_checksum_buzzer.ino
 * @brief Receptor FSK con buzzer que reproduce el mensaje recibido
 * @details Reproduce sonido según bits: 1=suena, 0=silencio, con pausa de 0.5s entre bits
 */
#include "arduinoFFT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// --- Parámetros de Hardware y Señal ---
#define ADC_PIN 34
#define BUZZER_PIN 25  // Pin para controlar el buzzer
#define SAMPLES 1536
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Parámetros FSK ---
#define FREQ_0 1000
#define FREQ_1 2000
#define BIT_DURATION 88
#define SYM_FFT_N 64

// --- Configuración del Mensaje ---
const int PACKET_LEN = 13;  // 1 bit sincronización + 12 bits datos
const int MESSAGE_LEN = 4;
const uint8_t expected_confirmation_bits[4] = {1, 0, 1, 0};

// --- Buzzer Config ---
const int BUZZER_FREQ = 2000;  // Frecuencia del tono en Hz
const int BIT_SOUND_DURATION = 300;  // Duración del sonido por bit (ms)
const int BIT_PAUSE_DURATION = 500;  // Pausa entre bits (ms)

// --- Buffers y Objeto FFT ---
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// --- Filtro Pasa Bajas IIR de Segundo Orden ---
// Butterworth filter: y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
// Diseñado para ~3000 Hz cutoff a 8kHz sampling
const double FILTER_A0 = 0.1206;
const double FILTER_A1 = 0.2412;
const double FILTER_A2 = 0.1206;
const double FILTER_B1 = 1.3695;
const double FILTER_B2 = 0.5132;

double filtered_sample = 0.0;
double filtered_sample_z1 = 0.0;  // x[n-1]
double filtered_sample_z2 = 0.0;  // x[n-2]
double filtered_output_z1 = 0.0;  // y[n-1]
double filtered_output_z2 = 0.0;  // y[n-2]

// --- TFT ST7735 ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Máquina de Estados ---
enum State { WAITING_FOR_SIGNAL, SAMPLING, COMPUTING, DISPLAYING, CLEANUP, COOLDOWN };
State current_state = WAITING_FOR_SIGNAL;
const double SIGNAL_THRESHOLD = 200.0;
unsigned long cleanup_start_time = 0;
const unsigned long CLEANUP_DURATION_MS = 50;
// Cooldown para evitar recibir el mismo mensaje múltiples veces
unsigned long cooldown_start_time = 0;
const unsigned long COOLDOWN_DURATION_MS = 2000;  // 2 segundos de espera antes de recibir otro mensaje

// --- Funciones de Ayuda ---
static void demodulateSymbolFFT(const double* in, int& bit_out) {
  static double r[SYM_FFT_N], im[SYM_FFT_N];
  double mag0, mag1;
  for (int i = 0; i < SYM_FFT_N; i++) { r[i] = in[i]; im[i] = 0.0; }
  ArduinoFFT<double> f(r, im, SYM_FFT_N, SAMPLING_FREQ);
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  f.compute(FFTDirection::Forward);
  f.complexToMagnitude();
  int b0 = (int)round((double)FREQ_0 * SYM_FFT_N / SAMPLING_FREQ);
  int b1 = (int)round((double)FREQ_1 * SYM_FFT_N / SAMPLING_FREQ);
  mag0 = r[b0];
  mag1 = r[b1];
  bit_out = (mag1 > mag0) ? 1 : 0;
}

static void demodulateSymbolFFT_withMags(const double* in, int& bit_out, double& out_mag0, double& out_mag1) {
  static double r[SYM_FFT_N], im[SYM_FFT_N];
  for (int i = 0; i < SYM_FFT_N; i++) { r[i] = in[i]; im[i] = 0.0; }
  ArduinoFFT<double> f(r, im, SYM_FFT_N, SAMPLING_FREQ);
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  f.compute(FFTDirection::Forward);
  f.complexToMagnitude();
  int b0 = (int)round((double)FREQ_0 * SYM_FFT_N / SAMPLING_FREQ);
  int b1 = (int)round((double)FREQ_1 * SYM_FFT_N / SAMPLING_FREQ);
  out_mag0 = r[b0];
  out_mag1 = r[b1];
  bit_out = (out_mag1 > out_mag0) ? 1 : 0;
}

// --- Función para reproducir el mensaje en el buzzer ---
void playMessageOnBuzzer(const uint8_t* message, int length) {
  Serial.println("\n--- Reproduciendo mensaje en buzzer ---");
  
  for (int i = 0; i < length; i++) {
    Serial.print("Bit ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(message[i]);
    
    if (message[i] == 1) {
      // Bit 1: Sonar el buzzer
      tone(BUZZER_PIN, BUZZER_FREQ, BIT_SOUND_DURATION);
      delay(BIT_SOUND_DURATION);
    } else {
      // Bit 0: Silencio
      noTone(BUZZER_PIN);
      delay(BIT_SOUND_DURATION);
    }
    
    // Pausa entre bits
    noTone(BUZZER_PIN);
    delay(BIT_PAUSE_DURATION);
  }
  
  Serial.println("--- Reproducción completada ---\n");
}

static void tftShowValidPacket(const uint8_t* packet12, const double* mags0, const double* mags1,
                               const uint8_t* msg4) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.println("FSK Rx OK");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 14);
  tft.print("Bits(12): ");
  for (int i = 0; i < 12; ++i) tft.print(packet12[i]);

  tft.setCursor(2, 26);
  tft.println("i b m0  m1");
  tft.setCursor(83, 26);
  tft.println("i b m0  m1");

  int xL = 2, xR = 82;
  int y = 38;
  for (int i = 0; i < 12; ++i) {
    int x = (i < 6) ? xL : xR;
    if (i == 6) y = 38;
    tft.setCursor(x, y);
    tft.print(i); tft.print(" ");
    tft.print(packet12[i]); tft.print(" ");
    int m0 = (int)round(mags0[i]);
    int m1 = (int)round(mags1[i]);
    if (m0 > 999) m0 = 999; if (m1 > 999) m1 = 999;
    tft.print(m0); tft.print(" "); tft.print(m1);
    y += 10;
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(2, 120);
  tft.print("Msg(4): ");
  for (int i = 0; i < 4; ++i) tft.print(msg4[i]);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);  // Configurar pin del buzzer
  
  Serial.println("\n\n========================================");
  Serial.println("  Receptor FSK con Verificacion y Buzzer");
  Serial.println("========================================");
  Serial.println("Esperando mensajes correctos...");

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(true);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.println("ESP32 Receptor FSK");
  tft.setCursor(4, 16);
  tft.println("Esperando mensaje valido...");
}

void loop() {
  switch (current_state) {
    case WAITING_FOR_SIGNAL:
      if (abs((double)analogRead(ADC_PIN) - 2048.0) > SIGNAL_THRESHOLD) {
        current_state = SAMPLING;
      }
      break;

    case SAMPLING: {
      unsigned long next_t = micros();
      for (int i = 0; i < SAMPLES; i++) {
        while (micros() < next_t) { /* esperar */ }
        double raw_sample = (double)analogRead(ADC_PIN);
        
        // Aplicar filtro IIR de segundo orden Butterworth
        double filtered_out = FILTER_A0 * raw_sample + 
                             FILTER_A1 * filtered_sample_z1 + 
                             FILTER_A2 * filtered_sample_z2 - 
                             FILTER_B1 * filtered_output_z1 - 
                             FILTER_B2 * filtered_output_z2;
        
        // Actualizar estados
        filtered_sample_z2 = filtered_sample_z1;
        filtered_sample_z1 = raw_sample;
        filtered_output_z2 = filtered_output_z1;
        filtered_output_z1 = filtered_out;
        
        vReal[i] = filtered_out;
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
      uint8_t received_packet[PACKET_LEN];
      bool received_all_zeros = true;

      double acc_mag0 = 0.0, acc_mag1 = 0.0;
      double mags0[PACKET_LEN];
      double mags1[PACKET_LEN];
      
      // --- Búsqueda de sincronización: encontrar el mejor offset ---
      // Intentar diferentes offsets y elegir el que mejor match con confirmación esperada
      // Bits de confirmación ahora están en posiciones 5-8 (bit 0 es sync, bits 1-4 son mensaje)
      int best_offset = 0;
      int best_matches = 0;
      
      // Expandir rango de búsqueda
      for (int test_offset = -10; test_offset <= 10; test_offset++) {
        int matches = 0;
        // Verificar bits de confirmación (5-8) contra patrón esperado 1010
        for (int k = 5; k < 9; ++k) {
          int offset = k * BIT_DURATION + (BIT_DURATION - SYM_FFT_N) / 2 + 10 + test_offset;
          if (offset >= 0 && offset + SYM_FFT_N <= SAMPLES) {
            int bit_est = 0;
            double m0 = 0.0, m1 = 0.0;
            demodulateSymbolFFT_withMags(&vReal[offset], bit_est, m0, m1);
            if (bit_est == expected_confirmation_bits[k - 5]) {
              matches++;
            }
          }
        }
        // Solo considerar offsets que dan al menos 3 matches
        if (matches >= 3 && matches > best_matches) {
          best_matches = matches;
          best_offset = test_offset;
        }
      }
      
      // Si no hay buen match, usar offset 0 por defecto
      if (best_matches < 3) {
        best_offset = 0;
      }
      
      // Debug: mostrar offset elegido
      Serial.print("DEBUG: best_offset="); Serial.print(best_offset); 
      Serial.print(" matches="); Serial.println(best_matches);
      
      // Usar el mejor offset encontrado
      for (int k = 0; k < PACKET_LEN; ++k) {
        int bit_est = 0;
        double m0 = 0.0, m1 = 0.0;
        // Bit 0 es sincronización (se ignora en la validación), no aplicar extra_offset
        int offset = k * BIT_DURATION + (BIT_DURATION - SYM_FFT_N) / 2 + 10 + best_offset;
        if (offset >= 0 && offset + SYM_FFT_N <= SAMPLES) {
          demodulateSymbolFFT_withMags(&vReal[offset], bit_est, m0, m1);
          received_packet[k] = (uint8_t)bit_est;
          if (bit_est == 1) received_all_zeros = false;
          acc_mag0 += m0; acc_mag1 += m1;
          mags0[k] = m0; mags1[k] = m1;
        }
      }

      // --- Validación mejorada: threshold adaptativo y mayoría de votos ---
      for (int k = 0; k < PACKET_LEN; ++k) {
        double margin = fabs(mags0[k] - mags1[k]);
        int current_bit = received_packet[k];
        
        // Si el margen es muy bajo (< 3000), el bit es poco confiable
        if (margin < 3000.0 && k > 0 && k < PACKET_LEN - 1) {
          int left_bit = received_packet[k - 1];
          int right_bit = received_packet[k + 1];
          
          // Si ambos vecinos son iguales, usar consenso
          if (left_bit == right_bit) {
            received_packet[k] = left_bit;
          }
          // Si vecinos son diferentes, mantener el bit actual pero marcar como bajo margen
        }
      }

      if (!received_all_zeros) {
        // Ignorar bit 0 (sincronización) y procesar bits 1-12 como datos
        uint8_t received_message[MESSAGE_LEN];
        uint8_t received_confirmation[MESSAGE_LEN];
        uint8_t received_checksum[MESSAGE_LEN];
        
        // Bits 1-4: Mensaje
        for(int i = 0; i < MESSAGE_LEN; i++) {
          received_message[i] = received_packet[1 + i];
        }
        // Bits 5-8: Confirmación
        for(int i = 0; i < MESSAGE_LEN; i++) {
          received_confirmation[i] = received_packet[5 + i];
        }
        // Bits 9-12: Checksum
        for(int i = 0; i < MESSAGE_LEN; i++) {
          received_checksum[i] = received_packet[9 + i];
        }

        bool confirmation_ok = true;
        bool checksum_ok = true;

        for(int i = 0; i < MESSAGE_LEN; i++) {
          if (received_confirmation[i] != expected_confirmation_bits[i]) {
            confirmation_ok = false;
            break;
          }
        }

        for(int i = 0; i < MESSAGE_LEN; i++) {
          if ((received_message[i] ^ received_confirmation[i]) != received_checksum[i]) {
            checksum_ok = false;
            break;
          }
        }
        
        if (confirmation_ok && checksum_ok) {
          Serial.print("Paquete recibido (13 bits, primero es sync): ");
          for (int i = 0; i < PACKET_LEN; ++i) Serial.print(received_packet[i]);
          Serial.println();
          Serial.print("Mensaje extraído (bits 1-4): ");
          for (int i = 0; i < MESSAGE_LEN; ++i) Serial.print(received_message[i]);
          Serial.println();

          Serial.println("Idx  Bit   MagF0   MagF1");
          Serial.println("---  ---  ------  ------");
          for (int i = 0; i < PACKET_LEN; ++i) {
            Serial.print(i); Serial.print("    ");
            Serial.print(received_packet[i]); Serial.print("    ");
            Serial.print(mags0[i], 1); Serial.print("    ");
            Serial.println(mags1[i], 1);
          }
          
          double avg_m0 = acc_mag0 / PACKET_LEN;
          double avg_m1 = acc_mag1 / PACKET_LEN;
          Serial.print("Promedio Mag -> F0="); Serial.print(FREQ_0); Serial.print("Hz: "); Serial.print(avg_m0, 1);
          Serial.print(" | F1="); Serial.print(FREQ_1); Serial.print("Hz: "); Serial.println(avg_m1, 1);

          Serial.print("Mensaje Recibido: ");
          for(int i=0; i<MESSAGE_LEN; i++) Serial.print(received_message[i]);
          Serial.println();
          
          Serial.print("Mensaje (4): ");
          for (int i = 0; i < MESSAGE_LEN; ++i) Serial.print(received_message[i]);
          Serial.println();

          tftShowValidPacket(received_packet, mags0, mags1, received_message);
          
          // *** REPRODUCIR MENSAJE EN EL BUZZER ***
          playMessageOnBuzzer(received_message, MESSAGE_LEN);
          
          // Transición a cooldown para evitar duplicados
          cooldown_start_time = millis();
          current_state = COOLDOWN;
          break;
        }
      }
      
      // --- Limpieza y reseteo de estado para evitar interferencia ---
      // Resetear buffers de FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = 0.0;
        vImag[i] = 0.0;
      }
      // Resetear estado del filtro IIR
      filtered_sample = 0.0;
      filtered_sample_z1 = 0.0;
      filtered_sample_z2 = 0.0;
      filtered_output_z1 = 0.0;
      filtered_output_z2 = 0.0;
      
      // Transición a estado de limpieza sin bloqueante ---
      cleanup_start_time = millis();
      current_state = CLEANUP;
      break;
    }

    case COOLDOWN: {
      // Esperar el tiempo de cooldown sin procesar señales
      unsigned long cooldown_elapsed = millis() - cooldown_start_time;
      
      if (cooldown_elapsed >= COOLDOWN_DURATION_MS) {
        // Cooldown terminado, volver a esperar señales
        current_state = WAITING_FOR_SIGNAL;
      }
      break;
    }

    case CLEANUP: {
      // Limpieza gradual sin bloquear
      unsigned long cleanup_elapsed = millis() - cleanup_start_time;
      
      // Resetear buffers de FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = 0.0;
        vImag[i] = 0.0;
      }
      // Resetear estado del filtro IIR
      filtered_sample = 0.0;
      filtered_sample_z1 = 0.0;
      filtered_sample_z2 = 0.0;
      filtered_output_z1 = 0.0;
      filtered_output_z2 = 0.0;
      
      // Después del tiempo de limpieza, volver a esperar señal
      if (cleanup_elapsed >= CLEANUP_DURATION_MS) {
        current_state = WAITING_FOR_SIGNAL;
      }
      break;
    }
  }
}