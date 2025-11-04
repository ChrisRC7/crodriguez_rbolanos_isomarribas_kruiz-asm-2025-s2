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
#define BIT_DURATION 128
#define SYM_FFT_N 64

// --- Configuración del Mensaje ---
const int PACKET_LEN = 12;
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

// --- Filtro Pasa Bajas IIR ---
// Filtro de primer orden: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
// Frecuencia de corte aprox. 3000 Hz (ajustable)
const double FILTER_ALPHA = 0.6;  // Mayor valor = más respuesta, menor suavizado
double filtered_sample = 0.0;     // Estado del filtro

// --- TFT ST7735 ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Máquina de Estados ---
enum State { WAITING_FOR_SIGNAL, SAMPLING, COMPUTING, DISPLAYING };
State current_state = WAITING_FOR_SIGNAL;
const double SIGNAL_THRESHOLD = 200.0;

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
        // Aplicar filtro IIR pasa bajas
        filtered_sample = FILTER_ALPHA * raw_sample + (1.0 - FILTER_ALPHA) * filtered_sample;
        vReal[i] = filtered_sample;
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
      for (int k = 0; k < PACKET_LEN; ++k) {
        int bit_est = 0;
        double m0 = 0.0, m1 = 0.0;
        // Usar solo la mitad central de cada bit (offset de 32 muestras, 64 muestras de longitud)
        int offset = k * BIT_DURATION + (BIT_DURATION - SYM_FFT_N) / 2;
        demodulateSymbolFFT_withMags(&vReal[offset], bit_est, m0, m1);
        received_packet[k] = (uint8_t)bit_est;
        if (bit_est == 1) received_all_zeros = false;
        acc_mag0 += m0; acc_mag1 += m1;
        mags0[k] = m0; mags1[k] = m1;
      }

      if (!received_all_zeros) {
        uint8_t received_message[MESSAGE_LEN];
        uint8_t received_confirmation[MESSAGE_LEN];
        uint8_t received_checksum[MESSAGE_LEN];
        for(int i = 0; i < MESSAGE_LEN; i++) {
          received_message[i] = received_packet[i];
          received_confirmation[i] = received_packet[i + MESSAGE_LEN];
          received_checksum[i] = received_packet[i + 2 * MESSAGE_LEN];
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
          Serial.print("Paquete recibido (12): ");
          for (int i = 0; i < PACKET_LEN; ++i) Serial.print(received_packet[i]);
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
        }
      }
      
      current_state = WAITING_FOR_SIGNAL;
      break;
    }
  }
}