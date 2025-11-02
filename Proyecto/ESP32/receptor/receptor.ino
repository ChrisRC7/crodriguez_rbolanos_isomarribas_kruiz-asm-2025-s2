/**
 * @file esp32_receptor_checksum.ino
 * @brief Receptor FSK que utiliza un checksum personalizado de 12 bits para validar el mensaje.
 * @details Solo imprime el mensaje de 4 bits si pasa ambas verificaciones.
 */
#include "arduinoFFT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// --- Parámetros de Hardware y Señal ---
#define ADC_PIN 34
#define SAMPLES 768 // 12 bits * 64 muestras/bit = 768
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Parámetros FSK ---
#define FREQ_0 1000
#define FREQ_1 2000
#define BIT_DURATION 64
#define SYM_FFT_N 64

// --- Configuración del Mensaje ---
const int PACKET_LEN = 12;
const int MESSAGE_LEN = 4;
// Bits de confirmación que esperamos recibir
const uint8_t expected_confirmation_bits[4] = {1, 0, 1, 0};

// --- Buffers y Objeto FFT ---
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// --- TFT ST7735 (mismos pines que TFTModule.ino) ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Máquina de Estados ---
enum State { WAITING_FOR_SIGNAL, SAMPLING, COMPUTING, DISPLAYING };
State current_state = WAITING_FOR_SIGNAL;
const double SIGNAL_THRESHOLD = 200.0;

// --- Función de Ayuda (sin cambios) ---
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

// Variante que expone también las magnitudes en F0 y F1 (reutiliza la misma FFT por símbolo)
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

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  Serial.println("\n\n========================================");
  Serial.println("  Receptor FSK con Verificacion Personalizada");
  Serial.println("========================================");
  Serial.println("Esperando mensajes correctos...");

  // Inicializar TFT
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

// --- Mostrar en TFT cuando el mensaje es válido ---
static void tftShowValidPacket(const uint8_t* packet12, const double* mags0, const double* mags1,
                               const uint8_t* msg4) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.println("FSK Rx OK");

  // Línea de 12 bits
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 14);
  tft.print("Bits(12): ");
  for (int i = 0; i < 12; ++i) tft.print(packet12[i]);

  // Cabecera magnitudes
  tft.setCursor(2, 26);
  tft.println("i b m0  m1");

  tft.setCursor(83, 26);
  tft.println("i b m0  m1");

  // Dos columnas (0..5) y (6..11) para compactar
  int xL = 2, xR = 82; // columnas
  int y = 38;
  for (int i = 0; i < 12; ++i) {
    int x = (i < 6) ? xL : xR;
    if (i == 6) y = 38; // reset Y para la 2da columna
    tft.setCursor(x, y);
    tft.print(i); tft.print(" ");
    tft.print(packet12[i]); tft.print(" ");
    // Magnitudes redondeadas cortas
    int m0 = (int)round(mags0[i]);
    int m1 = (int)round(mags1[i]);
    // Limitar a 3 dígitos para caber
    if (m0 > 999) m0 = 999; if (m1 > 999) m1 = 999;
    tft.print(m0); tft.print(" "); tft.print(m1);
    y += 10;
  }

  // Mensaje de 4 bits al final
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(2, 120);
  tft.print("Msg(4): ");
  for (int i = 0; i < 4; ++i) tft.print(msg4[i]);
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
        vReal[i] = (double)analogRead(ADC_PIN);
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

      

      // 1. Demodular los 12 bits (y acumular/guardar magnitudes en F0/F1 por símbolo)
      double acc_mag0 = 0.0, acc_mag1 = 0.0;
      double mags0[PACKET_LEN];
      double mags1[PACKET_LEN];
      for (int k = 0; k < PACKET_LEN; ++k) {
        int bit_est = 0;
        double m0 = 0.0, m1 = 0.0;
        demodulateSymbolFFT_withMags(&vReal[k * BIT_DURATION], bit_est, m0, m1);
        received_packet[k] = (uint8_t)bit_est;
        if (bit_est == 1) received_all_zeros = false;
        acc_mag0 += m0; acc_mag1 += m1;
        mags0[k] = m0; mags1[k] = m1;
      }

      // 2. Si no es solo ruido, proceder con la verificación
      if (!received_all_zeros) {
        // Separar el paquete en sus 3 partes
        uint8_t received_message[MESSAGE_LEN];
        uint8_t received_confirmation[MESSAGE_LEN];
        uint8_t received_checksum[MESSAGE_LEN];
        for(int i = 0; i < MESSAGE_LEN; i++) {
          received_message[i] = received_packet[i];
          received_confirmation[i] = received_packet[i + MESSAGE_LEN];
          received_checksum[i] = received_packet[i + 2 * MESSAGE_LEN];
        }

        // 3. Realizar las dos comprobaciones
        bool confirmation_ok = true;
        bool checksum_ok = true;

        // Comprobación 1: ¿Coinciden los bits de confirmación?
        for(int i = 0; i < MESSAGE_LEN; i++) {
          if (received_confirmation[i] != expected_confirmation_bits[i]) {
            confirmation_ok = false;
            break;
          }
        }

        // Comprobación 2: ¿Coincide el checksum?
        for(int i = 0; i < MESSAGE_LEN; i++) {
          if ((received_message[i] ^ received_confirmation[i]) != received_checksum[i]) {
            checksum_ok = false;
            break;
          }
        }
        
        // 4. Si y SOLO SI ambas comprobaciones son correctas, imprimir el mensaje y diagnósticos
        if (confirmation_ok && checksum_ok) {
          // Paquete completo (12 bits)
          Serial.print("Paquete recibido (12): ");
          for (int i = 0; i < PACKET_LEN; ++i) Serial.print(received_packet[i]);
          Serial.println();

          // Magnitudes por símbolo (F0/F1) y bit decidido
          Serial.println("Idx  Bit   MagF0   MagF1");
          Serial.println("---  ---  ------  ------");
          for (int i = 0; i < PACKET_LEN; ++i) {
            Serial.print(i); Serial.print("    ");
            Serial.print(received_packet[i]); Serial.print("    ");
            Serial.print(mags0[i], 1); Serial.print("    ");
            Serial.println(mags1[i], 1);
          }
          // También promedio (referencia rápida)
          double avg_m0 = acc_mag0 / PACKET_LEN;
          double avg_m1 = acc_mag1 / PACKET_LEN;
          Serial.print("Promedio Mag -> F0="); Serial.print(FREQ_0); Serial.print("Hz: "); Serial.print(avg_m0, 1);
          Serial.print(" | F1="); Serial.print(FREQ_1); Serial.print("Hz: "); Serial.println(avg_m1, 1);

          Serial.print("Mensaje Recibido: ");
          for(int i=0; i<MESSAGE_LEN; i++) Serial.print(received_message[i]);
          Serial.println();
          // De último: mostrar solo los primeros 4 bits (mensaje)
          Serial.print("Mensaje (4): ");
          for (int i = 0; i < MESSAGE_LEN; ++i) Serial.print(received_message[i]);
          Serial.println();

          // Mostrar en TFT (mismo contenido, compacto)
          tftShowValidPacket(received_packet, mags0, mags1, received_message);
        }
      }
      
      current_state = WAITING_FOR_SIGNAL;
      break;
    }
  }
}