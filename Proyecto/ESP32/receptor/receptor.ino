/**
 * @file esp32_receptor_checksum.ino
 * @brief Receptor FSK que utiliza un checksum personalizado de 12 bits para validar el mensaje.
 * @details Solo imprime el mensaje de 4 bits si pasa ambas verificaciones.
 */
#include "arduinoFFT.h"

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

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  Serial.println("\n\n========================================");
  Serial.println("  Receptor FSK con Verificacion Personalizada");
  Serial.println("========================================");
  Serial.println("Esperando mensajes correctos...");
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

      // 1. Demodular los 12 bits
      for (int k = 0; k < PACKET_LEN; ++k) {
        int bit_est = 0;
        demodulateSymbolFFT(&vReal[k * BIT_DURATION], bit_est);
        received_packet[k] = (uint8_t)bit_est;
        if (bit_est == 1) received_all_zeros = false;
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
        
        // 4. Si y SOLO SI ambas comprobaciones son correctas, imprimir el mensaje
        if (confirmation_ok && checksum_ok) {
          Serial.print("Mensaje Recibido: ");
          for(int i=0; i<MESSAGE_LEN; i++) Serial.print(received_message[i]);
          Serial.println();
        }
      }
      
      current_state = WAITING_FOR_SIGNAL;
      break;
    }
  }
}