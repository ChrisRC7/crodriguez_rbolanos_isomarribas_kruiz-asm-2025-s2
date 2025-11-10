/**
 * @file esp32_receptor_checksum_buzzer.ino
 * @brief Receptor FSK dual-core con detección automática de frecuencias
 * @details Detecta 4 frecuencias automáticamente y demodula 2 paquetes en paralelo
 */
#include "arduinoFFT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// --- Parámetros de Hardware y Señal ---
#define ADC_PIN 34
#define BUZZER_PIN 25
#define SAMPLES 2560   // Buffer para ambos paquetes
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Parámetros FSK ---
#define BIT_DURATION 64
#define SYM_FFT_N 64
const int PACKET_LEN_PKT1 = 13;  // Paquete 1: 1 sync + 4 msg + 4 conf + 4 checksum
const int PACKET_LEN_PKT2 = 17;  // Paquete 2: 1 sync + 8 msg + 4 conf + 4 checksum
const int MESSAGE_LEN_PKT1 = 4;
const int MESSAGE_LEN_PKT2 = 8;
const int CONF_LEN_PKT1 = 4;
const int CONF_LEN_PKT2 = 4;

// --- Configuración del Mensaje ---
const uint8_t expected_confirmation_bits_pkt1[4] = {1, 0, 1, 0};
const uint8_t expected_confirmation_bits_pkt2[4] = {1, 0, 1, 0};

// --- Buzzer Config ---
const int BUZZER_FREQ = 2000;    // Frecuencia del tono (Hz)
const int BIT_SOUND_DURATION = 100;  // Duración del sonido por bit 1 (ms)
const int BIT_PAUSE_DURATION = 100;  // Duración del silencio por bit 0 (ms)

// --- Variables para reproducción en bucle ---
volatile bool pkt1_received = false;
uint8_t saved_pkt1_message[8];
int saved_pkt1_length = 0;
volatile int target_frequency = 0;  // Frecuencia objetivo en Hz

// --- Control de alternancia de paquetes ---
volatile bool waiting_for_pkt1 = true;  // Empezamos esperando PKT1
volatile bool pkt2_received = false;

// --- Buffers Compartidos ---
double vReal[SAMPLES];
double vImag[SAMPLES];

// --- Frecuencias Detectadas (compartidas entre cores) ---
volatile bool frequencies_detected = false;  // Se detectarán en calibración
volatile int freq_0_pkt1 = 1000;
volatile int freq_1_pkt1 = 2000;
volatile int freq_0_pkt2 = 3000;
volatile int freq_1_pkt2 = 4000;

// --- Calibración ---
volatile bool calibration_mode = true;
volatile int calibration_step = 0;  // 0-7: PKT1(f0,f1,f0,f1), PKT2(f0,f1,f0,f1)
const int CALIBRATION_SAMPLES = 2048;  // ~256ms de muestreo @ 8kHz

// --- TFT ST7735 ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Máquina de Estados ---
enum State { WAITING_FOR_R, CALIBRATING, WAITING_FOR_SIGNAL, SAMPLING, DEMODULATING };
volatile State current_state = WAITING_FOR_R;
const double SIGNAL_THRESHOLD = 200.0;

// --- Mutex para proteger acceso a buffers ---
SemaphoreHandle_t xMutex = NULL;

// --- Resultado de paquetes ---
struct PacketResult {
  bool valid;
  uint8_t message[8];  // Máximo 8 bits (para PKT2)
  uint8_t full_packet[17];  // PKT2 ahora es 17 bits (máximo)
  double mags0[17];
  double mags1[17];
  int msg_len;
  int pkt_len;
};

volatile PacketResult pkt1_result;
volatile PacketResult pkt2_result;

// --- Recalibración bajo demanda ---
volatile bool recalibrate_requested = false;


// --- Función para detectar frecuencia durante calibración ---
int detectSingleFrequency(const double* signal, int sample_count) {
  static double r[1024], im[1024];
  int fft_size = (sample_count > 1024) ? 1024 : sample_count;
  
  for (int i = 0; i < fft_size; i++) {
    r[i] = signal[i];
    im[i] = 0.0;
  }
  
  ArduinoFFT<double> fft(r, im, fft_size, SAMPLING_FREQ);
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();
  
  // Buscar el pico máximo entre 500Hz y 5000Hz
  int min_bin = (500 * fft_size) / SAMPLING_FREQ;
  int max_bin = (5000 * fft_size) / SAMPLING_FREQ;
  
  int peak_bin = min_bin;
  double peak_mag = r[min_bin];
  
  for (int i = min_bin; i < max_bin && i < fft_size/2; i++) {
    if (r[i] > peak_mag) {
      peak_mag = r[i];
      peak_bin = i;
    }
  }
  
  int detected_freq = (peak_bin * SAMPLING_FREQ) / fft_size;
  return detected_freq;
}


// Variante que expone también las magnitudes en F0 y F1 (reutiliza la misma FFT por símbolo)
static void demodulateSymbolFFT_withMags(const double* in, int& bit_out, double& out_mag0, double& out_mag1, int f0, int f1) {
  static double r[SYM_FFT_N], im[SYM_FFT_N];
  for (int i = 0; i < SYM_FFT_N; i++) { r[i] = in[i]; im[i] = 0.0; }
  ArduinoFFT<double> f(r, im, SYM_FFT_N, SAMPLING_FREQ);
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  f.compute(FFTDirection::Forward);
  f.complexToMagnitude();
  int b0 = (int)round((double)f0 * SYM_FFT_N / SAMPLING_FREQ);
  int b1 = (int)round((double)f1 * SYM_FFT_N / SAMPLING_FREQ);
  out_mag0 = r[b0];
  out_mag1 = r[b1];
  bit_out = (out_mag1 > out_mag0) ? 1 : 0;
}

bool processPacket(const double* signal, PacketResult& result, int f0, int f1, int pkt_len, int msg_len, const uint8_t* expected_conf, const char* pkt_name, int conf_len, bool validate_checksum, bool validate_parity) {
  uint8_t received_packet[25];  // Máximo
  bool received_all_zeros = true;
  
  // Buscar el bit de sincronización (primer '1')
  int sync_offset = 0;
  bool sync_found = false;
  
  for (int offset = 0; offset < BIT_DURATION * 3 && !sync_found; offset++) {
    int bit_est = 0;
    double m0 = 0.0, m1 = 0.0;
    demodulateSymbolFFT_withMags(&signal[offset], bit_est, m0, m1, f0, f1);
    if (bit_est == 1 && m1 > m0 * 1.5) {  // Sync encontrado con buena confianza
      sync_offset = offset;
      sync_found = true;
    }
  }
  
  if (!sync_found) {
    sync_offset = 0;  // Usar offset 0 si no se encuentra
  }
  
  // Demodular todos los bits del paquete desde el sync
  double total_energy = 0.0;
  for (int k = 0; k < pkt_len; ++k) {
    int bit_est = 0;
    double m0 = 0.0, m1 = 0.0;
    demodulateSymbolFFT_withMags(&signal[sync_offset + k * BIT_DURATION], bit_est, m0, m1, f0, f1);
    received_packet[k] = (uint8_t)bit_est;
    if (bit_est == 1) received_all_zeros = false;
    result.mags0[k] = m0;
    result.mags1[k] = m1;
    total_energy += (m0 + m1);  // Acumular energía total
  }
  
  // Calcular energía promedio
  double avg_energy = total_energy / pkt_len;
  
  // Rechazar si la energía promedio es muy baja (probablemente ruido)
  if (avg_energy < 1000.0) {
    return false;  // Energía insuficiente, no es una señal válida
  }
  
  result.valid = false;
  result.msg_len = msg_len;
  result.pkt_len = pkt_len;
  
  if (received_all_zeros) {
    return false;
  }
  
  // Extraer mensaje y confirmación
  uint8_t received_message[8];
  uint8_t received_confirmation[8];
  
  for(int i = 0; i < msg_len; i++) {
    received_message[i] = received_packet[1 + i];
  }
  
  for(int i = 0; i < conf_len; i++) {
    received_confirmation[i] = received_packet[1 + msg_len + i];
  }
  
  // Validar confirmación
  bool confirmation_ok = true;
  for(int i = 0; i < conf_len; i++) {
    if (received_confirmation[i] != expected_conf[i]) {
      confirmation_ok = false;
      break;
    }
  }
  
  // Validar checksum solo si se requiere
  bool checksum_ok = true;
  if (validate_checksum) {
    uint8_t received_checksum[8];
    for(int i = 0; i < conf_len; i++) {  // Checksum tiene conf_len bits (4)
      received_checksum[i] = received_packet[1 + msg_len + conf_len + i];
    }
    
    // Validar: checksum[i] = mensaje[i] XOR confirmacion[i] (solo los primeros conf_len bits)
    for(int i = 0; i < conf_len; i++) {
      if ((received_message[i] ^ received_confirmation[i]) != received_checksum[i]) {
        checksum_ok = false;
        break;
      }
    }
  }
  
  // Validar paridad si se requiere
  bool parity_ok = true;
  if (validate_parity) {
    int ones_count = 0;
    for(int i = 0; i < pkt_len; i++) {
      if (received_packet[i] == 1) ones_count++;
    }
    // Paridad par: el total de 1s debe ser par
    parity_ok = (ones_count % 2 == 0);
  }
  
  // Solo mostrar debug si falló la validación (opcional, puedes comentar esta sección)
  
  if (!confirmation_ok || !checksum_ok || !parity_ok) {
    Serial.print("[");
    Serial.print(pkt_name);
    Serial.print("] Validación fallida - Conf:");
    Serial.print(confirmation_ok ? "OK" : "FAIL");
    if (validate_checksum) {
      Serial.print(" Chk:");
      Serial.print(checksum_ok ? "OK" : "FAIL");
    }
    if (validate_parity) {
      Serial.print(" Par:");
      Serial.print(parity_ok ? "OK" : "FAIL");
    }
    Serial.print(" Pkt:");
    for(int i=0; i<pkt_len && i<13; i++) Serial.print(received_packet[i]);
    if(pkt_len > 13) Serial.print("...");
    Serial.println();
  }
  
  
  if (confirmation_ok && checksum_ok && parity_ok) {
    result.valid = true;
    for (int i = 0; i < msg_len; i++) {
      result.message[i] = received_message[i];
    }
    for (int i = 0; i < pkt_len; i++) {
      result.full_packet[i] = received_packet[i];
    }
    
    Serial.print("\n[");
    Serial.print(pkt_name);
    Serial.print("] Válido (");
    Serial.print(msg_len);
    Serial.print(" bits, f0=");
    Serial.print(f0);
    Serial.print(", f1=");
    Serial.print(f1);
    Serial.print(") Msg: ");
    for(int i=0; i<msg_len; i++) Serial.print(received_message[i]);
    
    // Si es PKT2 (8 bits), mostrar como ASCII
    if (msg_len == 8) {
      uint8_t ascii_val = 0;
      for (int i = 0; i < 8; i++) {
        ascii_val = (ascii_val << 1) | received_message[i];
      }
    Serial.print(" = '");
    Serial.print((char)ascii_val);
    Serial.print("' (");
    Serial.print(ascii_val);
    Serial.print(")");
    
    // Mostrar espectrograma para PKT2
    Serial.println();
    Serial.println("  Espectrograma PKT2:");
    Serial.print("  Bit | Mag@f0 | Mag@f1 | Dec\n");
    for(int i=0; i<pkt_len; i++) {
      Serial.print("   ");
      if(i<10) Serial.print(" ");
      Serial.print(i);
      Serial.print(" | ");
      Serial.print((int)result.mags0[i]);
      Serial.print(" | ");
      Serial.print((int)result.mags1[i]);
      Serial.print(" | ");
      Serial.println(result.full_packet[i]);
    }
  }
  Serial.println();
  
  return true;
}  return false;
}

// --- Función para convertir 4 bits a frecuencia ---
int bitsToFrequency(const uint8_t* bits, int length) {
  // Convertir bits a valor decimal (0-15 para 4 bits)
  int value = 0;
  for (int i = 0; i < length; i++) {
    value = (value << 1) | bits[i];
  }
  
  // Mapear valores 0-15 a frecuencias entre 200Hz y 3000Hz
  // Puedes ajustar este rango según necesites
  int base_freq = 200;
  int freq_step = 200;  // Incrementos de 200Hz
  int frequency = base_freq + (value * freq_step);
  
  return frequency;
}

// --- Tabla de lookup para generar señal seno ---
const int SINE_TABLE_SIZE = 256;
uint8_t sineTable[SINE_TABLE_SIZE];

void generateSineTable() {
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    // Generar valores de 0-255 (rango del DAC de 8 bits)
    sineTable[i] = (uint8_t)(127.5 + 127.5 * sin(2.0 * PI * i / SINE_TABLE_SIZE));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Generar tabla de seno
  generateSineTable();
  
  Serial.println("\n========================================");
  Serial.println("  Receptor FSK Dual-Core");
  Serial.println("  Esperando 'R' para iniciar CALIBRACION");
  Serial.println("  (en cualquier momento presiona 'R' para recalibrar)");
  Serial.println("========================================");

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(true);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.println("ESP32 Receptor Dual");
  tft.setCursor(4, 16);
  tft.println("Esperando 'R'...");
  
  xMutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(core0Task, "Core0", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(buzzerTask, "SineWave", 4096, NULL, 1, NULL, 1);
  
  Serial.println("Sistema iniciado - PKT1: Frecuencia continua, PKT2: ASCII");
  Serial.println("Use 'R' para calibrar.");
}

// --- Mostrar en TFT cuando el mensaje es válido ---
static void tftShowDualPackets() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(2, 2);
  tft.println("FSK Dual Rx");
  
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(2, 14);
  tft.print("PKT1(");
  tft.print(freq_0_pkt1);
  tft.print(",");
  tft.print(freq_1_pkt1);
  tft.print("):");
  if (pkt1_result.valid) {
    for (int i = 0; i < pkt1_result.msg_len; i++) tft.print(pkt1_result.message[i]);
  } else {
    tft.print("NONE");
  }
  
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(2, 26);
  tft.print("PKT2(");
  tft.print(freq_0_pkt2);
  tft.print(",");
  tft.print(freq_1_pkt2);
  tft.print("):");
  if (pkt2_result.valid) {
    for (int i = 0; i < pkt2_result.msg_len; i++) tft.print(pkt2_result.message[i]);
    
    // Convertir a carácter ASCII
    uint8_t val = 0;
    for (int i = 0; i < pkt2_result.msg_len; i++) {
      val = (val << 1) | pkt2_result.message[i];
    }
    tft.print(" = '");
    tft.print((char)val);
    tft.print("'");
  } else {
    tft.print("NONE");
  }
  
  // Mostrar paquetes completos de forma compacta
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 42);
  tft.print("P1(");
  tft.print(pkt1_result.pkt_len);
  tft.print("):");
  if (pkt1_result.valid) {
    for (int i = 0; i < pkt1_result.pkt_len && i < 13; i++) tft.print(pkt1_result.full_packet[i]);
  }
  
  tft.setCursor(2, 54);
  tft.print("P2(");
  tft.print(pkt2_result.pkt_len);
  tft.print("):");
  if (pkt2_result.valid) {
    // Primera línea: primeros 13 bits
    for (int i = 0; i < 13 && i < pkt2_result.pkt_len; i++) tft.print(pkt2_result.full_packet[i]);
    
    // Segunda línea: resto
    if (pkt2_result.pkt_len > 13) {
      tft.setCursor(2, 66);
      for (int i = 13; i < pkt2_result.pkt_len; i++) tft.print(pkt2_result.full_packet[i]);
    }
  }
}

void core0Task(void * pvParameters) {
  Serial.println("[Core 0] Iniciado");
  
  while(1) {
    // Lectura no bloqueante de Serial para 'R'
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'R' || c == 'r') {
        recalibrate_requested = true;
      }
    }

    if (recalibrate_requested) {
      // Reset variables para recalibrar
      calibration_step = 0;
      calibration_mode = true;
      frequencies_detected = false;
      waiting_for_pkt1 = true;
      pkt1_received = false;
      pkt2_received = false;
      pkt1_result.valid = false;
      pkt2_result.valid = false;
      current_state = CALIBRATING;
      recalibrate_requested = false;
      Serial.println("\n[Recalibracion] Iniciando proceso de calibracion...");
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(2,2); tft.println("Recalibrando...");
    }

    if (current_state == WAITING_FOR_R) {
      // Sólo esperar a que el usuario pida calibración (recalibrate_requested lo activa)
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    if (current_state == CALIBRATING) {
      // Modo calibración: capturar 8 tonos de 3 segundos cada uno
      if (calibration_step < 8) {
        Serial.print("\n[CALIBRACION] Paso ");
        Serial.print(calibration_step + 1);
        Serial.println("/8 - Esperando señal...");
        
        // Esperar señal fuerte
        while (abs((double)analogRead(ADC_PIN) - 2048.0) < SIGNAL_THRESHOLD) {
          delay(10);
        }
        
        Serial.println("[CALIBRACION] Señal detectada, muestreando ~250ms...");
        
        // Capturar 2048 muestras @ 8kHz (~256ms de señal)
        unsigned long next_t = micros();
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
          while (micros() < next_t) { }
          vReal[i] = (double)analogRead(ADC_PIN);
          next_t += SAMPLING_PERIOD_US;
        }
        
        // Detectar frecuencia dominante
        int detected_freq = detectSingleFrequency(vReal, CALIBRATION_SAMPLES);
        
        Serial.print("[CALIBRACION] Frecuencia detectada: ");
        Serial.print(detected_freq);
        Serial.println(" Hz");
        
        // Almacenar según el paso
        switch(calibration_step) {
          case 0: freq_0_pkt1 = detected_freq; break;
          case 1: freq_1_pkt1 = detected_freq; break;
          case 2: // Validación freq_0_pkt1
            if (abs(detected_freq - freq_0_pkt1) > 100) {
              Serial.println("[CALIBRACION] ADVERTENCIA: Frecuencia inconsistente");
            }
            break;
          case 3: // Validación freq_1_pkt1
            if (abs(detected_freq - freq_1_pkt1) > 100) {
              Serial.println("[CALIBRACION] ADVERTENCIA: Frecuencia inconsistente");
            }
            break;
          case 4: freq_0_pkt2 = detected_freq; break;
          case 5: freq_1_pkt2 = detected_freq; break;
          case 6: // Validación freq_0_pkt2
            if (abs(detected_freq - freq_0_pkt2) > 100) {
              Serial.println("[CALIBRACION] ADVERTENCIA: Frecuencia inconsistente");
            }
            break;
          case 7: // Validación freq_1_pkt2
            if (abs(detected_freq - freq_1_pkt2) > 100) {
              Serial.println("[CALIBRACION] ADVERTENCIA: Frecuencia inconsistente");
            }
            break;
        }
        
        calibration_step++;
        
        // Esperar a que termine el tono de 3 segundos
        delay(2750);
        
        if (calibration_step >= 8) {
          // Calibración completa
          Serial.println("\n========== CALIBRACION COMPLETA ==========");
          Serial.print("PKT1 -> f0: ");
          Serial.print(freq_0_pkt1);
          Serial.print(" Hz, f1: ");
          Serial.print(freq_1_pkt1);
          Serial.println(" Hz");
          Serial.print("PKT2 -> f0: ");
          Serial.print(freq_0_pkt2);
          Serial.print(" Hz, f1: ");
          Serial.print(freq_1_pkt2);
          Serial.println(" Hz");
          Serial.println("==========================================\n");
          
          frequencies_detected = true;
          calibration_mode = false;
          current_state = WAITING_FOR_SIGNAL;
        }
      }
    }
    else if (current_state == WAITING_FOR_SIGNAL) {
      if (abs((double)analogRead(ADC_PIN) - 2048.0) > SIGNAL_THRESHOLD) {
        current_state = SAMPLING;
      }
    }
    else if (current_state == SAMPLING) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        unsigned long next_t = micros();
        for (int i = 0; i < SAMPLES; i++) {
          while (micros() < next_t) { }
          vReal[i] = (double)analogRead(ADC_PIN);
          vImag[i] = 0.0;
          next_t += SAMPLING_PERIOD_US;
        }
        
        double mean = 0;
        for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
        mean /= SAMPLES;
        for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;
        
        xSemaphoreGive(xMutex);
        
        // Ir directo a demodulación (frecuencias fijas)
        current_state = DEMODULATING;
      }
    }
    else if (current_state == DEMODULATING) {
      // Alternar entre PKT1 y PKT2 según lo que estemos esperando
      if (waiting_for_pkt1) {
        // Intentar demodular PKT1
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          processPacket(vReal, (PacketResult&)pkt1_result, freq_0_pkt1, freq_1_pkt1, PACKET_LEN_PKT1, MESSAGE_LEN_PKT1, expected_confirmation_bits_pkt1, "PKT1", CONF_LEN_PKT1, true, false);
          xSemaphoreGive(xMutex);
        }
        
        // Si PKT1 es válido, guardarlo y cambiar a esperar PKT2
        if (pkt1_result.valid) {
          saved_pkt1_length = pkt1_result.msg_len;
          for (int i = 0; i < saved_pkt1_length; i++) {
            saved_pkt1_message[i] = pkt1_result.message[i];
          }
          
          // Convertir bits a frecuencia
          target_frequency = bitsToFrequency(saved_pkt1_message, saved_pkt1_length);
          pkt1_received = true;
          
          Serial.print("[PKT1] Guardado! Bits: ");
          for (int i = 0; i < saved_pkt1_length; i++) {
            Serial.print(saved_pkt1_message[i]);
          }
          Serial.print(" -> Frecuencia: ");
          Serial.print(target_frequency);
          Serial.println(" Hz (reproduciendo continuamente)");
          
          Serial.println("[SISTEMA] PKT1 OK -> Esperando PKT2...");
          waiting_for_pkt1 = false;  // Cambiar a esperar PKT2
          
          // Delay para sincronizar con el siguiente paquete del transmisor
          delay(200);  // Esperar a que termine PKT1 y empiece PKT2
          
          // Mostrar espectrograma PKT1 en TFT
          tft.fillScreen(ST77XX_BLACK);
          tft.setTextSize(1);
          tft.setTextColor(ST77XX_CYAN);
          tft.setCursor(2, 2);
          tft.println("PKT1 Recibido");
          
          tft.setTextColor(ST77XX_GREEN);
          tft.setCursor(2, 14);
          tft.print("Msg: ");
          for (int i = 0; i < pkt1_result.msg_len; i++) {
            tft.print(pkt1_result.message[i]);
          }
          
          tft.setTextColor(ST77XX_WHITE);
          tft.setCursor(2, 26);
          tft.println("Espectrograma:");
          for (int i = 0; i < 8 && i < pkt1_result.pkt_len; i++) {
            tft.setCursor(2, 38 + i * 10);
            tft.print(i);
            tft.print(":");
            tft.print((int)pkt1_result.mags0[i]);
            tft.print("/");
            tft.print((int)pkt1_result.mags1[i]);
            tft.print("=");
            tft.print(pkt1_result.full_packet[i]);
          }
        }
      } else {
        // Intentar demodular PKT2 (ahora con checksum de 4 bits)
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          processPacket(vReal, (PacketResult&)pkt2_result, freq_0_pkt2, freq_1_pkt2, PACKET_LEN_PKT2, MESSAGE_LEN_PKT2, expected_confirmation_bits_pkt2, "PKT2", CONF_LEN_PKT2, true, false);
          xSemaphoreGive(xMutex);
        }
        
        // Si PKT2 es válido, validar que el carácter esté en el rango permitido
        if (pkt2_result.valid) {
          // Decodificar el valor ASCII de los 8 bits
          uint8_t ascii_val = 0;
          for (int i = 0; i < pkt2_result.msg_len; i++) {
            ascii_val = (ascii_val << 1) | pkt2_result.message[i];
          }
          
          // Validar que sea '0'-'9' (48-57) o 'A'-'D' (65-68)
          bool valid_char = (ascii_val >= 48 && ascii_val <= 57) || (ascii_val >= 65 && ascii_val <= 68);
          
          if (!valid_char) {
            Serial.print("[PKT2] Carácter inválido: '");
            Serial.print((char)ascii_val);
            Serial.print("' (");
            Serial.print(ascii_val);
            Serial.println(") - Solo se aceptan 0-9 o A-D");
            // No marcar como válido y continuar esperando
          } else {
            Serial.print("[SISTEMA] PKT2 OK (");
            Serial.print((char)ascii_val);
            Serial.println(") -> Volviendo a esperar PKT1...");
            pkt2_received = true;
            waiting_for_pkt1 = true;  // Volver a esperar PKT1
            
            // Delay para sincronizar con el siguiente paquete del transmisor
            delay(200);  // Esperar a que termine PKT2 y empiece PKT1
            
            // Mostrar ASCII y espectrograma PKT2 en TFT
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextSize(1);
            tft.setTextColor(ST77XX_CYAN);
            tft.setCursor(2, 2);
            tft.println("PKT2 ASCII");
            
            tft.setTextColor(ST77XX_YELLOW);
            tft.setCursor(2, 14);
            tft.print("Bits: ");
            for (int i = 0; i < pkt2_result.msg_len; i++) {
              tft.print(pkt2_result.message[i]);
            }
            
            tft.setCursor(2, 26);
            tft.print("ASCII: '");
            tft.print((char)ascii_val);
            tft.print("' (");
            tft.print(ascii_val);
            tft.print(")");
            
            tft.setTextColor(ST77XX_WHITE);
            tft.setCursor(2, 38);
            tft.println("Espectrograma:");
            for (int i = 0; i < 10 && i < pkt2_result.pkt_len; i++) {
              tft.setCursor(2, 50 + i * 10);
              tft.print(i);
              tft.print(":");
              tft.print((int)pkt2_result.mags0[i]);
              tft.print("/");
              tft.print((int)pkt2_result.mags1[i]);
              tft.print("=");
              tft.print(pkt2_result.full_packet[i]);
            }
          }
        }
      }
      
      // Volver directamente a SAMPLING para recibir el siguiente paquete rápidamente
      current_state = SAMPLING;
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void buzzerTask(void * pvParameters) {
  Serial.println("[Sine Wave Task] Iniciado en Core 1");
  
  // Variables para generación de onda
  unsigned long phase_accumulator = 0;
  unsigned long last_update = 0;
  
  while(1) {
    if (pkt1_received && target_frequency > 0) {
      // Generar señal seno continua usando DAC
      unsigned long current_micros = micros();
      
      // Calcular incremento de fase basado en la frecuencia objetivo
      // phase_increment = (frecuencia * SINE_TABLE_SIZE * 2^32) / sample_rate
      // Asumiendo sample_rate ~40kHz (25us por muestra)
      unsigned long phase_increment = ((unsigned long long)target_frequency * SINE_TABLE_SIZE * 4294967296ULL) / 40000UL;
      
      // Actualizar fase
      phase_accumulator += phase_increment;
      
      // Obtener índice de la tabla (usar los 8 bits más significativos)
      int table_index = (phase_accumulator >> 24) & 0xFF;
      
      // Escribir al DAC (pin 25 en ESP32 tiene DAC)
      dacWrite(BUZZER_PIN, sineTable[table_index]);
      
      // Control de timing para mantener ~40kHz de tasa de muestreo
      delayMicroseconds(25);
      
    } else {
      // No hay frecuencia, silencio
      dacWrite(BUZZER_PIN, 128);  // Valor medio (sin señal)
      delay(10);
    }
  }
}

void loop() {
  delay(1000);
}