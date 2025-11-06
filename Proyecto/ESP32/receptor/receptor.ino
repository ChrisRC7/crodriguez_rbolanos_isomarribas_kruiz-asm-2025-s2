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
#define SAMPLES 2560   // Capturar ambos paquetes: (13+25)*64 = 2432, uso 2560 para margen
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Parámetros FSK ---
#define BIT_DURATION 64
#define SYM_FFT_N 64
const int PACKET_LEN_PKT1 = 13;  // Paquete 1: 1 sync + 4 msg + 4 conf + 4 checksum
const int PACKET_LEN_PKT2 = 25;  // Paquete 2: 1 sync + 8 msg + 8 conf + 8 checksum
const int MESSAGE_LEN_PKT1 = 4;
const int MESSAGE_LEN_PKT2 = 8;
const int CONF_LEN_PKT1 = 4;
const int CONF_LEN_PKT2 = 8;

// --- Configuración del Mensaje ---
const uint8_t expected_confirmation_bits_pkt1[4] = {1, 0, 1, 0};
const uint8_t expected_confirmation_bits_pkt2[8] = {1, 0, 1, 0, 1, 0, 1, 0};

// --- Buzzer Config ---
const int BUZZER_FREQ = 2000;
const int BIT_SOUND_DURATION = 100;
const int BIT_PAUSE_DURATION = 100;

// --- Variables para reproducción en bucle ---
volatile bool pkt1_received = false;
uint8_t saved_pkt1_message[8];
int saved_pkt1_length = 0;

// --- Buffers Compartidos ---
double vReal[SAMPLES];
double vImag[SAMPLES];

// --- Frecuencias Detectadas (compartidas entre cores) ---
volatile bool frequencies_detected = true;  // Ya las conocemos
volatile int freq_0_pkt1 = 1000;
volatile int freq_1_pkt1 = 2000;
volatile int freq_0_pkt2 = 3000;
volatile int freq_1_pkt2 = 4000;

// --- TFT ST7735 ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Máquina de Estados ---
enum State { WAITING_FOR_SIGNAL, SAMPLING, DEMODULATING };
volatile State current_state = WAITING_FOR_SIGNAL;
const double SIGNAL_THRESHOLD = 200.0;

// --- Mutex para proteger acceso a buffers ---
SemaphoreHandle_t xMutex = NULL;

// --- Resultado de paquetes ---
struct PacketResult {
  bool valid;
  uint8_t message[8];  // Máximo 8 bits (para PKT2)
  uint8_t full_packet[25];  // Máximo 25 bits (para PKT2)
  double mags0[25];
  double mags1[25];
  int msg_len;
  int pkt_len;
};

volatile PacketResult pkt1_result;
volatile PacketResult pkt2_result;


// --- Función para detectar las 4 frecuencias dominantes ---
void detectFrequencies() {
  Serial.println("\n=== Detectando frecuencias ===");
  
  // Usar menos muestras para detección rápida
  const int DETECT_SAMPLES = 1024;  // Aumentado para mejor resolución
  static double r[DETECT_SAMPLES], im[DETECT_SAMPLES];
  
  for (int i = 0; i < DETECT_SAMPLES; i++) {
    r[i] = vReal[i];
    im[i] = 0.0;
  }
  
  ArduinoFFT<double> fft(r, im, DETECT_SAMPLES, SAMPLING_FREQ);
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();
  
  const int MIN_BIN = (500 * DETECT_SAMPLES) / SAMPLING_FREQ;
  const int MAX_BIN = (5000 * DETECT_SAMPLES) / SAMPLING_FREQ;
  
  struct Peak {
    int bin;
    double magnitude;
    int freq;
  };
  
  Peak peaks[4] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  
  // Buscar los 4 picos más altos, asegurando separación mínima de 700Hz
  const int MIN_SEPARATION_BINS = (700 * DETECT_SAMPLES) / SAMPLING_FREQ;
  
  for (int i = MIN_BIN; i < MAX_BIN && i < DETECT_SAMPLES/2; i++) {
    // Verificar que este pico esté suficientemente separado de los ya encontrados
    bool too_close = false;
    for (int p = 0; p < 4; p++) {
      if (peaks[p].magnitude > 0 && abs(i - peaks[p].bin) < MIN_SEPARATION_BINS) {
        too_close = true;
        break;
      }
    }
    
    if (!too_close) {
      for (int p = 0; p < 4; p++) {
        if (r[i] > peaks[p].magnitude) {
          for (int j = 3; j > p; j--) {
            peaks[j] = peaks[j-1];
          }
          peaks[p].bin = i;
          peaks[p].magnitude = r[i];
          peaks[p].freq = (i * SAMPLING_FREQ) / DETECT_SAMPLES;
          break;
        }
      }
    }
  }
  
  // Ordenar por frecuencia
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (peaks[j].freq < peaks[i].freq) {
        Peak temp = peaks[i];
        peaks[i] = peaks[j];
        peaks[j] = temp;
      }
    }
  }
  
  // Mostrar info de depuración
  Serial.println("  Picos detectados:");
  for (int i = 0; i < 4; i++) {
    Serial.print("    Pico ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(peaks[i].freq);
    Serial.print("Hz (mag: ");
    Serial.print(peaks[i].magnitude);
    Serial.println(")");
  }
  
  freq_0_pkt1 = peaks[0].freq;
  freq_1_pkt1 = peaks[1].freq;
  freq_0_pkt2 = peaks[2].freq;
  freq_1_pkt2 = peaks[3].freq;
  
  Serial.print("  Asignacion final:\n");
  Serial.print("    PKT1: f0=");
  Serial.print(freq_0_pkt1);
  Serial.print("Hz, f1=");
  Serial.println(freq_1_pkt1);
  Serial.print("    PKT2: f0=");
  Serial.print(freq_0_pkt2);
  Serial.print("Hz, f1=");
  Serial.println(freq_1_pkt2);
  
  frequencies_detected = true;
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
  for (int k = 0; k < pkt_len; ++k) {
    int bit_est = 0;
    double m0 = 0.0, m1 = 0.0;
    demodulateSymbolFFT_withMags(&signal[sync_offset + k * BIT_DURATION], bit_est, m0, m1, f0, f1);
    received_packet[k] = (uint8_t)bit_est;
    if (bit_est == 1) received_all_zeros = false;
    result.mags0[k] = m0;
    result.mags1[k] = m1;
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
    for(int i = 0; i < msg_len; i++) {
      received_checksum[i] = received_packet[1 + msg_len + conf_len + i];
    }
    
    for(int i = 0; i < msg_len; i++) {
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
  /*
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
  */
  
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

// --- Función para reproducir el mensaje en el buzzer ---
void playMessageOnBuzzer(const uint8_t* message, int length, const char* label) {
  Serial.print("\n[Buzzer] ");
  Serial.println(label);
  
  for (int i = 0; i < length; i++) {
    if (message[i] == 1) {
      tone(BUZZER_PIN, BUZZER_FREQ, BIT_SOUND_DURATION);
      delay(BIT_SOUND_DURATION);
    } else {
      noTone(BUZZER_PIN);
      delay(BIT_SOUND_DURATION);
    }
    noTone(BUZZER_PIN);
    delay(BIT_PAUSE_DURATION);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(ADC_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println("\n========================================");
  Serial.println("  Receptor FSK Dual-Core");
  Serial.println("  Frecuencias fijas:");
  Serial.print("    PKT1: ");
  Serial.print(freq_0_pkt1);
  Serial.print("/");
  Serial.print(freq_1_pkt1);
  Serial.println(" Hz");
  Serial.print("    PKT2: ");
  Serial.print(freq_0_pkt2);
  Serial.print("/");
  Serial.print(freq_1_pkt2);
  Serial.println(" Hz");
  Serial.println("========================================");;

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(true);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.println("ESP32 Receptor Dual");
  tft.setCursor(4, 16);
  tft.println("Esperando senal...");
  
  xMutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(core0Task, "Core0", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(buzzerTask, "Buzzer", 4096, NULL, 1, NULL, 1);
  
  Serial.println("Sistema iniciado - PKT1: Buzzer bucle, PKT2: ASCII");
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
    if (current_state == WAITING_FOR_SIGNAL) {
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
      // Demodular PKT1 (buzzer)
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        processPacket(vReal, (PacketResult&)pkt1_result, freq_0_pkt1, freq_1_pkt1, PACKET_LEN_PKT1, MESSAGE_LEN_PKT1, expected_confirmation_bits_pkt1, "PKT1", CONF_LEN_PKT1, true, false);
        xSemaphoreGive(xMutex);
      }
      
      // Demodular PKT2 (ASCII) - sin delay
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        processPacket(vReal, (PacketResult&)pkt2_result, freq_0_pkt2, freq_1_pkt2, PACKET_LEN_PKT2, MESSAGE_LEN_PKT2, expected_confirmation_bits_pkt2, "PKT2", CONF_LEN_PKT2, true, false);
        xSemaphoreGive(xMutex);
      }
      
      // Si PKT1 es válido, guardarlo para reproducción en bucle
      if (pkt1_result.valid && !pkt1_received) {
        saved_pkt1_length = pkt1_result.msg_len;
        for (int i = 0; i < saved_pkt1_length; i++) {
          saved_pkt1_message[i] = pkt1_result.message[i];
        }
        pkt1_received = true;
        Serial.println("\n[PKT1] Guardado! Reproduciendo en bucle...");
      }
      
      // Si PKT2 es válido, mostrar ASCII en TFT
      if (pkt2_result.valid) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_CYAN);
        tft.setCursor(2, 2);
        tft.println("FSK Receptor");
        
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(2, 16);
        tft.print("PKT2 (");
        tft.print(freq_0_pkt2);
        tft.print("/");
        tft.print(freq_1_pkt2);
        tft.println("Hz)");
        
        tft.setTextColor(ST77XX_GREEN);
        tft.setCursor(2, 28);
        tft.print("Bits: ");
        for (int i = 0; i < pkt2_result.msg_len; i++) {
          tft.print(pkt2_result.message[i]);
        }
        
        tft.setCursor(2, 40);
        tft.print("ASCII: '");
        uint8_t val = 0;
        for (int i = 0; i < pkt2_result.msg_len; i++) {
          val = (val << 1) | pkt2_result.message[i];
        }
        tft.print((char)val);
        tft.print("' (");
        tft.print(val);
        tft.println(")");
        
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(2, 54);
        tft.print("Paquete:");
        tft.setCursor(2, 64);
        for (int i = 0; i < pkt2_result.pkt_len && i < 13; i++) {
          tft.print(pkt2_result.full_packet[i]);
        }
      }
      
      current_state = WAITING_FOR_SIGNAL;
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void buzzerTask(void * pvParameters) {
  Serial.println("[Buzzer Task] Iniciado en Core 1");
  
  while(1) {
    if (pkt1_received) {
      // Reproducir el mensaje guardado en bucle infinito
      for (int i = 0; i < saved_pkt1_length; i++) {
        if (saved_pkt1_message[i] == 1) {
          tone(BUZZER_PIN, BUZZER_FREQ, BIT_SOUND_DURATION);
          delay(BIT_SOUND_DURATION);
        } else {
          noTone(BUZZER_PIN);
          delay(BIT_SOUND_DURATION);
        }
        noTone(BUZZER_PIN);
        delay(BIT_PAUSE_DURATION);
      }
      // Pausa entre repeticiones
      delay(500);
    } else {
      // Esperar hasta que se reciba PKT1
      delay(100);
    }
  }
}

void loop() {
  delay(1000);
}