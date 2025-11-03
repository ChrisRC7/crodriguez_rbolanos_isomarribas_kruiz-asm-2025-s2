/**
 * @file receptor_dual_core.ino
 * @brief Receptor dual-core para ESP32 que procesa 2 señales simultáneas:
 *        - Core 0 (Receptor 1): Escanea espectro, sintoniza señal piloto, reproduce en parlante
 *        - Core 1 (Receptor 2): Escanea espectro, demodula FSK de 4 bits, muestra en TFT
 * @details Usa FreeRTOS para procesamiento paralelo en ambos cores
 */

#include "arduinoFFT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// --- Parámetros de Hardware ---
#define ADC_PIN 34              // Pin de entrada ADC compartido
#define DAC_AUDIO_PIN 25        // DAC para salida de audio (parlante)

// --- Parámetros de Muestreo ---
#define SAMPLING_FREQ 8000
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)
#define FFT_SIZE 256            // Tamaño de FFT para escaneo (reducido para menor carga)
#define SCAN_SAMPLES 256        // Muestras para escaneo inicial

// --- Bandas esperadas para robustecer el escaneo ---
#define PILOT_BAND_MIN 300      // Hz
#define PILOT_BAND_MAX 700      // Hz
#define FSK_BAND_MIN   1200     // Hz
#define FSK_BAND_MAX   2800     // Hz

// --- Parámetros FSK (Receptor 2) ---
#define FSK_SAMPLES 64          // Muestras usadas por FFT por símbolo
// Duración de bit esperada (debe coincidir con el transmisor)
#define FSK_BIT_DURATION_MS 200
#define FSK_SAMPLES_PER_BIT (SAMPLING_FREQ * FSK_BIT_DURATION_MS / 1000)
#define MESSAGE_LEN 4

// --- TFT ST7735 ---
#define TFT_RST    22
#define TFT_CS     19
#define TFT_DC     5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Utilidad: espera activa basada en millis sin usar delay
static inline void busyWaitMs(uint32_t ms) {
  unsigned long t0 = millis();
  while ((millis() - t0) < ms) {
    // Ceder al scheduler sin dormir el CPU profundo
    taskYIELD();
  }
}

// --- Variables compartidas entre cores ---
volatile double shared_adc_buffer[FFT_SIZE];
volatile bool buffer_ready = false;
SemaphoreHandle_t adc_mutex;
SemaphoreHandle_t buffer_semaphore;
// Handles de tareas para diagnóstico
static TaskHandle_t hTaskCore0 = nullptr;
static TaskHandle_t hTaskCore1 = nullptr;

volatile float detected_freq_1 = 0;  // Banda 1 (más baja)
volatile float detected_freq_2 = 0;  // Centro de la banda FSK (promedio f0/f1)
volatile bool frequencies_locked = false;
// Buffers globales para FFT de escaneo (evitan uso excesivo de stack)
static double scan_vReal[FFT_SIZE];
static double scan_vImag[FFT_SIZE];
// Frecuencias FSK detectadas (dentro de la banda FSK)
volatile float detected_fsk_f0 = 0;
volatile float detected_fsk_f1 = 0;

// --- Filtro IIR Pasa Banda (Butterworth de 2do orden) ---
struct IIR_BandpassFilter {
  // Coeficientes del filtro (calculados dinámicamente)
  float b0, b1, b2;  // Numerador
  float a1, a2;      // Denominador (a0 = 1)
  
  // Estados del filtro
  float x1, x2;  // Entradas anteriores
  float y1, y2;  // Salidas anteriores
  
  // Inicializar filtro con frecuencia central y ancho de banda
  void init(float center_freq, float bandwidth, float fs) {
    // Biquad pasabanda (Audio EQ Cookbook)
    // Usamos Q = center_freq / bandwidth (aprox). Asegurar límites razonables.
    if (bandwidth < 1.0f) bandwidth = 1.0f;
    float Q = center_freq / bandwidth;
    if (Q < 0.25f) Q = 0.25f;           // evitar Q demasiado bajo (inestable)
    if (Q > 20.0f) Q = 20.0f;           // evitar Q excesivo

    float w0 = 2.0f * PI * center_freq / fs;
    float cw = cosf(w0);
    float sw = sinf(w0);
    float alpha = sw / (2.0f * Q);

    float a0 = 1.0f + alpha;
    b0 =  alpha / a0;
    b1 =  0.0f / a0;
    b2 = -alpha / a0;
    a1 = (-2.0f * cw) / a0;
    a2 = (1.0f - alpha) / a0;

    // Reset de estados
    x1 = x2 = 0.0f;
    y1 = y2 = 0.0f;
  }
  
  // Procesar una muestra
  float process(float input) {
    // Ecuación diferencial IIR: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    
    // Actualizar estados
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    
    return output;
  }
};

// Reemplazar la estructura anterior con la nueva
#define BandpassFilter IIR_BandpassFilter

// =============================================================================
// CORE 0 - RECEPTOR 1: Señal Piloto -> Parlante
// =============================================================================

IIR_BandpassFilter filter_core0;
volatile bool core0_ready = false;

void task_receptor1(void *parameter) {
  Serial.println("[Core 0] Receptor 1 iniciado - Señal Piloto -> Parlante");
  
  // Configurar DAC para salida de audio
  dacWrite(DAC_AUDIO_PIN, 128);
  
  // Esperar a que se detecten las frecuencias (sin delay)
  while(!frequencies_locked) {
    taskYIELD();
  }
  
  Serial.print("[Core 0] Sintonizando frecuencia piloto: ");
  Serial.print(detected_freq_1);
  Serial.println(" Hz");
  
  // Inicializar filtro IIR pasabanda para la señal piloto
  // Ancho de banda estrecho (150 Hz) para mejor selectividad
  filter_core0.init(detected_freq_1, 150.0, SAMPLING_FREQ);
  
  Serial.println("[Core 0] Filtro IIR pasabanda configurado:");
  Serial.print("  Centro: "); Serial.print(detected_freq_1); Serial.println(" Hz");
  Serial.println("  Ancho de banda: 150 Hz");
  Serial.println("  Tipo: Butterworth 2do orden");
  
  double local_buffer[128];
  int buffer_index = 0;
  unsigned long next_sample = micros();
  
  core0_ready = true;
  
  // Loop principal - Muestrear, filtrar y reproducir
  while(true) {
    // Muestrear ADC con verificación de tiempo
    unsigned long now = micros();
    if (now >= next_sample) {
      next_sample += SAMPLING_PERIOD_US;
      
      int adc_val = analogRead(ADC_PIN);
      float sample = (float)(adc_val - 2048);  // Centrar en 0
      
      // Aplicar filtro IIR pasabanda
      float filtered = filter_core0.process(sample);
      
      // Amplificar (ganancia ajustable)
      filtered *= 4.0;  // Ganancia aumentada para compensar atenuación del filtro
      
      // Limitar para evitar saturación
      if(filtered > 2047) filtered = 2047;
      if(filtered < -2048) filtered = -2048;
      
      // Enviar a DAC (0-255)
      int dac_val = (int)(127.5 + filtered / 16.0);
      if(dac_val < 0) dac_val = 0;
      if(dac_val > 255) dac_val = 255;
      dacWrite(DAC_AUDIO_PIN, dac_val);
      
      // Buffer para monitoreo (opcional)
      local_buffer[buffer_index++] = filtered;
      if(buffer_index >= 128) {
        buffer_index = 0;
        
        // Imprimir estadísticas cada segundo
        static unsigned long last_print = 0;
        if(millis() - last_print > 1000) {
          last_print = millis();
          
          // Calcular RMS para indicar nivel de señal
          float rms = 0;
          for(int i = 0; i < 128; i++) {
            rms += local_buffer[i] * local_buffer[i];
          }
          rms = sqrt(rms / 128.0);
          
          Serial.print("[Core 0] Audio RMS: ");
          Serial.print(rms, 1);
          Serial.print(" | Freq: ");
          Serial.print(detected_freq_1);
          Serial.println(" Hz");
        }
      }
    }
    
    // Dormir brevemente cada ~1ms (8 muestras) para que IDLE task pueda correr
    static uint16_t sample_counter = 0;
    if (++sample_counter >= 8) {
      sample_counter = 0;
      vTaskDelay(1);
    }
  }
}

// =============================================================================
// CORE 1 - RECEPTOR 2: FSK de 4 bits -> TFT
// =============================================================================

IIR_BandpassFilter filter_core1;
volatile bool core1_ready = false;

// Demodulación FSK usando FFT por símbolo
void demodulateSymbolFFT(const double* in, int& bit_out, double& mag0, double& mag1, 
                         float freq0, float freq1) {
  static double r[64], im[64];                   // Buffers locales para la FFT (parte real e imaginaria)
  for(int i = 0; i < 64; i++) {                  // Copiar 64 muestras del símbolo al buffer real
    r[i] = in[i];                                 // Asignar muestra i a la parte real
    im[i] = 0.0;                                  // Inicializar parte imaginaria en 0
  }
  
  ArduinoFFT<double> f(r, im, 64, SAMPLING_FREQ); // Instanciar FFT de 64 puntos a fs=8 kHz
  f.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Aplicar ventana Hamming para reducir fugas espectrales
  f.compute(FFTDirection::Forward);               // Calcular la FFT directa
  f.complexToMagnitude();                         // Convertir a magnitud (|X[k]|)
  
  int b0 = (int)round((double)freq0 * 64.0 / SAMPLING_FREQ); // Bin correspondiente a f0
  int b1 = (int)round((double)freq1 * 64.0 / SAMPLING_FREQ); // Bin correspondiente a f1
  
  mag0 = r[b0];                                   // Magnitud en f0
  mag1 = r[b1];                                   // Magnitud en f1
  bit_out = (mag1 > mag0) ? 1 : 0;                // Decidir el bit: 1 si f1 domina, 0 en caso contrario
}

void task_receptor2(void *parameter) {
  Serial.println("[Core 1] Receptor 2 iniciado - FSK 4 bits -> TFT");
  
  // Esperar a que se detecten las frecuencias (sin delay)
  while(!frequencies_locked) {
    taskYIELD();
  }
  
  Serial.print("[Core 1] Sintonizando FSK en torno a: ");
  Serial.print(detected_fsk_f0); Serial.print("/"); Serial.print(detected_fsk_f1);
  Serial.println(" Hz");
  
  // Inicializar filtro IIR pasabanda para la señal FSK
  // Centro entre f0 y f1, ancho que cubra ambas con margen
  float fsk_center = (detected_fsk_f0 + detected_fsk_f1) * 0.5f;
  float fsk_bw = fabsf(detected_fsk_f1 - detected_fsk_f0) + 400.0f; // margen 400 Hz
  if (fsk_bw < 800.0f) fsk_bw = 800.0f; // evitar BW demasiado estrecho
  filter_core1.init(fsk_center, fsk_bw, SAMPLING_FREQ);
  
  Serial.println("[Core 1] Filtro IIR pasabanda configurado:");
  Serial.print("  Centro: "); Serial.print(fsk_center); Serial.println(" Hz");
  Serial.print("  Ancho de banda: "); Serial.print(fsk_bw); Serial.println(" Hz");
  Serial.println("  Tipo: Butterworth 2do orden");
  
  // Frecuencias FSK detectadas por escaneo
  float fsk_freq_0 = detected_fsk_f0;
  float fsk_freq_1 = detected_fsk_f1;
  
  Serial.print("[Core 1] Esperando FSK en: ");
  Serial.print(fsk_freq_0); Serial.print(" Hz (bit 0) / ");
  Serial.print(fsk_freq_1); Serial.println(" Hz (bit 1)");
  
  uint8_t received_bits[MESSAGE_LEN];             // Buffer para almacenar los 4 bits demodulados
  double symbol_buffer[FSK_SAMPLES];              // Últimas 64 muestras de cada bit para la FFT
  static unsigned long last_ui = 0; // control no bloqueante de actualización UI
  
  core1_ready = true;
  
  // Loop principal - Demodular FSK
  while(true) {                                   // Bucle infinito del receptor FSK
    unsigned long next_sample = micros();         // Próximo instante de muestreo en micros
    for(int bit_idx = 0; bit_idx < MESSAGE_LEN; bit_idx++) { // Iterar por los 4 bits del mensaje
      double circ[FSK_SAMPLES];                   // Buffer circular de 64 muestras
      int head = 0;                               // Índice de inserción en el buffer circular
      for (int n = 0; n < FSK_SAMPLES_PER_BIT; n++) { // Recoger 1600 muestras (200 ms por bit)
        while (micros() < next_sample) {          // Espera activa corta hasta el siguiente tick de muestreo
          /* espera fina */
        }
        next_sample += SAMPLING_PERIOD_US;        // Programar el siguiente muestreo a 8 kHz

        int adc_val = analogRead(ADC_PIN);        // Leer el ADC
        float sample = (float)(adc_val - 2048);   // Centrar la señal alrededor de 0
        double x = filter_core1.process(sample);  // Filtrar con pasabanda FSK

        circ[head] = x;                           // Guardar la muestra filtrada en el buffer circular
        head = (head + 1) % FSK_SAMPLES;          // Avanzar cabeza circular (0..63)

        if ((n & 511) == 0) taskYIELD();          // Ceder CPU ocasionalmente para evitar WDT
      }

      for (int i = 0; i < FSK_SAMPLES; i++) {     // Reconstruir las últimas 64 muestras en orden temporal
        int idx = (head + i) % FSK_SAMPLES;        // Calcular índice circular
        symbol_buffer[i] = circ[idx];             // Copiar al buffer para FFT del símbolo
      }

      int bit; double mag0, mag1;                 // Variables para resultado y magnitudes en f0/f1
      demodulateSymbolFFT(symbol_buffer,          // Demodular este símbolo usando FFT de 64 puntos
                          bit, mag0, mag1,       // Entradas/salidas de la función
                          fsk_freq_0, fsk_freq_1); // Frecuencias FSK detectadas (f0/f1)
      received_bits[bit_idx] = bit;               // Guardar el bit decidido

      taskYIELD();                                 // Ceder CPU entre bits
    }
    
    // Actualización UI/Serial no bloqueante cada 500 ms
    if (millis() - last_ui >= 500) {
      last_ui = millis();

      // Mostrar en TFT
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(1);
      tft.setCursor(2, 2);
      tft.println("FSK Receptor 2");
      
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(2, 20);
      tft.print("Freq: ");
      tft.print((int)detected_freq_2);
      tft.println(" Hz");
      
      tft.setCursor(2, 35);
      tft.print("Mensaje (4b): ");
      for(int i = 0; i < MESSAGE_LEN; i++) {
        tft.print(received_bits[i]);
      }

      // Imprimir también en Serial
      Serial.print("[Core 1] Mensaje recibido: ");
      for(int i = 0; i < MESSAGE_LEN; i++) {
        Serial.print(received_bits[i]);
      }
      Serial.println();
    }
    // Ceder brevemente sin dormir
    taskYIELD();
  }
}

// =============================================================================
// ESCANEO DE ESPECTRO (ejecutado en setup antes de lanzar tasks)
// =============================================================================

void scanSpectrum() {
  Serial.println("\n[Escaneo] Analizando espectro de frecuencias...");

  // Capturar muestras
  unsigned long next_t = micros();
  for(int i = 0; i < FFT_SIZE; i++) {
    // Espera fina sin monopolizar CPU
    while(micros() < next_t) { /* espera activa corta */ }
    scan_vReal[i] = (double)analogRead(ADC_PIN);
    scan_vImag[i] = 0.0;
    next_t += SAMPLING_PERIOD_US;
    if ((i & 31) == 0) vTaskDelay(1); // cada 32 muestras, ceder para IDLE
  }
  
  // Remover DC offset
  double mean = 0;
  for(int i = 0; i < FFT_SIZE; i++) mean += scan_vReal[i];
  mean /= FFT_SIZE;
  for(int i = 0; i < FFT_SIZE; i++) scan_vReal[i] -= mean;
  
  // Calcular FFT
  ArduinoFFT<double> FFT(scan_vReal, scan_vImag, FFT_SIZE, SAMPLING_FREQ);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  // Buscar pico de PILOTO dentro de su banda
  double pilot_peak_mag = 0.0;
  float pilot_peak_freq = 0.0f;
  int i_min_pilot = (int)ceil((double)PILOT_BAND_MIN * FFT_SIZE / SAMPLING_FREQ);
  int i_max_pilot = (int)floor((double)PILOT_BAND_MAX * FFT_SIZE / SAMPLING_FREQ);
  if (i_min_pilot < 1) i_min_pilot = 1;
  if (i_max_pilot > (FFT_SIZE/2 - 1)) i_max_pilot = (FFT_SIZE/2 - 1);
  for (int i = i_min_pilot; i <= i_max_pilot; ++i) {
    double mag = scan_vReal[i];
    if (mag > pilot_peak_mag) {
      pilot_peak_mag = mag;
      pilot_peak_freq = (i * SAMPLING_FREQ) / (float)FFT_SIZE;
    }
  }
  detected_freq_1 = pilot_peak_freq;
  // Si el pico está lejos de 500 Hz más de 100 Hz, forzar 500 Hz (robustez)
  if (fabsf(detected_freq_1 - 500.0f) > 100.0f) {
    detected_freq_1 = 500.0f;
  }

  // Buscar dos picos más prominentes dentro de la banda FSK
  double fsk_peak_mag[2] = {0.0, 0.0};
  float  fsk_peak_freq[2] = {0.0f, 0.0f};
  int i_min_fsk = (int)ceil((double)FSK_BAND_MIN * FFT_SIZE / SAMPLING_FREQ);
  int i_max_fsk = (int)floor((double)FSK_BAND_MAX * FFT_SIZE / SAMPLING_FREQ);
  if (i_min_fsk < 1) i_min_fsk = 1;
  if (i_max_fsk > (FFT_SIZE/2 - 1)) i_max_fsk = (FFT_SIZE/2 - 1);
  for (int i = i_min_fsk; i <= i_max_fsk; ++i) {
    double mag = scan_vReal[i];
    float freq = (i * SAMPLING_FREQ) / (float)FFT_SIZE;
    if (mag > fsk_peak_mag[0]) {
      // desplazar el primero al segundo
      fsk_peak_mag[1] = fsk_peak_mag[0];
      fsk_peak_freq[1] = fsk_peak_freq[0];
      fsk_peak_mag[0] = mag;
      fsk_peak_freq[0] = freq;
    } else if (mag > fsk_peak_mag[1]) {
      fsk_peak_mag[1] = mag;
      fsk_peak_freq[1] = freq;
    }
  }
  // Orden ascendente
  if (fsk_peak_freq[0] > fsk_peak_freq[1]) {
    float tf = fsk_peak_freq[0]; fsk_peak_freq[0] = fsk_peak_freq[1]; fsk_peak_freq[1] = tf;
    double tm = fsk_peak_mag[0]; fsk_peak_mag[0] = fsk_peak_mag[1]; fsk_peak_mag[1] = tm;
  }

  detected_fsk_f0 = fsk_peak_freq[0];
  detected_fsk_f1 = fsk_peak_freq[1];
  // Fallbacks si no se detectan picos válidos
  if (detected_freq_1 < PILOT_BAND_MIN || detected_freq_1 > PILOT_BAND_MAX) {
    detected_freq_1 = 500.0f;
  }
  if (detected_fsk_f0 < FSK_BAND_MIN || detected_fsk_f1 < FSK_BAND_MIN || fabsf(detected_fsk_f1 - detected_fsk_f0) < 300.0f) {
    detected_fsk_f0 = 1500.0f;
    detected_fsk_f1 = 2500.0f;
  }
  detected_freq_2 = (detected_fsk_f0 + detected_fsk_f1) * 0.5f;
  frequencies_locked = true;
  
  Serial.print("[Escaneo] Piloto: ");
  Serial.print(detected_freq_1);
  Serial.println(" Hz");

  Serial.print("[Escaneo] FSK f0/f1: ");
  Serial.print(detected_fsk_f0); Serial.print(" / "); Serial.print(detected_fsk_f1);
  Serial.println(" Hz");
}

// =============================================================================
// SETUP Y LOOP PRINCIPAL
// =============================================================================

void setup() {
  Serial.begin(115200);
  // No usar delay: pequeña espera activa para estabilizar Serial si es necesario
  busyWaitMs(200);
  
  Serial.println("\n\n========================================");
  Serial.println("  Receptor Dual-Core ESP32");
  Serial.println("  Core 0: Piloto -> Parlante");
  Serial.println("  Core 1: FSK -> TFT");
  Serial.println("========================================\n");
  
  // Configurar hardware
  pinMode(ADC_PIN, INPUT);
  
  // Inicializar TFT
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.println("ESP32 Dual-Core Rx");
  tft.setCursor(4, 20);
  tft.println("Escaneando...");
  
  // Crear semáforos
  adc_mutex = xSemaphoreCreateMutex();
  buffer_semaphore = xSemaphoreCreateBinary();
  
  // Escanear espectro para detectar las 2 bandas (sin delay)
  busyWaitMs(100);
  scanSpectrum();
  
  tft.setCursor(4, 40);
  tft.print("Banda 1: ");
  tft.print((int)detected_freq_1);
  tft.println(" Hz");
  tft.setCursor(4, 55);
  tft.print("Banda 2: ");
  tft.print((int)detected_freq_2);
  tft.println(" Hz");
  
  busyWaitMs(200);
  
  // Lanzar tareas en los dos cores
  xTaskCreatePinnedToCore(
    task_receptor1,      // Función
    "Receptor1_Audio",   // Nombre
    6144,                // Stack size (6 KB)
    NULL,                // Parámetros
    1,                   // Prioridad
    &hTaskCore0,         // Handle
    0                    // Core 0
  );
  
  xTaskCreatePinnedToCore(
    task_receptor2,      // Función
    "Receptor2_FSK",     // Nombre
    10240,               // Stack size (10 KB)
    NULL,                // Parámetros
    1,                   // Prioridad
    &hTaskCore1,         // Handle
    1                    // Core 1
  );
  
  Serial.println("\n[Main] Ambos receptores lanzados en cores separados");
}

void loop() {
  // El loop principal puede quedar vacío o monitorear estado
  static unsigned long last_status = 0;
  if(millis() - last_status > 5000) {
    last_status = millis();
    Serial.println("\n[Main] Sistema operando...");
    Serial.print("  Core 0 (Audio): ");
    Serial.println(core0_ready ? "Activo" : "Iniciando");
    Serial.print("  Core 1 (FSK):   ");
    Serial.println(core1_ready ? "Activo" : "Iniciando");
    // Diagnóstico de recursos
    Serial.print("  Free heap: "); Serial.println(ESP.getFreeHeap());
    if (hTaskCore0) {
      UBaseType_t wm0 = uxTaskGetStackHighWaterMark(hTaskCore0);
      Serial.print("  Stack watermark Core0: "); Serial.println((uint32_t)wm0);
    }
    if (hTaskCore1) {
      UBaseType_t wm1 = uxTaskGetStackHighWaterMark(hTaskCore1);
      Serial.print("  Stack watermark Core1: "); Serial.println((uint32_t)wm1);
    }
  }
  // No usar delay; iteración rápida con yield para no bloquear WDT
  taskYIELD();
}
