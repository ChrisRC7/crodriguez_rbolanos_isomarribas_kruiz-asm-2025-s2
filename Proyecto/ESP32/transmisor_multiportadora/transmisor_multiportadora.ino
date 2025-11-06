/**
 * @file transmisor_multiportadora.ino
 * @brief Transmisor que genera 2 señales simultáneas en portadoras diferentes:
 *        - Portadora 1: Señal piloto senoidal pura
 *        - Portadora 2: Mensaje de 4 bits modulado en FSK
 * @details Suma ambas señales, aplica filtro IIR pasa bajas y transmite por DAC (ESP32) o PWM (Pico)
 */

// --- Configuración de Hardware ---
#if defined(ESP32)
  #define DAC_PIN 25
  #define USE_DAC true
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO)
  #define PWM_PIN 0
  #define USE_DAC false
#else
  #error "Placa no soportada: define el pin DAC/PWM para tu plataforma."
#endif

// --- Filtro IIR Pasa Bajas (Butterworth de 2do orden, fc=3500 Hz, fs=8000 Hz) ---
// Coeficientes calculados para filtro IIR Butterworth
// b0, b1, b2 (numerador), a1, a2 (denominador, a0=1)
#define IIR_B0  0.4128015980
#define IIR_B1  0.8256031961
#define IIR_B2  0.4128015980
#define IIR_A1 -0.1311436995
#define IIR_A2  0.1823500916

// Variables de estado del filtro IIR
float iir_x1 = 0.0, iir_x2 = 0.0;  // Entradas anteriores
float iir_y1 = 0.0, iir_y2 = 0.0;  // Salidas anteriores

// --- Parámetros del Sistema ---
#define SAMPLING_FREQ 8000      // Hz - Frecuencia de muestreo
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// --- Portadora 1: Señal Piloto ---
#define PILOT_FREQ 500          // Hz - Frecuencia de la señal piloto senoidal
#define PILOT_AMPLITUDE 0.3     // Amplitud relativa (0.0 - 1.0)

// --- Portadora 2: FSK para mensaje de 4 bits ---
#define FSK_FREQ_0 1500         // Hz - Frecuencia para bit 0
#define FSK_FREQ_1 2500         // Hz - Frecuencia para bit 1
#define FSK_AMPLITUDE 0.7       // Amplitud relativa (0.0 - 1.0)
#define BIT_DURATION_MS 200     // ms por bit
#define SAMPLES_PER_BIT (SAMPLING_FREQ * BIT_DURATION_MS / 1000)

// --- Mensaje de 4 bits (quemado en código) ---
const uint8_t MESSAGE[4] = {1, 0, 1, 1};  // Mensaje a transmitir
const int MESSAGE_LEN = 4;

// --- Variables globales ---
unsigned long phase_pilot = 0;      // Fase acumulada para señal piloto
unsigned long phase_fsk = 0;        // Fase acumulada para FSK
int current_bit_index = 0;          // Índice del bit actual
int sample_count = 0;               // Contador de muestras en el bit actual
uint32_t freq_increment_pilot;      // Incremento de fase para piloto
uint32_t freq_increment_fsk_0;      // Incremento de fase para FSK bit 0
uint32_t freq_increment_fsk_1;      // Incremento de fase para FSK bit 1

// Tabla de seno precalculada (256 muestras para 0-2π)
const int SINE_TABLE_SIZE = 256;
int8_t sine_table[SINE_TABLE_SIZE];

// --- Setup ---
void setup() {
  Serial.begin(115200);
  // No usar delay; si se desea, breve espera activa
  unsigned long t0 = millis();
  while (millis() - t0 < 50) { /* no dormir */ }
  
  Serial.println("\n========================================");
  Serial.println("  Transmisor Multiportadora FSK");
  Serial.println("========================================");
  Serial.print("Portadora 1 (Piloto): "); Serial.print(PILOT_FREQ); Serial.println(" Hz");
  Serial.print("Portadora 2 (FSK): "); Serial.print(FSK_FREQ_0); Serial.print(" / ");
  Serial.print(FSK_FREQ_1); Serial.println(" Hz");
  Serial.print("Mensaje (4 bits): ");
  for(int i = 0; i < MESSAGE_LEN; i++) Serial.print(MESSAGE[i]);
  Serial.println("\n========================================\n");

  // Inicializar tabla de seno (-127 a +127)
  for(int i = 0; i < SINE_TABLE_SIZE; i++) {
    sine_table[i] = (int8_t)(127.0 * sin(2.0 * PI * i / SINE_TABLE_SIZE));
  }

  // Calcular incrementos de fase (DDS - Direct Digital Synthesis)
  // phase_increment = (freq * 2^32) / sampling_freq
  freq_increment_pilot = ((unsigned long long)PILOT_FREQ << 32) / SAMPLING_FREQ;
  freq_increment_fsk_0 = ((unsigned long long)FSK_FREQ_0 << 32) / SAMPLING_FREQ;
  freq_increment_fsk_1 = ((unsigned long long)FSK_FREQ_1 << 32) / SAMPLING_FREQ;

  // Configurar DAC o PWM
  #ifdef ESP32
    // DAC en ESP32 (0-255)
    dacWrite(DAC_PIN, 128);
  #else
    // PWM en Pico
    pinMode(PWM_PIN, OUTPUT);
    analogWriteFreq(SAMPLING_FREQ);
    analogWriteRange(255);
  #endif

  Serial.println("Transmitiendo...");
}

// --- Función para obtener seno desde tabla ---
inline int8_t get_sine(uint32_t phase) {
  // Tomar los 8 bits superiores de phase como índice
  uint8_t index = (phase >> 24);
  return sine_table[index];
}

// --- Filtro IIR Pasa Bajas ---
float apply_iir_lowpass(float input) {
  // Ecuación diferencial: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  float output = IIR_B0 * input + IIR_B1 * iir_x1 + IIR_B2 * iir_x2 
                 - IIR_A1 * iir_y1 - IIR_A2 * iir_y2;
  
  // Actualizar estados
  iir_x2 = iir_x1;
  iir_x1 = input;
  iir_y2 = iir_y1;
  iir_y1 = output;
  
  return output;
}

// --- Loop principal ---
void loop() {
  static unsigned long next_sample_time = micros();
  
  // Esperar con verificación no bloqueante (libera CPU periódicamente)
  unsigned long now = micros();
  if (now < next_sample_time) {
    unsigned long wait_us = next_sample_time - now;
    if (wait_us > 100) {
      delayMicroseconds(50);  // Micro-sleep si hay margen
    }
    // Espera fina
    while(micros() < next_sample_time) { /* busy wait */ }
  }
  next_sample_time += SAMPLING_PERIOD_US;

  // 1. Generar señal piloto (senoidal pura)
  int8_t pilot_sample = get_sine(phase_pilot);
  phase_pilot += freq_increment_pilot;

  // 2. Generar señal FSK (mensaje de 4 bits)
  uint8_t current_bit = MESSAGE[current_bit_index];
  uint32_t fsk_increment = (current_bit == 1) ? freq_increment_fsk_1 : freq_increment_fsk_0;
  int8_t fsk_sample = get_sine(phase_fsk);
  phase_fsk += fsk_increment;

  // 3. Sumar ambas señales con sus amplitudes
  float combined = (PILOT_AMPLITUDE * pilot_sample) + (FSK_AMPLITUDE * fsk_sample);
  
  // 4. Aplicar filtro IIR pasa bajas para suavizar la señal
  float filtered = apply_iir_lowpass(combined);
  
  // 5. Escalar a rango 0-255 para DAC
  int dac_value = (int)(127.5 + filtered);
  if(dac_value < 0) dac_value = 0;
  if(dac_value > 255) dac_value = 255;

  // 6. Enviar a DAC o PWM
  #ifdef ESP32
    dacWrite(DAC_PIN, dac_value);
  #else
    analogWrite(PWM_PIN, dac_value);
  #endif

  // 7. Avanzar al siguiente bit si es necesario
  sample_count++;
  if(sample_count >= SAMPLES_PER_BIT) {
    sample_count = 0;
    current_bit_index++;
    if(current_bit_index >= MESSAGE_LEN) {
      current_bit_index = 0;  // Repetir mensaje continuamente
      
      // Imprimir estadísticas cada ciclo completo
      static unsigned long last_print = 0;
      if(millis() - last_print > 2000) {
        last_print = millis();
        Serial.print("Tx: ");
        for(int i = 0; i < MESSAGE_LEN; i++) Serial.print(MESSAGE[i]);
        Serial.print(" | Piloto: "); Serial.print(PILOT_FREQ); Serial.print(" Hz");
        Serial.print(" | FSK: "); Serial.print(FSK_FREQ_0); Serial.print("/");
        Serial.print(FSK_FREQ_1); Serial.println(" Hz");
      }
    }
  }
}
