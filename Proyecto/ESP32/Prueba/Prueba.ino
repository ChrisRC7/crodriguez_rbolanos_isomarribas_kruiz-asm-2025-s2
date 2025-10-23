/*
 * Modulador FSK con FFT - ESP32
 * Genera señal FSK y analiza las frecuencias mediante FFT
 * Muestra resultados en Monitor Serial
 * Usa micros() para timing preciso
 */

#include "arduinoFFT.h"

// Parámetros FSK
#define SAMPLES 256              // Muestras para FFT (debe ser potencia de 2)
#define SAMPLING_FREQ 8000       // Frecuencia de muestreo en Hz
#define FREQ_0 1000              // Frecuencia para bit "0" (Hz)
#define FREQ_1 2000              // Frecuencia para bit "1" (Hz)
#define BIT_DURATION 50          // Duración de cada bit en muestras

// Intervalo de muestreo en microsegundos
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQ)

// Datos a modular (ejemplo: secuencia binaria)
const uint8_t data_bits[] = {1, 0, 1, 1, 0, 1, 0, 0};
const int num_bits = sizeof(data_bits);

// Variables FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// Variables de control de tiempo
unsigned long last_analysis_time = 0;
unsigned long analysis_interval = 2000000; // 2 segundos en microsegundos

// Estados
enum State {
  GENERATING,
  COMPUTING,
  DISPLAYING,
  WAITING
};

State current_state = GENERATING;

void setup() {
  Serial.begin(115200);
  delay(2000); // Esperar a que se estabilice el puerto serial
  
  Serial.println("\n\n\n========================================");
  Serial.println("    MODULADOR FSK CON FFT - ESP32");
  Serial.println("========================================");
  Serial.printf("Frecuencia bit '0': %d Hz\n", FREQ_0);
  Serial.printf("Frecuencia bit '1': %d Hz\n", FREQ_1);
  Serial.printf("Frecuencia muestreo: %d Hz\n", SAMPLING_FREQ);
  Serial.printf("Periodo muestreo: %d us\n", SAMPLING_PERIOD_US);
  Serial.printf("Muestras FFT: %d\n", SAMPLES);
  Serial.println("========================================\n");
  
  last_analysis_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  
  switch (current_state) {
    case GENERATING:
      generateFSKSignal();
      current_state = COMPUTING;
      break;
      
    case COMPUTING:
      computeFFT();
      current_state = DISPLAYING;
      break;
      
    case DISPLAYING:
      displayResults();
      current_state = WAITING;
      last_analysis_time = current_time;
      break;
      
    case WAITING:
      // Esperar el intervalo usando micros()
      if ((current_time - last_analysis_time) >= analysis_interval) {
        current_state = GENERATING;
      }
      break;
  }
}

void generateFSKSignal() {
  Serial.println("Generando señal FSK...");
  Serial.print("Secuencia binaria: ");
  
  for (int i = 0; i < num_bits; i++) {
    Serial.print(data_bits[i]);
  }
  Serial.println("\n");
  
  // Limpiar arrays
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = 0;
    vImag[i] = 0;
  }
  
  // Timestamp inicial para muestreo preciso
  unsigned long start_time = micros();
  unsigned long next_sample_time = start_time;
  
  // Generar señal modulada con timing preciso
  int sample_idx = 0;
  for (int bit = 0; bit < num_bits && sample_idx < SAMPLES; bit++) {
    float frequency = (data_bits[bit] == 1) ? FREQ_1 : FREQ_0;
    
    // Generar muestras para este bit
    for (int s = 0; s < BIT_DURATION && sample_idx < SAMPLES; s++) {
      // Esperar hasta el próximo tiempo de muestreo
      while (micros() < next_sample_time) {
        // Espera activa para precisión
      }
      
      // Calcular tiempo exacto transcurrido
      unsigned long elapsed_us = micros() - start_time;
      float t = elapsed_us / 1000000.0; // Convertir a segundos
      
      // Generar muestra
      vReal[sample_idx] = sin(2.0 * PI * frequency * t);
      vImag[sample_idx] = 0;
      
      sample_idx++;
      next_sample_time += SAMPLING_PERIOD_US;
    }
  }
  
  unsigned long total_time = micros() - start_time;
  Serial.printf("Muestras generadas: %d\n", sample_idx);
  Serial.printf("Tiempo de generación: %lu us\n", total_time);
  Serial.printf("Tiempo esperado: %lu us\n\n", (unsigned long)(SAMPLES * SAMPLING_PERIOD_US));
}

void computeFFT() {
  Serial.println("Calculando FFT...");
  unsigned long start_time = micros();
  
  // Aplicar ventana de Hamming
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  
  // Calcular FFT
  FFT.compute(FFTDirection::Forward);
  
  // Calcular magnitudes
  FFT.complexToMagnitude();
  
  unsigned long compute_time = micros() - start_time;
  Serial.printf("FFT calculada en %lu us\n\n", compute_time);
}

void displayResults() {
  Serial.println("========================================");
  Serial.println("         ANÁLISIS DE FRECUENCIAS");
  Serial.println("========================================");
  
  // Encontrar picos dominantes con separación mínima
  const int MIN_PEAK_SEPARATION = 15; // bins mínimos entre picos
  double peak1_mag = 0, peak2_mag = 0;
  int peak1_idx = 0, peak2_idx = 0;
  
  // Buscar el primer pico más grande
  for (int i = 2; i < (SAMPLES/2); i++) {
    if (vReal[i] > peak1_mag) {
      peak1_mag = vReal[i];
      peak1_idx = i;
    }
  }
  
  // Buscar el segundo pico más grande, suficientemente separado del primero
  for (int i = 2; i < (SAMPLES/2); i++) {
    // Ignorar picos cercanos al primer pico
    if (abs(i - peak1_idx) > MIN_PEAK_SEPARATION) {
      if (vReal[i] > peak2_mag) {
        peak2_mag = vReal[i];
        peak2_idx = i;
      }
    }
  }
  
  // Calcular frecuencias reales
  double freq1 = (peak1_idx * 1.0 * SAMPLING_FREQ) / SAMPLES;
  double freq2 = (peak2_idx * 1.0 * SAMPLING_FREQ) / SAMPLES;
  
  // Asegurar que freq1 < freq2
  if (freq1 > freq2) {
    double temp = freq1;
    freq1 = freq2;
    freq2 = temp;
    
    double temp_mag = peak1_mag;
    peak1_mag = peak2_mag;
    peak2_mag = temp_mag;
  }
  
  // Mostrar resultados principales
  Serial.println("FRECUENCIAS DETECTADAS:");
  Serial.println("------------------------");
  Serial.printf("  Frecuencia 1: %.2f Hz (Magnitud: %.2f)\n", freq1, peak1_mag);
  Serial.printf("  Frecuencia 2: %.2f Hz (Magnitud: %.2f)\n\n", freq2, peak2_mag);
  
  // Mostrar top 10 frecuencias para debug
  Serial.println("TOP 10 COMPONENTES DE FRECUENCIA:");
  Serial.println("----------------------------------");
  for (int rank = 0; rank < 10; rank++) {
    double max_mag = 0;
    int max_idx = 0;
    for (int i = 2; i < (SAMPLES/2); i++) {
      bool already_shown = false;
      // Verificar si ya fue mostrado
      for (int j = 0; j < rank; j++) {
        // Marcar como mostrado si está muy cerca de uno anterior
        if (abs(i - max_idx) < 2) already_shown = true;
      }
      if (!already_shown && vReal[i] > max_mag) {
        max_mag = vReal[i];
        max_idx = i;
      }
    }
    double freq = (max_idx * 1.0 * SAMPLING_FREQ) / SAMPLES;
    Serial.printf("  %d. %.2f Hz → Magnitud: %.2f\n", rank + 1, freq, max_mag);
  }
  Serial.println();
  
  Serial.println("IDENTIFICACIÓN FSK:");
  Serial.println("-------------------");
  Serial.printf("  Bit '0' esperado: %d Hz → Detectado: %.2f Hz\n", FREQ_0, freq1);
  Serial.printf("  Bit '1' esperado: %d Hz → Detectado: %.2f Hz\n", FREQ_1, freq2);
  
  // Calcular error
  float error_0 = abs(freq1 - FREQ_0);
  float error_1 = abs(freq2 - FREQ_1);
  Serial.printf("\n  Error bit '0': %.2f Hz (%.2f%%)\n", error_0, (error_0/FREQ_0)*100);
  Serial.printf("  Error bit '1': %.2f Hz (%.2f%%)\n", error_1, (error_1/FREQ_1)*100);
  
  // Mostrar resolución de frecuencia
  float freq_resolution = (float)SAMPLING_FREQ / SAMPLES;
  Serial.printf("\n  Resolución FFT: %.2f Hz/bin\n", freq_resolution);
  
  Serial.println("\n========================================");
  
  // Gráfico ASCII simple de las frecuencias principales
  Serial.println("\nGRÁFICO DE ESPECTRO (Frecuencias principales):");
  Serial.println("------------------------------------------------");
  printBarGraph("Freq 0", freq1, peak1_mag, peak1_mag > peak2_mag ? peak1_mag : peak2_mag);
  printBarGraph("Freq 1", freq2, peak2_mag, peak1_mag > peak2_mag ? peak1_mag : peak2_mag);
  Serial.println("------------------------------------------------\n");
  
  Serial.println("\n");
}

void printBarGraph(const char* label, float freq, float magnitude, float max_mag) {
  Serial.printf("%s (%.0fHz): ", label, freq);
  int bar_length = (int)((magnitude / max_mag) * 40);
  for (int i = 0; i < bar_length; i++) {
    Serial.print("█");
  }
  Serial.printf(" %.1f\n", magnitude);
}