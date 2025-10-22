// --- Configuración de la Señal ---
#define DAC_PIN 25                // Pin de salida de audio. Puedes usar GPIO 25 o 26.
#define SINE_FREQ 0.1           // Frecuencia de la señal piloto en Hz. ¡Puedes cambiarla!
#define SINE_TABLE_SIZE 256       // Tamaño de la tabla de consulta. 256 es un buen valor.


hw_timer_t *timer = NULL;         // Puntero para el objeto del temporizador.
uint8_t sin_table[SINE_TABLE_SIZE]; // Arreglo para guardar la onda pre-calculada.
volatile int table_index = 0;     // Índice para recorrer la tabla. 'volatile' 

/**
 * @brief Interrupt Service Routine (ISR) - Se ejecuta cada vez que el temporizador se dispara.
 * Su única tarea es enviar el siguiente valor de la tabla al DAC.
 * IRAM_ATTR asegura que la función se cargue en la RAM para una ejecución más rápida.
 */
void IRAM_ATTR onTimer() {
  dacWrite(DAC_PIN, sin_table[table_index]); // Escribe el valor actual en el DAC.
  table_index++;                             // Avanza al siguiente valor.
  if (table_index == SINE_TABLE_SIZE) {      // Si llegamos al final de la tabla,
    table_index = 0;                         // vuelve al inicio.
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando generador de señal piloto (API actualizada)...");

  // --- 1. Generar la Tabla de Consulta (Lookup Table) ---
  // Pre-calculamos un ciclo completo de la onda sinusoidal.
  // El DAC del ESP32 tiene 8 bits (0-255). La onda se mapea a este rango.
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    // La fórmula mapea una sinusoide de amplitud 1 a un rango de 0-255
    sin_table[i] = (uint8_t)(127.5 + 127.5 * sin(2 * PI * i / SINE_TABLE_SIZE));
  }
  
  // ===================== SECCIÓN CORREGIDA =====================
  
  // --- 2. Configurar el Temporizador de Hardware (API actualizada) ---
  // Ahora, timerBegin() especifica la frecuencia del tick del temporizador.
  // Usaremos 1 MHz (1,000,000 ticks por segundo) para que los cálculos sean en microsegundos.
  timer = timerBegin(1000000); // 1 MHz clock frequency. No se necesita el número de timer ni prescaler.
  
  // Asociamos nuestra función 'onTimer' al temporizador.
  // La nueva función ya no necesita el tercer argumento 'true'.
  timerAttachInterrupt(timer, &onTimer);
  
  // Frecuencia de la interrupción:
  // Queremos 'SINE_TABLE_SIZE' interrupciones por cada ciclo de 'SINE_FREQ'.
  uint32_t timer_interrupt_frequency = SINE_FREQ * SINE_TABLE_SIZE;
  
  // Calculamos el valor de la alarma en microsegundos (porque nuestro timer tick es de 1 µs).
  uint64_t alarmValue = 1000000 / timer_interrupt_frequency;
  
  // Configuramos y habilitamos la alarma en un solo paso usando timerAlarm().
  // El 'true' significa que la alarma se recargará automáticamente.
  // El '0' final es para un contador de alarmas que no usaremos.
  timerAlarm(timer, alarmValue, true, 0);

  // =================== FIN DE SECCIÓN CORREGIDA ==================

  Serial.printf("Señal de %.2f Hz generada en el pin %d\n", (float)SINE_FREQ, DAC_PIN);
}

void loop() {
  // El loop principal queda libre.
  // Todo el trabajo de generación de la señal lo hace el temporizador en segundo plano.
  delay(1000);
}