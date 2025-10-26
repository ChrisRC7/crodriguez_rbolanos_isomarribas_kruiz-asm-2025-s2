/**
 * @file arduino_transmisor.ino
 * @brief Transmisor FSK para Arduino usando la función tone().
 * @details Genera una onda cuadrada en un pin con frecuencias que
 * representan un '1' o un '0' lógico. Este método es no bloqueante.
 */

// --- Configuración de Pines ---
const int FSK_PIN = 9; // Pin de salida para la señal (debe ser un pin PWM)

// --- Parámetros FSK (deben coincidir con el receptor) ---
const int FREQ_0 = 1000; // Frecuencia para el bit '0' en Hz
const int FREQ_1 = 2000; // Frecuencia para el bit '1' en Hz
const int BIT_DURATION_MS = 8; // Cada bit dura 8 ms

// --- Mensaje de prueba (patrón alterno para diagnóstico) ---
const uint8_t data_bits[] = {1, 0, 1, 0, 1, 0, 1, 0};
const int num_bits = sizeof(data_bits) / sizeof(data_bits[0]);

// --- Variables de estado y temporización para la transmisión ---
int current_bit_index = 0;
unsigned long last_transmission_time = 0;
const long transmission_interval = 5000; // 5 segundos entre transmisiones
unsigned long bit_start_time_ms = 0;
bool is_transmitting = false;

void setup() {
  Serial.begin(9600); // El Arduino Uno suele usar 9600 baudios
  pinMode(FSK_PIN, OUTPUT);
  delay(1000);
  Serial.println("Transmisor FSK (Arduino) iniciado...");
  // Permite una transmisión inmediata al inicio
  last_transmission_time = -transmission_interval;
}

void loop() {
  unsigned long current_time_ms = millis();

  // 1. Iniciar una nueva transmisión si ha pasado el intervalo
  if (!is_transmitting && (current_time_ms - last_transmission_time >= transmission_interval)) {
    is_transmitting = true;
    current_bit_index = 0;
    bit_start_time_ms = current_time_ms;
    
    Serial.print("Iniciando transmision: ");
    for (int i=0; i < num_bits; i++) Serial.print(data_bits[i]);
    Serial.println();
  }

  // 2. Gestionar la transmisión activa bit a bit
  if (is_transmitting) {
    // Si el bit actual ya completó su duración, pasar al siguiente
    if (current_time_ms - bit_start_time_ms >= BIT_DURATION_MS) {
      bit_start_time_ms += BIT_DURATION_MS; // Actualizar el tiempo para el siguiente bit
      current_bit_index++;
    }

    // Si ya se enviaron todos los bits, finalizar la transmisión
    if (current_bit_index >= num_bits) {
      is_transmitting = false;
      noTone(FSK_PIN); // Detener la generación de la señal
      last_transmission_time = current_time_ms;
      Serial.println("Transmision completa.");
    } else {
      // Generar la frecuencia del bit actual usando tone()
      int current_freq = (data_bits[current_bit_index] == 1) ? FREQ_1 : FREQ_0;
      tone(FSK_PIN, current_freq);
    }
  }
}