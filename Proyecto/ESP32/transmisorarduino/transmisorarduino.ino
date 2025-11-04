/**
 * @file transmisorarduino.ino
 * @brief Transmisor FSK que envía un paquete de 12 bits con un checksum XOR personalizado.
 *        Adaptado para Arduino (mismo flujo y lógica que el transmisor original).
 */

// --- Configuración de Pines ---
const int FSK_PIN = 9; // Pin digital 9 en Arduino (equivalente al GP15 usado en Pico)

// --- Parámetros FSK ---
const int FREQ_0 = 1000;
const int FREQ_1 = 2000;
const long BIT_DURATION_US = 11000; // 11 ms

// --- Componentes del Mensaje ---
const uint8_t data_message[4] = {0, 1, 0, 1};
const uint8_t confirmation_bits[4] = {1, 0, 1, 0}; // Patrón de confirmación fijo
const int MESSAGE_LEN = 4;

// --- Paquete de Transmisión Completo (13 bits = 1 bit sync + 12 bits data) ---
const int PACKET_LEN = 13;
uint8_t transmission_packet[PACKET_LEN];

// --- Temporización (para transmisión continua) ---
const long HALF_PERIOD_0_US = 1000000 / (2 * FREQ_0);
const long HALF_PERIOD_1_US = 1000000 / (2 * FREQ_1);
int current_bit_index = 0;
unsigned long bit_start_time_us = 0;
unsigned long next_toggle_time_us = 0;
// Control de transmisión periódica
unsigned long last_packet_time_ms = 0;
bool is_transmitting = true; // iniciar transmitiendo el primer paquete
const unsigned long PACKET_INTERVAL_MS = 0; // sin delay, transmitir continuamente

void setup() {
  Serial.begin(115200);
  pinMode(FSK_PIN, OUTPUT);
  digitalWrite(FSK_PIN, LOW);
  delay(1000);

  // --- Construir el paquete de transmisión de 13 bits ---
  // Bit 0: Bit de sincronización (siempre 1)
  transmission_packet[0] = 1;
  
  uint8_t checksum_bits[MESSAGE_LEN];

  // 1. Calcular el checksum (XOR entre el mensaje y los bits de confirmación)
  for (int i = 0; i < MESSAGE_LEN; i++) {
    checksum_bits[i] = data_message[i] ^ confirmation_bits[i];
  }
  
  // 2. Ensamblar el paquete completo (desplazado +1 por el bit de sync)
  for (int i = 0; i < MESSAGE_LEN; i++) {
    transmission_packet[1 + i] = data_message[i];                              // Bits 1-4: Mensaje
    transmission_packet[1 + MESSAGE_LEN + i] = confirmation_bits[i];          // Bits 5-8: Confirmación
    transmission_packet[1 + 2 * MESSAGE_LEN + i] = checksum_bits[i];          // Bits 9-12: Checksum
  }
  
  Serial.println("Transmisor FSK con Checksum Personalizado (Arduino) iniciado...");
  Serial.print("Paquete completo a transmitir (13 bits, primero es sync=1): ");
  for (int i = 0; i < PACKET_LEN; i++) Serial.print(transmission_packet[i]);
  Serial.println();
}

void loop() {
  unsigned long current_time_us = micros();
  unsigned long current_time_ms = millis();

  // Si no estamos transmitiendo, comprobar si ha pasado el intervalo
  if (!is_transmitting) {
    if (current_time_ms - last_packet_time_ms >= PACKET_INTERVAL_MS) {
      // Iniciar nueva transmisión
      is_transmitting = true;
      current_bit_index = 0;
      bit_start_time_us = 0;
      next_toggle_time_us = 0;
      Serial.println("Iniciando transmisión de paquete...");
    } else {
      // Esperar sin hacer nada
      return;
    }
  }

  // Transmisión del bit actual
  if (bit_start_time_us == 0) {
    bit_start_time_us = current_time_us;
    next_toggle_time_us = current_time_us;
    // Reportar la frecuencia activa al inicio de cada bit
    int freq = (transmission_packet[current_bit_index] == 1) ? FREQ_1 : FREQ_0;
    Serial.print("Bit ");
    Serial.print(current_bit_index);
    Serial.print(" → modulando a ");
    Serial.print(freq);
    Serial.println(" Hz");
  }

  if (current_time_us >= next_toggle_time_us) {
    digitalWrite(FSK_PIN, !digitalRead(FSK_PIN));
    long half_period = (transmission_packet[current_bit_index] == 1) ? HALF_PERIOD_1_US : HALF_PERIOD_0_US;
    next_toggle_time_us += half_period;
  }

  if (current_time_us - bit_start_time_us >= BIT_DURATION_US) {
    current_bit_index++;
    bit_start_time_us = 0;

    if (current_bit_index >= PACKET_LEN) {
      // Pausa antes de la siguiente transmisión
      is_transmitting = false;
      last_packet_time_ms = millis();
      // Asegurar que la salida quede en LOW cuando no se transmite
      digitalWrite(FSK_PIN, LOW);
      Serial.println("Paquete transmitido. Esperando 1s para el siguiente.");
    }
  }
}