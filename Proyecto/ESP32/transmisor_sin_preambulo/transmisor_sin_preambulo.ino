/**
 * @file transmisor_sin_preambulo.ino
 * @brief Transmisor FSK simplificado sin preámbulo de piloto.
 * @details Transmisión directa con sync pattern [1,0] para alineación.
 */

// --- Configuración de Pines ---
const int FSK_OUT_PIN = 13; // Pin de salida FSK → Conectar al Pin 34 del receptor

// --- Parámetros FSK (Optimizado para 18 kHz) ---
const int FREQ_0 = 800;      // Frecuencia para bit 0 (0.8 kHz)
const int FREQ_1 = 2400;     // Frecuencia para bit 1 (2.4 kHz)
const long BIT_DURATION_US = 16000; // ✅ 16 ms por bit = 288 muestras @ 18 kHz (mejor SNR)

// --- Componentes del Mensaje ---
const uint8_t data_message[4] = {1, 1, 1, 1};
const uint8_t confirmation_bits[4] = {1, 0, 1, 0}; // Patrón de confirmación fijo
const int MESSAGE_LEN = 4;

// --- Paquete de Transmisión Completo (12 bits) ---
const int PACKET_LEN = 12;
uint8_t transmission_packet[PACKET_LEN];

// --- Sincronización (2 bits) ---
const int SYNC_LEN = 2;
const uint8_t sync_bits[SYNC_LEN] = {1, 0}; // Patrón de sync: 1 luego 0
const int TX_LEN = SYNC_LEN + PACKET_LEN;   // Longitud total a transmitir (sync + payload)
uint8_t tx_bits[TX_LEN];

// --- Temporización ---
const long HALF_PERIOD_0_US = 1000000 / (2 * FREQ_0);
const long HALF_PERIOD_1_US = 1000000 / (2 * FREQ_1);

int current_bit_index = 0;
unsigned long last_transmission_time = 0;
const long transmission_interval = 500; // Pausa de 0.5 segundos entre envíos
unsigned long bit_start_time_us = 0;
unsigned long next_data_toggle_us = 0;
bool is_transmitting = false;

void setup() {
  Serial.begin(115200);
  pinMode(FSK_OUT_PIN, OUTPUT);
  digitalWrite(FSK_OUT_PIN, LOW);
  delay(1000);

  // --- Construir el paquete de transmisión de 12 bits ---
  uint8_t checksum_bits[MESSAGE_LEN];

  // 1. Calcular el checksum (XOR entre el mensaje y los bits de confirmación)
  for (int i = 0; i < MESSAGE_LEN; i++) {
    checksum_bits[i] = data_message[i] ^ confirmation_bits[i];
  }
  
  // 2. Ensamblar el paquete completo
  for (int i = 0; i < MESSAGE_LEN; i++) {
    transmission_packet[i] = data_message[i];                   // Primeros 4: Mensaje
    transmission_packet[i + MESSAGE_LEN] = confirmation_bits[i];  // Siguientes 4: Confirmación
    transmission_packet[i + 2 * MESSAGE_LEN] = checksum_bits[i];    // Últimos 4: Checksum
  }
  
  // 3. Construir vector de transmisión: [SYNC] + [PAYLOAD 12 bits]
  for (int i = 0; i < SYNC_LEN; ++i) tx_bits[i] = sync_bits[i];
  for (int i = 0; i < PACKET_LEN; ++i) tx_bits[SYNC_LEN + i] = transmission_packet[i];
  
  Serial.println("========================================");
  Serial.println("  Transmisor FSK sin Preambulo");
  Serial.println("  Optimizado para Receptor @ 18 kHz");
  Serial.println("========================================");
  Serial.print("Bits a transmitir (SYNC+PAYLOAD): ");
  for(int i=0; i < TX_LEN; i++) Serial.print(tx_bits[i]);
  Serial.println();
  Serial.println("\n--- Configuracion de Frecuencias ---");
  Serial.printf("FREQ_0 (bit 0):  %d Hz\n", FREQ_0);
  Serial.printf("FREQ_1 (bit 1):  %d Hz (Separacion: %d Hz)\n", FREQ_1, FREQ_1 - FREQ_0);
  Serial.printf("Duracion bit:    %ld ms\n", BIT_DURATION_US / 1000);
  Serial.printf("Duracion paquete: %ld ms\n", (BIT_DURATION_US * TX_LEN) / 1000);
  Serial.printf("Intervalo entre transmisiones: %ld ms\n\n", transmission_interval);
  Serial.println("Conectar Pin 13 (FSK_OUT) al Pin 34 del receptor");
  Serial.println("========================================\n");
  
  last_transmission_time = millis() - transmission_interval;
}

void loop() {
  unsigned long current_time_ms = millis();
  unsigned long current_time_us = micros();
  
  // --- Iniciar nueva transmisión ---
  if (!is_transmitting && (current_time_ms - last_transmission_time >= transmission_interval)) {
    is_transmitting = true;
    current_bit_index = 0;
    Serial.print("[TX] ");
    for(int i=0; i < TX_LEN; i++) Serial.print(tx_bits[i]);
    Serial.print(" | ");
  }
  
  if (is_transmitting) {
    // Inicializar el bit actual
    if (current_bit_index < TX_LEN && bit_start_time_us == 0) {
      bit_start_time_us = current_time_us;
      next_data_toggle_us = current_time_us;
    }
    
    // Generar la señal FSK durante la duración del bit
    if (current_bit_index < TX_LEN && (current_time_us - bit_start_time_us < BIT_DURATION_US)) {
      if (current_time_us >= next_data_toggle_us) {
        digitalWrite(FSK_OUT_PIN, !digitalRead(FSK_OUT_PIN));
        long half_period = (tx_bits[current_bit_index] == 1) ? HALF_PERIOD_1_US : HALF_PERIOD_0_US;
        next_data_toggle_us += half_period;
      }
    } else if (current_bit_index < TX_LEN) {
      // Avanzar al siguiente bit
      current_bit_index++;
      bit_start_time_us = 0;
    } else {
      // Transmisión completa
      is_transmitting = false;
      digitalWrite(FSK_OUT_PIN, LOW);
      last_transmission_time = current_time_ms;
      Serial.println("OK");
    }
  } else {
    // Cuando no estamos transmitiendo, mantener FSK_OUT en bajo
    digitalWrite(FSK_OUT_PIN, LOW);
  }
}
