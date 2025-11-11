/**
 * @file transmisorarduino.ino
 * @brief Transmisor FSK dual que envía 2 paquetes alternadamente:
 *        - Paquete 1: f0=1000Hz, f1=2000Hz (mensaje de 4 bits)
 *        - Paquete 2: f0=3000Hz, f1=4000Hz (carácter ASCII convertido a 4 bits)
 */

// --- Configuración de Pines ---
const int FSK_PIN = 9; // Pin digital 9 en Arduino

// --- Parámetros FSK Paquete 1 ---
const int FREQ_0_PKT1 = 1000;
const int FREQ_1_PKT1 = 2000;

// --- Parámetros FSK Paquete 2 ---
const int FREQ_0_PKT2 = 3000;
const int FREQ_1_PKT2 = 4000;

const long BIT_DURATION_US = 8000; // 8 ms

// --- Modo de Calibración ---
const long CALIBRATION_BIT_DURATION_MS = 3000; // 3 segundos por bit
const uint8_t calibration_sequence[] = {0, 1, 0, 1}; // Secuencia para calibrar
const int CALIBRATION_LENGTH = 4;
bool calibration_done = false;

// --- Componentes del Paquete 1 ---
const uint8_t data_message_pkt1[4] = {0, 1, 0, 1};
const uint8_t confirmation_bits_pkt1[4] = {1, 0, 1, 0};
const int MESSAGE_LEN_PKT1 = 4;
const int PACKET_LEN_PKT1 = 13;  // 1 sync + 4 mensaje + 4 confirmación + 4 checksum

// --- Componentes del Paquete 2 (Carácter ASCII) ---
char ascii_char = 'A'; // Carácter a enviar (cualquier ASCII)
uint8_t data_message_pkt2[8];  // 8 bits para ASCII completo
const uint8_t confirmation_bits_pkt2[4] = {1, 0, 1, 0};
const int MESSAGE_LEN_PKT2 = 8;
const int CONF_LEN_PKT2 = 4;
const int PACKET_LEN_PKT2 = 17;  // 1 sync + 8 mensaje + 4 confirmación + 4 checksum

// --- Paquetes de Transmisión ---
uint8_t transmission_packet_1[PACKET_LEN_PKT1];
uint8_t transmission_packet_2[PACKET_LEN_PKT2];

// --- Control de Transmisión ---
int current_packet = 1; // 1 o 2
int current_bit_index = 0;
unsigned long bit_start_time_us = 0;
unsigned long next_toggle_time_us = 0;

// --- Función para convertir carácter ASCII a 8 bits ---
void asciiTo8Bits(char c, uint8_t* bits) {
  uint8_t value = (uint8_t)c;
  
  // Convertir a 8 bits (MSB primero)
  for (int i = 0; i < 8; i++) {
    bits[i] = (value >> (7 - i)) & 1;
  }
}

// --- Función para construir un paquete ---
void buildPacket(uint8_t* packet, const uint8_t* data_msg, const uint8_t* conf_bits, int msg_len) {
  uint8_t* checksum_bits = new uint8_t[msg_len];
  
  // Calcular checksum (XOR entre mensaje y confirmación)
  for (int i = 0; i < msg_len; i++) {
    checksum_bits[i] = data_msg[i] ^ conf_bits[i];
  }
  
  // Ensamblar el paquete
  packet[0] = 1; // Bit de sincronización
  
  for (int i = 0; i < msg_len; i++) {
    packet[1 + i] = data_msg[i];                    // Mensaje
    packet[1 + i + msg_len] = conf_bits[i];         // Confirmación
    packet[1 + i + 2 * msg_len] = checksum_bits[i]; // Checksum
  }
  
  delete[] checksum_bits;
}

// --- Función para construir paquete 2 (con checksum de 4 bits) ---
void buildPacket2(uint8_t* packet, const uint8_t* data_msg, const uint8_t* conf_bits, int msg_len, int conf_len) {
  // Ensamblar el paquete: sync + mensaje (8 bits) + confirmación (4 bits) + checksum (4 bits)
  packet[0] = 1; // Bit de sincronización
  
  // Copiar mensaje (8 bits)
  for (int i = 0; i < msg_len; i++) {
    packet[1 + i] = data_msg[i];
  }
  
  // Copiar confirmación (4 bits)
  for (int i = 0; i < conf_len; i++) {
    packet[1 + msg_len + i] = conf_bits[i];
  }
  
  // Calcular checksum (XOR entre los primeros 4 bits del mensaje y confirmación)
  for (int i = 0; i < conf_len; i++) {
    packet[1 + msg_len + conf_len + i] = data_msg[i] ^ conf_bits[i];
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(FSK_PIN, OUTPUT);
  digitalWrite(FSK_PIN, LOW);
  delay(1000);

  // --- Construir Paquete 1 (13 bits: 4 bits mensaje) ---
  buildPacket(transmission_packet_1, data_message_pkt1, confirmation_bits_pkt1, MESSAGE_LEN_PKT1);
  
  // --- Construir Paquete 2 (17 bits: 8 bits ASCII + 4 bits confirmación + 4 bits checksum) ---
  asciiTo8Bits(ascii_char, data_message_pkt2);
  buildPacket2(transmission_packet_2, data_message_pkt2, confirmation_bits_pkt2, MESSAGE_LEN_PKT2, CONF_LEN_PKT2);
  
  Serial.println("=== Transmisor FSK Dual (Arduino) ===");
  Serial.println();
  
  // Información Paquete 1
  Serial.println("--- PAQUETE 1 (13 bits) ---");
  Serial.print("Frecuencias: f0=");
  Serial.print(FREQ_0_PKT1);
  Serial.print("Hz, f1=");
  Serial.print(FREQ_1_PKT1);
  Serial.println("Hz");
  Serial.print("Paquete: ");
  for(int i=0; i < PACKET_LEN_PKT1; i++) Serial.print(transmission_packet_1[i]);
  Serial.println();
  Serial.print("Mensaje (4 bits): ");
  for(int i=0; i < MESSAGE_LEN_PKT1; i++) Serial.print(data_message_pkt1[i]);
  Serial.println();
  
  // Información Paquete 2
  Serial.println();
  Serial.println("--- PAQUETE 2 (25 bits) ---");
  Serial.print("Frecuencias: f0=");
  Serial.print(FREQ_0_PKT2);
  Serial.print("Hz, f1=");
  Serial.print(FREQ_1_PKT2);
  Serial.println("Hz");
  Serial.print("Caracter ASCII: '");
  Serial.print(ascii_char);
  Serial.print("' (");
  Serial.print((int)ascii_char);
  Serial.println(")");
  Serial.print("Paquete: ");
  for(int i=0; i < PACKET_LEN_PKT2; i++) Serial.print(transmission_packet_2[i]);
  Serial.println();
  Serial.print("Mensaje (8 bits): ");
  for(int i=0; i < MESSAGE_LEN_PKT2; i++) Serial.print(data_message_pkt2[i]);
  Serial.println();
  Serial.print("Confirmacion (8 bits): ");
  for(int i=0; i < CONF_LEN_PKT2; i++) Serial.print(confirmation_bits_pkt2[i]);
  Serial.println();
  Serial.println();
  Serial.println("Transmitiendo paquetes alternadamente...");
  Serial.print("Duracion PKT1: ");
  Serial.print(PACKET_LEN_PKT1 * 8);
  Serial.print("ms, PKT2: ");
  Serial.print(PACKET_LEN_PKT2 * 8);
  Serial.println("ms");
  
  Serial.println();
  Serial.println("=== MODO CALIBRACION ===");
  Serial.println("Enviando secuencia de calibracion: 0,1,0,1");
  Serial.print("Duracion por bit: ");
  Serial.print(CALIBRATION_BIT_DURATION_MS);
  Serial.println("ms");
  Serial.println("Frecuencias PKT1 (para calibrar PKT1):");
  Serial.print("  f0=");
  Serial.print(FREQ_0_PKT1);
  Serial.print("Hz, f1=");
  Serial.print(FREQ_1_PKT1);
  Serial.println("Hz");
  Serial.println();
  
  // Enviar secuencia de calibración para PKT1
  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
    Serial.print("Enviando bit ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(calibration_sequence[i]);
    
    int freq = (calibration_sequence[i] == 1) ? FREQ_1_PKT1 : FREQ_0_PKT1;
    long half_period = 1000000 / (2 * freq);
    unsigned long start_time = millis();
    
    while (millis() - start_time < CALIBRATION_BIT_DURATION_MS) {
      digitalWrite(FSK_PIN, HIGH);
      delayMicroseconds(half_period);
      digitalWrite(FSK_PIN, LOW);
      delayMicroseconds(half_period);
    }
  }
  
  digitalWrite(FSK_PIN, LOW);
  Serial.println("Calibracion PKT1 completada!");
  Serial.println();
  
  // Calibración para PKT2
  Serial.println("Frecuencias PKT2 (para calibrar PKT2):");
  Serial.print("  f0=");
  Serial.print(FREQ_0_PKT2);
  Serial.print("Hz, f1=");
  Serial.print(FREQ_1_PKT2);
  Serial.println("Hz");
  Serial.println();
  
  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
    Serial.print("Enviando bit ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(calibration_sequence[i]);
    
    int freq = (calibration_sequence[i] == 1) ? FREQ_1_PKT2 : FREQ_0_PKT2;
    long half_period = 1000000 / (2 * freq);
    unsigned long start_time = millis();
    
    while (millis() - start_time < CALIBRATION_BIT_DURATION_MS) {
      digitalWrite(FSK_PIN, HIGH);
      delayMicroseconds(half_period);
      digitalWrite(FSK_PIN, LOW);
      delayMicroseconds(half_period);
    }
  }
  
  digitalWrite(FSK_PIN, LOW);
  Serial.println("Calibracion PKT2 completada!");
  Serial.println();
  Serial.println("Iniciando transmision normal...");
  Serial.println("========================================");
  
  calibration_done = true;
}

void loop() {
  unsigned long current_time_us = micros();
  
  // Seleccionar paquete y parámetros actuales
  uint8_t* current_packet_data;
  int current_packet_len;
  
  if (current_packet == 1) {
    current_packet_data = transmission_packet_1;
    current_packet_len = PACKET_LEN_PKT1;
  } else {
    current_packet_data = transmission_packet_2;
    current_packet_len = PACKET_LEN_PKT2;
  }
  
  int freq_0 = (current_packet == 1) ? FREQ_0_PKT1 : FREQ_0_PKT2;
  int freq_1 = (current_packet == 1) ? FREQ_1_PKT1 : FREQ_1_PKT2;
  long half_period_0 = 1000000 / (2 * freq_0);
  long half_period_1 = 1000000 / (2 * freq_1);

  // Inicializar bit si es necesario
  if (bit_start_time_us == 0) {
    bit_start_time_us = current_time_us;
    next_toggle_time_us = current_time_us;
    
    // Reportar solo al inicio de cada paquete
    if (current_bit_index == 0) {
      Serial.print("\n[TX] Paquete ");
      Serial.print(current_packet);
      Serial.print(" (");
      Serial.print(current_packet_len);
      Serial.print(" bits, f0=");
      Serial.print(freq_0);
      Serial.print("Hz, f1=");
      Serial.print(freq_1);
      Serial.println("Hz)");
    }
  }

  // Toggle del pin según la frecuencia del bit actual
  if (current_time_us >= next_toggle_time_us) {
    digitalWrite(FSK_PIN, !digitalRead(FSK_PIN));
    long half_period = (current_packet_data[current_bit_index] == 1) ? half_period_1 : half_period_0;
    next_toggle_time_us += half_period;
  }
  
  // Verificar si el bit actual ha terminado
  if (current_time_us - bit_start_time_us >= BIT_DURATION_US) {
    current_bit_index++;
    bit_start_time_us = 0;
    
    // Si terminamos el paquete actual
    if (current_bit_index >= current_packet_len) {
      current_bit_index = 0;
      // Alternar al siguiente paquete
      current_packet = (current_packet == 1) ? 2 : 1;
    }
  }
}