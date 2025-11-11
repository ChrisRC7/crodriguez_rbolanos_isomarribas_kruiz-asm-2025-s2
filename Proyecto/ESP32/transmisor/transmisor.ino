/**
 * @file transmisor.ino (Raspberry Pi Pico)
 * @brief Transmisor FSK dual que envía 2 paquetes alternadamente:
 *        - Paquete 1: f0=1000Hz, f1=2000Hz (mensaje de 4 bits)
 *        - Paquete 2: f0=3000Hz, f1=4000Hz (carácter ASCII convertido a 8 bits)
 *        Incluye modo de calibración previo.
 */

// --- Configuración de Pines ---
const int FSK_PIN = 0; // GP0 en Raspberry Pi Pico

// --- Parámetros FSK Paquete 1 ---
const int FREQ_0_PKT1 = 1000;
const int FREQ_1_PKT1 = 2000;

// --- Parámetros FSK Paquete 2 ---
const int FREQ_0_PKT2 = 3000;
const int FREQ_1_PKT2 = 4000;

// Duración de bit (común)
const unsigned long BIT_DURATION_US = 8000; // 8 ms

// --- Modo de Calibración ---
const unsigned long CALIBRATION_BIT_DURATION_MS = 3000; // 3 s por bit
const uint8_t calibration_sequence[] = {0, 1, 0, 1};
const int CALIBRATION_LENGTH = 4;
bool calibration_done = false;

// ============================
// Configuración KEYPAD 4x4
// ============================
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Pines de las filas (INPUT con pull-down interno)
const byte rowPins[ROWS] = {6, 7, 8, 9};
// Pines de las columnas (OUTPUT)
const byte colPins[COLS] = {10, 11, 12, 13};
char last_key = 0;
bool transmitting = false; // Cuando false, no se emite nada
char current_char = '0';   // Carácter vigente para PKT2
unsigned long last_key_check = 0;
unsigned long last_key_time = 0;
const unsigned long DEBOUNCE_DELAY_US = 300000; // 300 ms

// --- Componentes del Paquete 1 ---
const uint8_t data_message_pkt1[4] = {0, 1, 0, 1};
const uint8_t confirmation_bits_pkt1[4] = {1, 0, 1, 0};
const int MESSAGE_LEN_PKT1 = 4;
const int PACKET_LEN_PKT1 = 13;  // 1 sync + 4 msg + 4 conf + 4 checksum

// --- Componentes del Paquete 2 (Carácter ASCII) ---
char ascii_char = 'A'; // Carácter a enviar
uint8_t data_message_pkt2[8];    // 8 bits para ASCII
const uint8_t confirmation_bits_pkt2[4] = {1, 0, 1, 0};
const int MESSAGE_LEN_PKT2 = 8;
const int CONF_LEN_PKT2 = 4;
const int PACKET_LEN_PKT2 = 17;  // 1 sync + 8 msg + 4 conf + 4 checksum

// --- Paquetes de Transmisión ---
uint8_t transmission_packet_1[PACKET_LEN_PKT1];
uint8_t transmission_packet_2[PACKET_LEN_PKT2];

// --- Control de Transmisión ---
int current_packet = 1; // 1 o 2
int current_bit_index = 0;
unsigned long bit_start_time_us = 0;
unsigned long next_toggle_time_us = 0;

// --- Prototipos ---
void runCalibration();
char scan_keypad();

// --- Función para convertir carácter ASCII a 8 bits ---
void asciiTo8Bits(char c, uint8_t* bits) {
  uint8_t value = (uint8_t)c;
  for (int i = 0; i < 8; i++) {
    bits[i] = (value >> (7 - i)) & 1;
  }
}

// --- Función para construir un paquete con msg_len = conf_len = checksum_len ---
void buildPacket(uint8_t* packet, const uint8_t* data_msg, const uint8_t* conf_bits, int msg_len) {
  packet[0] = 1; // Bit de sincronización
  for (int i = 0; i < msg_len; i++) {
    packet[1 + i] = data_msg[i];                    // Mensaje
    packet[1 + i + msg_len] = conf_bits[i];         // Confirmación
    packet[1 + i + 2 * msg_len] = data_msg[i] ^ conf_bits[i]; // Checksum XOR
  }
}

// --- Función para construir paquete 2 (checksum de 4 bits) ---
void buildPacket2(uint8_t* packet, const uint8_t* data_msg, const uint8_t* conf_bits, int msg_len, int conf_len) {
  packet[0] = 1; // Bit de sincronización

  // Mensaje de 8 bits
  for (int i = 0; i < msg_len; i++) {
    packet[1 + i] = data_msg[i];
  }

  // Confirmación de 4 bits
  for (int i = 0; i < conf_len; i++) {
    packet[1 + msg_len + i] = conf_bits[i];
  }

  // Checksum: XOR entre los primeros 4 bits del mensaje y los 4 de confirmación
  for (int i = 0; i < conf_len; i++) {
    packet[1 + msg_len + conf_len + i] = data_msg[i] ^ conf_bits[i];
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(FSK_PIN, OUTPUT);
  digitalWrite(FSK_PIN, LOW);
  delay(1000);

  // Configurar keypad
  for (int i = 0; i < ROWS; i++) {
    pinMode(rowPins[i], INPUT_PULLDOWN);
  }
  for (int i = 0; i < COLS; i++) {
    pinMode(colPins[i], OUTPUT);
    digitalWrite(colPins[i], LOW);
  }

  // Construir Paquete 1
  buildPacket(transmission_packet_1, data_message_pkt1, confirmation_bits_pkt1, MESSAGE_LEN_PKT1);

  // Construir Paquete 2
  ascii_char = current_char; // inicial
  asciiTo8Bits(ascii_char, data_message_pkt2);
  buildPacket2(transmission_packet_2, data_message_pkt2, confirmation_bits_pkt2, MESSAGE_LEN_PKT2, CONF_LEN_PKT2);

  Serial.println("=== Transmisor FSK Dual (Raspberry Pi Pico) ===");
  Serial.println();

  // Información Paquete 1
  Serial.println("--- PAQUETE 1 (13 bits) ---");
  Serial.print("Frecuencias: f0=");
  Serial.print(FREQ_0_PKT1);
  Serial.print("Hz, f1=");
  Serial.print(FREQ_1_PKT1);
  Serial.println("Hz");
  Serial.print("Paquete: ");
  for (int i = 0; i < PACKET_LEN_PKT1; i++) Serial.print(transmission_packet_1[i]);
  Serial.println();
  Serial.print("Mensaje (4 bits): ");
  for (int i = 0; i < MESSAGE_LEN_PKT1; i++) Serial.print(data_message_pkt1[i]);
  Serial.println();

  // Información Paquete 2
  Serial.println();
  Serial.println("--- PAQUETE 2 (17 bits) ---");
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
  for (int i = 0; i < PACKET_LEN_PKT2; i++) Serial.print(transmission_packet_2[i]);
  Serial.println();
  Serial.print("Mensaje (8 bits): ");
  for (int i = 0; i < MESSAGE_LEN_PKT2; i++) Serial.print(data_message_pkt2[i]);
  Serial.println();
  Serial.print("Confirmacion (4 bits): ");
  for (int i = 0; i < CONF_LEN_PKT2; i++) Serial.print(confirmation_bits_pkt2[i]);
  Serial.println();
  Serial.println();
  Serial.println("Transmitiendo paquetes alternadamente...");
  Serial.print("Duracion PKT1: ");
  Serial.print(PACKET_LEN_PKT1 * 8);
  Serial.print("ms, PKT2: ");
  Serial.print(PACKET_LEN_PKT2 * 8);
  Serial.println("ms");
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  - Presiona 'R' por Serial para MODO CALIBRACION");
  Serial.println("  - Usa el keypad (0-9, A-D, #) para PKT2; '*' pausa la transmisión");
  Serial.println();
  Serial.println("Esperando entrada... (pausa por defecto, presiona una tecla para transmitir)");
  transmitting = false; // iniciar pausado
}

void loop() {
  // Comando de calibración por Serial en cualquier momento
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      transmitting = false;
      digitalWrite(FSK_PIN, LOW);
      runCalibration();
      // Tras calibración, permanecer pausado hasta nueva tecla
    }
  }

  // Keypad: escaneo con debounce
  unsigned long t = micros();
  if ((long)(t - last_key_check) > 50000) { // cada 50 ms
    last_key_check = t;
    char key = scan_keypad();
    if (key != 0 && key != last_key && (t - last_key_time > DEBOUNCE_DELAY_US)) {
      last_key = key;
      last_key_time = t;
      if (key == '*') {
        transmitting = false;
        digitalWrite(FSK_PIN, LOW);
        Serial.println("[Keypad] Pausa de transmisión ('*')");
      } else {
        current_char = key;
        ascii_char = current_char;
        asciiTo8Bits(ascii_char, data_message_pkt2);
        buildPacket2(transmission_packet_2, data_message_pkt2, confirmation_bits_pkt2, MESSAGE_LEN_PKT2, CONF_LEN_PKT2);
        transmitting = true;
        Serial.print("[Keypad] Nuevo PKT2 con char: ");
        Serial.println(current_char);
      }
    }
    if (key == 0) last_key = 0;
  }

  if (!transmitting) {
    // No transmitir nada mientras está pausado
    return;
  }

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

  // Inicializar bit
  if (bit_start_time_us == 0) {
    bit_start_time_us = current_time_us;
    next_toggle_time_us = current_time_us;

    // Reporte al inicio de cada paquete
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

  // Toggle del pin según el bit actual
  if (current_time_us >= next_toggle_time_us) {
    digitalWrite(FSK_PIN, !digitalRead(FSK_PIN));
    long half_period = (current_packet_data[current_bit_index] == 1) ? half_period_1 : half_period_0;
    next_toggle_time_us += half_period;
  }

  // Avance de bit
  if (current_time_us - bit_start_time_us >= BIT_DURATION_US) {
    current_bit_index++;
    bit_start_time_us = 0;

    // Si terminamos el paquete actual
    if (current_bit_index >= current_packet_len) {
      current_bit_index = 0;
      current_packet = (current_packet == 1) ? 2 : 1; // Alternar paquete
    }
  }
}

// --- Función de escaneo de keypad ---
char scan_keypad() {
  for (int col_index = 0; col_index < COLS; col_index++) {
    digitalWrite(colPins[col_index], HIGH);
    delayMicroseconds(200);
    for (int row_index = 0; row_index < ROWS; row_index++) {
      if (digitalRead(rowPins[row_index]) == HIGH) {
        char key = keys[row_index][col_index];
        delayMicroseconds(500);
        if (digitalRead(rowPins[row_index]) == HIGH) {
          digitalWrite(colPins[col_index], LOW);
          return key;
        }
      }
    }
    digitalWrite(colPins[col_index], LOW);
    delayMicroseconds(50);
  }
  return 0;
}

// --- Ejecutar calibración cuando se pida con 'R' ---
void runCalibration() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("=== MODO CALIBRACION (Presionado 'R') ===");
  Serial.println("Secuencia: 0,1,0,1 (3s por bit)");
  Serial.println("========================================");

  // PKT1
  Serial.println("[PKT1] Calibrando 1kHz y 2kHz...");
  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
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
  Serial.println("[PKT1] Calibracion completada!");

  delay(300);

  // PKT2
  Serial.println("[PKT2] Calibrando 3kHz y 4kHz...");
  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
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
  Serial.println("[PKT2] Calibracion completada!");
  Serial.println("========================================");
}