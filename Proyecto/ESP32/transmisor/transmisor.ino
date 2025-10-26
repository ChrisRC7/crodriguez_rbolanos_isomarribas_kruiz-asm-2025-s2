  /**
  * @file pico_transmisor_diagnostico.ino
  * @brief Transmisor FSK con un patrón de bits alterno para diagnóstico.
  */
  //Rasperry Pi Pico - Transmisor FSK Diagnóstico
  // --- Configuración de Pines ---
  const int FSK_PIN = 15; // GP15

  // --- Parámetros FSK ---
  const int FREQ_0 = 1000;
  const int FREQ_1 = 2000;
  const long BIT_DURATION_US = 8000; // 8 ms

  // --- MENSAJE DE DIAGNÓSTICO ---
  const uint8_t data_bits[] = {1, 0, 1, 0, 1, 0, 1, 0}; // Patrón alterno
  const int num_bits = sizeof(data_bits) / sizeof(data_bits[0]);

  // (El resto del código del transmisor es idéntico al anterior. No necesita cambios)
  const long HALF_PERIOD_0_US = 1000000 / (2 * FREQ_0);
  const long HALF_PERIOD_1_US = 1000000 / (2 * FREQ_1);
  int current_bit_index = 0;
  unsigned long last_transmission_time = 0;
  const long transmission_interval = 1000;
  unsigned long bit_start_time_us = 0;
  unsigned long next_toggle_time_us = 0;
  bool is_transmitting = false;

  void setup() {
    Serial.begin(115200);
    pinMode(FSK_PIN, OUTPUT);
    digitalWrite(FSK_PIN, LOW);
    delay(1000);
    Serial.println("Transmisor FSK (Diagnóstico) iniciado...");
    last_transmission_time = -transmission_interval;
  }

  void loop() {
    unsigned long current_time_ms = millis();
    unsigned long current_time_us = micros();
    if (!is_transmitting && (current_time_ms - last_transmission_time >= transmission_interval)) {
      is_transmitting = true;
      current_bit_index = 0;
      Serial.print("Iniciando transmisión: ");
      for(int i=0; i < num_bits; i++) Serial.print(data_bits[i]);
      Serial.println();
    }
    if (is_transmitting) {
      if (current_bit_index < num_bits && bit_start_time_us == 0) {
        bit_start_time_us = current_time_us;
        next_toggle_time_us = current_time_us;
      }
      if (current_bit_index < num_bits && (current_time_us - bit_start_time_us < BIT_DURATION_US)) {
        if (current_time_us >= next_toggle_time_us) {
          digitalWrite(FSK_PIN, !digitalRead(FSK_PIN));
          long half_period = (data_bits[current_bit_index] == 1) ? HALF_PERIOD_1_US : HALF_PERIOD_0_US;
          next_toggle_time_us += half_period;
        }
      } else if (current_bit_index < num_bits) {
        current_bit_index++;
        bit_start_time_us = 0;
      } else {
        is_transmitting = false;
        digitalWrite(FSK_PIN, LOW);
        last_transmission_time = current_time_ms;
        Serial.println("Transmisión completa.");
      }
    }
  }