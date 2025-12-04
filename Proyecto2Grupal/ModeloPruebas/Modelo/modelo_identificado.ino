/*
 * Modelo del Motor DC - Arduino
 * Generado automáticamente
 * 
 * Modelo: G(s) = 999.3659 / (0.3008*s + 1)
 * Frecuencia de muestreo: 5.30 Hz (188.65 ms)
 */

// ============================================================================
// PARÁMETROS DEL MODELO
// ============================================================================
const float K_MOTOR = 999.3659;      // Ganancia [RPM/V]
const float TAU_MOTOR = 0.3008;  // Constante de tiempo [s]
const float TS_SAMPLE = 0.1887;             // Período de muestreo [s]
const unsigned long TS_MS = 189;      // Período de muestreo [ms]

// Coeficientes discretos
const float A1 = 0.522515;
const float B0 = 1192.956288;
const float B1 = 1192.956288;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
float y_prev = 0.0;  // Velocidad anterior [RPM]
float u_prev = 0.0;  // Entrada anterior [V]

unsigned long tiempo_anterior = 0;

// ============================================================================
// FUNCIONES
// ============================================================================

void modelo_init() {
    y_prev = 0.0;
    u_prev = 0.0;
}

float modelo_update(float u) {
    // Ecuación en diferencias
    float y = A1 * y_prev + B0 * u + B1 * u_prev;
    
    // Actualizar estados
    y_prev = y;
    u_prev = u;
    
    return y;
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    // Inicializar modelo
    modelo_init();
    
    Serial.println("Modelo del Motor DC inicializado");
    Serial.print("Frecuencia de muestreo: ");
    Serial.print(1000.0 / TS_MS);
    Serial.println(" Hz");
    
    tiempo_anterior = millis();
}

void loop() {
    unsigned long tiempo_actual = millis();
    
    // Ejecutar cada TS_MS milisegundos
    if (tiempo_actual - tiempo_anterior >= TS_MS) {
        tiempo_anterior = tiempo_actual;
        
        // Leer entrada (ejemplo: PWM a voltaje)
        float pwm = analogRead(A0);
        float voltaje = (pwm / 1023.0) * 5.0;  // Convertir a voltios
        
        // Actualizar modelo
        float velocidad_estimada = modelo_update(voltaje);
        
        // Enviar datos por serial
        Serial.print(tiempo_actual);
        Serial.print(",");
        Serial.print(voltaje, 4);
        Serial.print(",");
        Serial.println(velocidad_estimada, 2);
    }
}
