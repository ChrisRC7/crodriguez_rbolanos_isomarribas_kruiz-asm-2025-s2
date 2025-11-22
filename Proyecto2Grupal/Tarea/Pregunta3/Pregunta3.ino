// ============================================================
// IMPLEMENTACIÓN DE CONTROLADOR PID SIN delay()
//  Basado en Tiempo (No bloqueante)
// ============================================================

#include <Arduino.h>

// Parámetros del PID
float Kp = 1.0;      // Ganancia proporcional
float Ki = 0.1;      // Ganancia integral
float Kd = 0.01;     // Ganancia derivativa

// Variables del PID
float error_prev = 0;
float integral = 0;
float setpoint = 100.0;  // Valor deseado

// Control de tiempo sin bloques
unsigned long tiempo_actual = 0;
unsigned long tiempo_prev = 0;
const unsigned long periodo_muestreo = 2;  // 2 ms para 500 Hz

// Pines
const int pin_sensor = A0;    // Entrada analógica
const int pin_actuador = 9;   // PWM

void setup() {
  Serial.begin(115200);
  pinMode(pin_sensor, INPUT);
  pinMode(pin_actuador, OUTPUT);
  
  tiempo_prev = millis();
}

void loop() {
  tiempo_actual = millis();
  
  // Verificar si es tiempo de ejecutar el PID
  if (tiempo_actual - tiempo_prev >= periodo_muestreo) {
    
    // PASO 1: Lectura del sensor (no bloqueante)
    float valor_sensor = analogRead(pin_sensor) * 5.0 / 1023.0;
    
    // PASO 2: Cálculo del error
    float error = setpoint - valor_sensor;
    
    // PASO 3: Término Proporcional
    float P = Kp * error;
    
    // PASO 4: Término Integral (con anti-windup)
    float dt = (tiempo_actual - tiempo_prev) / 1000.0;  // en segundos
    integral += Ki * error * dt;
    integral = constrain(integral, -255, 255);  // Limitar integral
    
    // PASO 5: Término Derivativo
    float D = Kd * (error - error_prev) / dt;
    
    // PASO 6: Salida PID
    float u = P + integral + D;
    u = constrain(u, 0, 255);  // Limitar a rango PWM
    
    // PASO 7: Escritura del actuador
    analogWrite(pin_actuador, (int)u);
    
    // PASO 8: Actualizar variables para próxima iteración
    error_prev = error;
    tiempo_prev = tiempo_actual;
    
    // PASO 9: Datos para depuración 
    Serial.print(valor_sensor);
    Serial.print(",");
    Serial.print(u);
    Serial.print(",");
    Serial.println(error);
  }
  
  // Otras tareas pueden ejecutarse aquí sin bloques
  // Esta es la ventaja sobre delay()
}
