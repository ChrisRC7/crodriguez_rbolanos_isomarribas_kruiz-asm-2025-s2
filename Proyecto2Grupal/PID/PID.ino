#include <NewPing.h>

// Pines del sensor ultrasónico HC-SR04
#define TRIG_PIN 5
#define ECHO_PIN 18
#define MAX_DISTANCE 400  // Distancia máxima en cm

// Crear objeto del sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Pines del driver de motor L298N
#define ENA_PIN 25   // Pin PWM para controlar velocidad
#define IN1_PIN 26   // Control de dirección
#define IN2_PIN 27   // Control de dirección

// Parámetros del control PID
float Kp = 2.0;      // Ganancia proporcional
float Ki = 0.1;      // Ganancia integral
float Kd = 0.5;      // Ganancia derivativa

// Variables del PID
float setpoint = 15.0;  // Distancia objetivo en cm
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Variables del sensor
float distance = 0;
unsigned long lastTime = 0;
float dt = 0;

// Constantes
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL = 0;

void setup() {
  Serial.begin(115200);
  
  // Configurar pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configurar pines del motor
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  // Configurar PWM para el motor
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL);
  
  // Inicializar motor detenido
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(PWM_CHANNEL, 0);
  
  lastTime = millis();
  
  Serial.println("Sistema PID iniciado");
  Serial.println("Setpoint: " + String(setpoint) + " cm");
}

void loop() {
  // Calcular tiempo transcurrido
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // Convertir a segundos
  lastTime = currentTime;
  
  // Leer distancia del sensor
  distance = sonar.ping_cm();
  
  // Si la lectura es 0 (fuera de rango), mantener la última lectura válida
  if (distance == 0) {
    distance = lastError + setpoint;  // Usar la última distancia conocida
  }
  
  // Calcular error
  error = setpoint - distance;
  
  // Calcular término integral
  integral += error * dt;
  
  // Anti-windup: limitar el término integral
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  
  // Calcular término derivativo
  if (dt > 0) {
    derivative = (error - lastError) / dt;
  }
  
  // Calcular salida PID
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Limitar la salida
  if (output > 255) output = 255;
  if (output < -255) output = -255;
  
  // Controlar el motor según la salida
  controlMotor(output);
  
  // Guardar error actual para la próxima iteración
  lastError = error;
  
  // Imprimir información de depuración
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.print(" cm | Error: ");
  Serial.print(error);
  Serial.print(" | Output: ");
  Serial.println(output);
  
  delay(50);  // Pequeño retardo para estabilidad
}

void controlMotor(float pwmValue) {
  if (pwmValue > 5) {
    // Motor hacia adelante (acercarse al objeto)
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL, abs(pwmValue));
  } 
  else if (pwmValue < -5) {
    // Motor hacia atrás (alejarse del objeto)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(PWM_CHANNEL, abs(pwmValue));
  } 
  else {
    // Zona muerta: detener el motor
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}
