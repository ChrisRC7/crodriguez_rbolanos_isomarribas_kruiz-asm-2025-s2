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

// Modos de operación
enum ControlMode {
  MODE_MANUAL,           // Control manual desde consola
  MODE_NO_CONTROL,       // Sin PID - motor apagado
  MODE_BAD_TUNING,       // PID con parámetros mal ajustados
  MODE_GOOD_TUNING       // PID con parámetros bien ajustados
};

ControlMode currentMode = MODE_MANUAL;

// Parámetros del control PID - Buenos
float Kp_good = 2.0;
float Ki_good = 0.1;
float Kd_good = 0.5;

// Parámetros del control PID - Malos (oscilatorio/inestable)
float Kp_bad = 10.0;   // Kp muy alto causa oscilaciones
float Ki_bad = 5.0;    // Ki muy alto causa overshoot
float Kd_bad = 0.0;    // Sin derivativo

// Parámetros activos
float Kp = 2.0;      // Ganancia proporcional
float Ki = 0.1;      // Ganancia integral
float Kd = 0.5;      // Ganancia derivativa

// Variables del PID
float setpoint = 10.0;  // Distancia objetivo en cm
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Variables del sensor
float distance = 0;
unsigned long lastTime = 0;
float dt = 0;

// Variables para control manual
int manualSpeed = 0;  // -255 a 255

// Constantes PWM
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

void setup() {
  Serial.begin(115200);
  
  // Configurar pines del sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configurar pines del motor
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // Configurar PWM para el motor (nueva API ESP32 v3.x)
  ledcAttachPWM_CHANNELWM_FREQ, PWM_RESOLUTION);
  
  // Inicializar motor detenido
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(ENA_PIN, 0);
  
  lastTime = millis();
  
  printMenu();
}

void printMenu() {
  Serial.println("\n==========================================");
  Serial.println("     SISTEMA DE CONTROL PID - MENU");
  Serial.println("==========================================");
  Serial.println("Modos de operacion:");
  Serial.println("  0 - Modo MANUAL");
  Serial.println("  1 - SIN CONTROL (motor apagado)");
  Serial.println("  2 - PID MAL AJUSTADO");
  Serial.println("  3 - PID BIEN AJUSTADO");
  Serial.println("\nControl manual (solo en modo 0):");
  Serial.println("  w - Adelante (incrementar velocidad)");
  Serial.println("  s - Atras (decrementar velocidad)");
  Serial.println("  x - DETENER motor");
  Serial.println("  + - Aumentar velocidad manual");
  Serial.println("  - - Disminuir velocidad manual");
  Serial.println("\nAjustes:");
  Serial.println("  d - Cambiar distancia objetivo (setpoint)");
  Serial.println("  m - Mostrar este menu");
  Serial.println("  i - Informacion actual");
  Serial.println("==========================================\n");
  printCurrentMode();
}

void printCurrentMode() {
  Serial.print("Modo actual: ");
  switch(currentMode) {
    case MODE_MANUAL:
      Serial.println("MANUAL");
      break;
    case MODE_NO_CONTROL:
      Serial.println("SIN CONTROL");
      break;
    case MODE_BAD_TUNING:
      Serial.print("PID MAL AJUSTADO (Kp=");
      Serial.print(Kp);
      Serial.print(", Ki=");
      Serial.print(Ki);
      Serial.print(", Kd=");
      Serial.print(Kd);
      Serial.println(")");
      break;
    case MODE_GOOD_TUNING:
      Serial.print("PID BIEN AJUSTADO (Kp=");
      Serial.print(Kp);
      Serial.print(", Ki=");
      Serial.print(Ki);
      Serial.print(", Kd=");
      Serial.print(Kd);
      Serial.println(")");
      break;
  }
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" cm\n");
}

void loop() {
  // Procesar comandos de la consola
  processSerialCommands();
  
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
  
  // Ejecutar según el modo actual
  switch(currentMode) {
    case MODE_MANUAL:
      // Control manual - usar la velocidad establecida
      controlMotor(manualSpeed);
      printManualStatus();
      break;
      
    case MODE_NO_CONTROL:
      // Sin control - motor apagado
      controlMotor(0);
      printNoControlStatus();
      break;
      
    case MODE_BAD_TUNING:
    case MODE_GOOD_TUNING:
      // Modo PID
      runPIDControl();
      break;
  }
  
  delay(50);  // Pequeño retardo para estabilidad
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case '0':
        currentMode = MODE_MANUAL;
        manualSpeed = 0;
        integral = 0;  // Reset integral
        Serial.println("\n>>> Modo MANUAL activado");
        break;
        
      case '1':
        currentMode = MODE_NO_CONTROL;
        integral = 0;  // Reset integral
        Serial.println("\n>>> Modo SIN CONTROL activado");
        break;
        
      case '2':
        currentMode = MODE_BAD_TUNING;
        Kp = Kp_bad;
        Ki = Ki_bad;
        Kd = Kd_bad;
        integral = 0;  // Reset integral
        Serial.println("\n>>> Modo PID MAL AJUSTADO activado");
        Serial.print("Parametros: Kp=");
        Serial.print(Kp);
        Serial.print(", Ki=");
        Serial.print(Ki);
        Serial.print(", Kd=");
        Serial.println(Kd);
        break;
        
      case '3':
        currentMode = MODE_GOOD_TUNING;
        Kp = Kp_good;
        Ki = Ki_good;
        Kd = Kd_good;
        integral = 0;  // Reset integral
        Serial.println("\n>>> Modo PID BIEN AJUSTADO activado");
        Serial.print("Parametros: Kp=");
        Serial.print(Kp);
        Serial.print(", Ki=");
        Serial.print(Ki);
        Serial.print(", Kd=");
        Serial.println(Kd);
        break;
        
      case 'w':
      case 'W':
        if (currentMode == MODE_MANUAL) {
          manualSpeed += 30;
          if (manualSpeed > 255) manualSpeed = 255;
          Serial.print(">>> Motor ADELANTE, velocidad: ");
          Serial.println(manualSpeed);
        } else {
          Serial.println(">>> Comando solo disponible en modo MANUAL");
        }
        break;
        
      case 's':
      case 'S':
        if (currentMode == MODE_MANUAL) {
          manualSpeed -= 30;
          if (manualSpeed < -255) manualSpeed = -255;
          Serial.print(">>> Motor ATRAS, velocidad: ");
          Serial.println(manualSpeed);
        } else {
          Serial.println(">>> Comando solo disponible en modo MANUAL");
        }
        break;
        
      case 'x':
      case 'X':
        if (currentMode == MODE_MANUAL) {
          manualSpeed = 0;
          Serial.println(">>> Motor DETENIDO");
        } else {
          Serial.println(">>> Comando solo disponible en modo MANUAL");
        }
        break;
        
      case '+':
        if (currentMode == MODE_MANUAL) {
          manualSpeed += 10;
          if (manualSpeed > 255) manualSpeed = 255;
          Serial.print(">>> Velocidad aumentada: ");
          Serial.println(manualSpeed);
        } else {
          Serial.println(">>> Comando solo disponible en modo MANUAL");
        }
        break;
        
      case '-':
        if (currentMode == MODE_MANUAL) {
          manualSpeed -= 10;
          if (manualSpeed < -255) manualSpeed = -255;
          Serial.print(">>> Velocidad disminuida: ");
          Serial.println(manualSpeed);
        } else {
          Serial.println(">>> Comando solo disponible en modo MANUAL");
        }
        break;
        
      case 'd':
      case 'D':
        Serial.println("\n>>> Ingrese nueva distancia objetivo (cm): ");
        while(Serial.available() == 0) {}
        setpoint = Serial.parseFloat();
        Serial.print("Nuevo setpoint: ");
        Serial.print(setpoint);
        Serial.println(" cm");
        integral = 0;  // Reset integral
        break;
        
      case 'm':
      case 'M':
        printMenu();
        break;
        
      case 'i':
      case 'I':
        printInfo();
        break;
    }
  }
}

void runPIDControl() {
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
  printPIDStatus();
}

void printPIDStatus() {
  Serial.print("Dist: ");
  Serial.print(distance);
  Serial.print(" cm | SP: ");
  Serial.print(setpoint);
  Serial.print(" | Err: ");
  Serial.print(error);
  Serial.print(" | P: ");
  Serial.print(Kp * error);
  Serial.print(" | I: ");
  Serial.print(Ki * integral);
  Serial.print(" | D: ");
  Serial.print(Kd * derivative);
  Serial.print(" | Out: ");
  Serial.println(output);
}

void printManualStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("MANUAL - Dist: ");
    Serial.print(distance);
    Serial.print(" cm | Vel: ");
    Serial.println(manualSpeed);
    lastPrint = millis();
  }
}

void printNoControlStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("SIN CONTROL - Dist: ");
    Serial.print(distance);
    Serial.println(" cm | Motor APAGADO");
    lastPrint = millis();
  }
}

void printInfo() {
  Serial.println("\n========== INFORMACION DEL SISTEMA ==========");
  Serial.print("Modo: ");
  switch(currentMode) {
    case MODE_MANUAL: Serial.println("MANUAL"); break;
    case MODE_NO_CONTROL: Serial.println("SIN CONTROL"); break;
    case MODE_BAD_TUNING: Serial.println("PID MAL AJUSTADO"); break;
    case MODE_GOOD_TUNING: Serial.println("PID BIEN AJUSTADO"); break;
  }
  Serial.print("Distancia actual: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" cm");
  Serial.print("Parametros PID: Kp=");
  Serial.print(Kp);
  Serial.print(", Ki=");
  Serial.print(Ki);
  Serial.print(", Kd=");
  Serial.println(Kd);
  if (currentMode == MODE_MANUAL) {
    Serial.print("Velocidad manual: ");
    Serial.println(manualSpeed);
  }
  Serial.println("============================================\n");
}

void controlMotor(float pwmValue) {
  if (pwmValue > 5) {
    // Motor hacia adelante (acercarse al objeto)
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, abs(pwmValue));
  } 
  else if (pwmValue < -5) {
    // Motor hacia atrás (alejarse del objeto)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, abs(pwmValue));
  } 
  else {
    // Zona muerta: detener el motor
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
  }
}
