# Prácticas – Control de Motores DC con ESP32 y Puente H

---

##  Práctica 1 – Control de dirección de un motor DC con ESP32 y puente H

### 1) Resumen  
**Nombre del proyecto:** Control de Dirección de Motor DC  
**Equipo / Autor(es):**  José Ismael Guerrero Román y Gerardo Esquivel De Luna 
**Curso / Asignatura:** Introducción a la Mecatrónica  
**Fecha:**   03/oct/25
**Descripción breve:** Se controló un motor de corriente directa (DC) mediante un ESP32 y un puente H (L298N), alternando su sentido de giro hacia adelante y hacia atrás, con pausas de detención entre cada cambio.

---

### 2) Objetivos  
**General:**  
Controlar la dirección de rotación de un motor DC mediante el uso de un puente H controlado por el ESP32.  

**Específicos:**  
- OE1: Configurar los pines del ESP32 como salidas digitales.  
- OE2: Programar el control de sentido (adelante/atrás).  
- OE3: Implementar pausas de parada entre los cambios de dirección.  

---

### 3) Alcance y Exclusiones  
**Incluye:**  
- Control de dirección de un solo motor DC.  
- Implementación con puente H L298N.  
- Alimentación de 6 V para el motor.  

**No incluye:**  
- Control de velocidad (PWM).  
- Sensores o control automático.  
- Comunicación con otros dispositivos.  

---

### 4) Requisitos  

**Software:**  
- Arduino IDE 2.x o superior.  
- Librería ESP32 (instalada desde el Gestor de Placas).  

**Hardware:**  
- ESP32 DevKit.  
- Puente H L298N.  
- Motor DC de 6 V.  
- Fuente externa de 6 V.  
- Cables Dupont / protoboard.  

**Conocimientos previos:**  
- Programación básica en Arduino.  
- Manejo de señales digitales.  
- Conceptos de puente H.  

---

### 5) Código

```cpp


#define in1 27
#define in2 14

void setup() {
  /* Declarar pines como salida */
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  /* ADELANTE */
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(1000);

  /* ALTO */
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);

  /* ATRÁS */
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(1000);

  /* ALTO */
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
}
```
---
## videos 
<video width="640" height="360" controls>
  <source src="../recursos/imgs/motor1.mp4" type="video/mp4">
  Tu navegador no soporta la reproducción de video.
</video>

---
##  Práctica 2 – Control de potencia de un motor DC con ESP32 y PWM

### 1) Resumen  
**Nombre del proyecto:** Control de Potencia de Motor DC  
**Equipo / Autor(es):** José Ismael Guerrero Román y Gerardo Esquivel De Luna  
**Curso / Asignatura:** Introducción a la Mecatrónica  
**Fecha:** 10/oct/25  
**Descripción breve:** Se utilizó un ESP32 y un puente H (L298N) alimentado a 6 V para controlar la velocidad de un motor DC mediante modulación por ancho de pulso (PWM), variando progresivamente su potencia desde el valor máximo hasta el mínimo.

---

### 2) Objetivos  
**General:**  
Regular la velocidad de un motor DC utilizando señales PWM generadas por el ESP32.  

**Específicos:**  
- OE1: Configurar un canal PWM en el ESP32 con la frecuencia y resolución adecuadas.  
- OE2: Implementar un programa que incremente y luego disminuya la velocidad del motor.  
- OE3: Observar y analizar la respuesta del motor ante los diferentes niveles de potencia.  

---

### 3) Alcance y Exclusiones  
**Incluye:**  
- Control de velocidad mediante modulación PWM.  
- Configuración y uso del puente H con alimentación de 6 V.  
- Conexión entre el ESP32, el puente H y el motor DC.  

**No incluye:**  
- Cambio de dirección del motor.  
- Lectura de sensores de velocidad o retroalimentación.  
- Control remoto o comunicación inalámbrica.  

---

### 4) Requisitos  

**Software:**  
- Arduino IDE 2.x o superior.  
- Librería del ESP32 instalada desde el Gestor de Placas.  

**Hardware:**  
- ESP32 DevKit.  
- Puente H L298N.  
- Motor DC de 6 V.  
- Fuente externa de 6 V.  
- Protoboard y cables Dupont.  

**Conocimientos previos:**  
- Programación básica en Arduino y ESP32.  
- Conceptos de modulación por ancho de pulso (PWM).  
- Electrónica básica y conexiones con puente H.  

---

### 5) Código

```cpp
#define IN1 27
#define IN2 14
#define PWM_PIN 12

// Configuración del canal PWM
#define PWM_CHANNEL 0
#define PWM_FREQ 1000   // Frecuencia de 1 kHz
#define PWM_RES 8       // Resolución de 8 bits (valores 0 a 255)

void setup() {
  /* Configurar pines de salida */
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  /* Configurar canal PWM */
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);

  /* Asociar el pin físico al canal PWM */
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
}

void loop() {
  /* GIRO HACIA ADELANTE */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  /* Aumentar gradualmente la velocidad */
  for (int i = 0; i <= 255; i++) {
    ledcWrite(PWM_CHANNEL, i);
    delay(10);  // Aumento progresivo
  }

  /* Mantener velocidad máxima */
  delay(1000);

  /* Disminuir gradualmente la velocidad */
  for (int i = 255; i >= 0; i--) {
    ledcWrite(PWM_CHANNEL, i);
    delay(10);  // Disminución progresiva
  }

  /* Detener el motor */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
}
```
---
## videos 
<video width="640" height="360" controls>
  <source src="../recursos/imgs/motor2.mp4" type="video/mp4">
  Tu navegador no soporta la reproducción de video.
</video>

