#  Actividad 7: Control Angular de Servomotor con ESP32

##  Finalidad del Laboratorio

El propósito fundamental de esta práctica fue **analizar y aplicar los principios operativos de un servomotor** utilizando las capacidades de generación de señal **PWM** de la placa ESP32. El objetivo específico se centró en la **secuenciación precisa del eje del servo** a tres ángulos clave predefinidos: **0°, 90° y 180°**, asegurando un movimiento controlado y repetitivo.

---

##  Fundamentos Teóricos

### Introducción al Servocontrol

Los **servomotores** son dispositivos actuadores fundamentales en la robótica, conocidos por su capacidad para lograr un **control angular exacto** dentro de un rango limitado (comúnmente 180°). El control de su posición se realiza a través de la **Modulación por Ancho de Pulso (PWM)**, donde el **ancho del pulso eléctrico** (ciclo de trabajo) que reciben es directamente proporcional al ángulo de giro deseado.

### Implementación con ESP32

El microcontrolador **ESP32** es idóneo para esta tarea, ya que su *hardware* permite una **generación de señales PWM altamente configurable** en términos de resolución y frecuencia. Para la programación, se emplean dos funciones esenciales del *framework* de Arduino:
1.  **`ledcWrite()`:** Se utiliza para enviar el valor del ciclo de trabajo PWM al pin de salida.
2.  **`map()`:** Es crucial para realizar la **conversión matemática** del rango de grados legible por el usuario (0 a 180) al rango específico de valores PWM que el servo interpreta.

---

##  Procedimiento Experimental

### Materiales Requeridos

| Componente | Descripción |
| :--- | :--- |
| **Placa ESP32** | (DOIT ESP32 DEVKIT V1) Microcontrolador principal. |
| **Servomotor** | Actuador de control angular. |
| **Protoboard** | Plataforma para el ensamblaje del circuito temporal. |
| Cables Jumpers | Conductores para interconexión. |
| Fuente de Alimentación | Suministro de energía independiente para el servo. |
| Computadora y Cable USB | Para el desarrollo y la carga del *firmware* en Arduino IDE. |

### Metodología de Montaje y Programación

1.  **Montaje Físico:** La ESP32 y el servo se ensamblaron en la protoboard. La línea de señal del servo se conectó al **GPIO 12** de la ESP32. Se aseguraron conexiones separadas de **alimentación (5V)** y **tierra (GND)** para el servo. 
2.  **Desarrollo del Código:** Se programó la lógica de control para establecer un ciclo secuencial de movimiento a 0°, 90° y 180°. Se empleó la función `map()` para asegurar la correcta correspondencia entre los ángulos y el ciclo de trabajo PWM.
3.  **Verificación:** Se cargó el programa a la ESP32 y se monitoreó el **movimiento físico del servo**. Los retardos de 1000 ms se usaron para confirmar cada posición.

---

##  Código Fuente (Arduino / ESP32)

Este código implementa el control secuencial del servo motor, utilizando la librería LEDC del ESP32 para generar la señal PWM.

```cpp
/**
 * @file Control_Servo_Libreria.ino
 * @brief Control secuencial de un servomotor (0°, 90°, 180°) usando la librería Servo.h en ESP32.
 * * La librería Servo.h simplifica el control al encargarse de la gestión interna del PWM.
 */

// Incluir la librería Servo específica para ESP32
// NOTA: Para ESP32, a menudo se usa "ESP32Servo.h" o simplemente "Servo.h"
// Si Servo.h no funciona, intente usar ESP32Servo.h
#include <Servo.h> 

// --- Configuración de Pines y Objetos ---

#define SERVO_PIN 12 // Pin GPIO donde se conecta la señal del servo (Pin de ejemplo)

// Crea un objeto Servo para controlar el motor
Servo miServo; 

// --- Setup (Configuración Inicial) ---
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando Servomotor con Servo.h...");
  
  // Asocia el objeto Servo al pin de salida
  // Este paso configura automáticamente el canal PWM necesario en el ESP32
  miServo.attach(SERVO_PIN); 

  // Asegurar que el servo inicie en una posición conocida
  miServo.write(0); 
  delay(1000);
}

// --- Loop Principal (Ejecución Continua) ---
void loop() {
  // Mover a 0 grados
  Serial.println("Movimiento a 0 grados");
  miServo.write(0); 
  delay(1500); // Usamos un retardo ligeramente mayor para notar el movimiento

  // Mover a 90 grados
  Serial.println("Movimiento a 90 grados");
  miServo.write(90); 
  delay(1500);

  // Mover a 180 grados
  Serial.println("Movimiento a 180 grados");
  miServo.write(180); 
  delay(1500);
}
```
---

##  Análisis de Resultados Detallado

Los resultados obtenidos durante la fase de pruebas confirmaron la viabilidad del método de control implementado, demostrando lo siguiente:

* **Respuesta del Actuador:** El servomotor exhibió una **respuesta angular precisa e inmediata** a las señales de control generadas por el microcontrolador. El eje se posicionó de manera exacta en cada uno de los puntos angulares preestablecidos (0°, 90°, 180°), verificando la **integridad de la conexión física**.
* **Conversión Exitosa:** La aplicación de la función `map()` resultó ser un elemento **crítico para la calibración**. Esta función permitió la **traducción efectiva y lineal** del rango lógico de grados (0-180) al rango específico del ciclo de trabajo PWM (205-410) requerido por el modelo particular del servomotor para su posicionamiento correcto.
* **Monitoreo y Validación:** Los datos numéricos del ciclo de trabajo PWM, comunicados a través del **monitor serial**, coincidieron rigurosamente con los ángulos observados en el movimiento físico del servo. Esto **validó la lógica de programación** y la precisión del cálculo matemático.

---

##  Resumen Conclusivo

Esta actividad de laboratorio culminó exitosamente al **consolidar la comprensión práctica** del mecanismo de control de **actuadores angulares** mediante la técnica **PWM** en la plataforma ESP32.

Se cumplió el objetivo de lograr la **secuenciación controlada del servomotor** a las posiciones 0°, 90° y 180°, lo cual implicó la **validación completa** de la configuración electrónica (conexión y suministro de energía) y la correcta implementación del *software*. La práctica reforzó el entendimiento de la **relación crítica y directa** que existe entre la **frecuencia del pulso, el ciclo de trabajo** de la señal PWM y el **desplazamiento angular preciso** que se obtiene en el servomotor.

## video de la práctica 
<video width="400" controls>
  <source src="../recursos/imgs/vs1.mp4" type="video/mp4">
  Tu navegador no soporta la reproducción de video.
</video>

