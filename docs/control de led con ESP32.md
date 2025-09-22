# Control de LED con ESP32 (botón, Bluetooth e intervalos)  

## 1) Resumen  
**Nombre del proyecto:** Control de LED con ESP32  
**Equipo / Autor(es):** José Ismael Guerrero Román y Gerardo Esquivel De Luna  
**Curso / Asignatura:** Introducción a la Mecatrónica
**Fecha:** 19/09/25
**Descripción breve:** Se programó un microcontrolador ESP32 para controlar un LED en tres casos distintos: mediante un botón físico, a través de comandos enviados por Bluetooth y con parpadeo automático en intervalos de tiempo definidos.  

---

## 2) Objetivos  
**General:**  
Explorar diferentes formas de control de un LED con ESP32 aplicando entradas físicas, comunicación inalámbrica y programación de temporización.  

**Específicos:**  
- OE1: Programar el ESP32 para que un botón controle el encendido y apagado inmediato del LED.  
- OE2: Configurar la comunicación Bluetooth del ESP32 para que el LED responda a comandos enviados desde la terminal (`"on"` y `"off"`).  
- OE3: Implementar un programa que haga parpadear el LED en intervalos de tiempo definidos.  
- OE4: Documentar con fotografías y videos el código y el funcionamiento de cada caso.  

---

## 3) Alcance y Exclusiones  
**Incluye:**  
- Programación del ESP32 en Arduino IDE (o entorno equivalente).  
- Caso 1: Control mediante botón físico.  
- Caso 2: Control mediante Bluetooth con comandos `"on"` y `"off"`.  
- Caso 3: Parpadeo automático del LED en intervalos.  
- Evidencias en fotos del código y videos del funcionamiento.  

**No incluye:**  
- Control de múltiples LEDs o actuadores.  
- Desarrollo de aplicaciones móviles personalizadas.  
- Integración con plataformas IoT externas.  

---

## 4) Requisitos  
**Software:**  
- Arduino IDE con soporte para ESP32.  
- Librerías necesarias para comunicación Bluetooth.  
- Monitor serie o aplicación de terminal Bluetooth.  

**Hardware:**  
- ESP32.  
- LED + resistencia limitadora (330 Ω a 820 Ω).  
- Pulsador (botón).  
- Protoboard y cables de conexión.  
- Fuente de alimentación USB (5 V).  

**Conocimientos previos:**  
- Programación básica en Arduino/ESP32.  
- Manejo de entradas digitales (lectura de botones).  
- Control de salidas digitales (encendido y apagado de LED).  
- Conceptos de comunicación básica vía Bluetooth.  
- Uso de funciones de temporización (`delay`, `millis`, etc.) para parpadeo en intervalos.

---

# Programas ESP32 - Control de LED

Este repositorio contiene tres ejemplos básicos para controlar un LED con un ESP32:  
1. Encender y apagar con un botón físico.  
2. Encender y apagar mediante comandos enviados por Bluetooth.  
3. Hacer parpadear el LED en intervalos de 2 segundos.  

---

##  1. Control con Botón 

```cpp
const int led = 33;
const int btn = 27;

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(btn, INPUT);
}

void loop() {
  int estado = digitalRead(btn);
  if (estado == 1) {
    digitalWrite(led, 1);  // LED encendido mientras el botón esté presionado
  } else {
    digitalWrite(led, 0);  // LED apagado cuando no se presiona
  }
}

```

---

## 2. Control con Bluetooth

```cpp
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int led = 33;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-de-los-PAPITOS"); // Nombre del dispositivo Bluetooth
  pinMode(led, OUTPUT);
}

void loop() {
  if (SerialBT.available()) {
    String mensaje = SerialBT.readString();
    Serial.println("Recibido: " + mensaje);

    if (mensaje == "on") {
      digitalWrite(led, 1);   // LED encendido por comando
    } 
    else if (mensaje == "off") {
      digitalWrite(led, 0);   // LED apagado por comando
    }
  }
  delay(1000);
}
```

---
## 3. Intervalos de Parpadeo

```cpp
const int led = 33;

void setup() {
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);  // LED encendido
  delay(2000);              // Espera 2 segundos
  digitalWrite(led, LOW);   // LED apagado
  delay(2000);              // Espera 2 segundos
}
```

---

[Video de Práctica Y códigos](https://iberopuebla-my.sharepoint.com/:f:/g/personal/203563_iberopuebla_mx/EhkJHZAIqP9FguhG2vSfIz8BWP6sGHuCXWlVYsU_gA8Pzg?e=9fY8qf)

