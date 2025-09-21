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
