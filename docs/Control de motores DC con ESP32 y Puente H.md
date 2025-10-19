# Prácticas – Control de Motores DC con ESP32 y Puente H

---

## 🧩 Práctica 1 – Control de dirección de un motor DC con ESP32 y puente H

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

```bash
# 1) Clonar
git clone https://github.com/<usuario>/<repo>.git
cd <repo>

# 2) Abrir el archivo .ino en Arduino IDE
# 3) Conectar el ESP32 por USB y seleccionar el puerto correcto
# 4) Subir el programa

